#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯感知节点 - 基于NCSC2024源程序main.py
- 激光雷达锥桶检测与聚类（使用简单聚类替代KMeans）
- 内外道分离与目标点计算
- 发布目标点话题供控制节点使用
- V7: 增加了 C++ 方案的扇形检测逻辑，用于区分黄线弯和直角弯

================================================================================
/target 话题协议（与 control_node.py 共用，改动需同步）
================================================================================
话题类型: std_msgs/Float32MultiArray
data 字段格式:
    data[0] = target_x  (车体坐标系下目标点的 x，或下列特殊魔法数字之一)
    data[1] = target_y  (target_x 是魔法数字时此字段含义见各模式说明)

正常模式：
    data[0], data[1] 为车体坐标系下的目标点坐标（米），control_node 用 PID
    跟踪此点。

特殊模式（魔法数字 + 触发条件 + 控制行为）：
  ┌─────────┬──────────────────┬────────────────────────────────────────────┐
  │ data[0] │ 模式名           │ 含义                                        │
  ├─────────┼──────────────────┼────────────────────────────────────────────┤
  │ -10000  │ YELLOW_CURVE     │ 扇形检测无锥桶（黄线弯）                    │
  │         │                  │ 触发: 车前方扇形 ROI 内有效锥桶数 < 阈值    │
  │         │                  │ 控制: 锁定左转，直到再次识别到有效目标点    │
  ├─────────┼──────────────────┼────────────────────────────────────────────┤
  │ -20000  │ RIGHT_ANGLE      │ 感知失败/直角弯                             │
  │         │                  │ 触发: 内外道点几何特征异常（详见代码）      │
  │         │                  │ 控制: 固定大角度左转 + 固定速度过弯         │
  ├─────────┼──────────────────┼────────────────────────────────────────────┤
  │ -30000  │ RIGHT_CORRECTION │ 三等分点为空（内外道配对失败）              │
  │         │                  │ 触发: gen3Equinoxes 输出为空                │
  │         │                  │ 控制: 向右修正                              │
  ├─────────┼──────────────────┼────────────────────────────────────────────┤
  │ -40000  │ KEEP_LAST        │ 保持上一非停车状态                          │
  │         │                  │ 触发: 内道点不足但外道点中等(5~6个)         │
  │         │                  │ 控制: 沿用 last_normal_theta 与当前档速     │
  ├─────────┼──────────────────┼────────────────────────────────────────────┤
  │ -99999  │ TRAFFIC_LIGHT    │ 红灯识别区（第一个停车区域）                │
  │         │                  │ 触发: odom 进入预设红灯区坐标范围           │
  │         │                  │ 控制: 停车 3 秒 → 解锁继续                  │
  ├─────────┼──────────────────┼────────────────────────────────────────────┤
  │ -88888  │ PARK_ZONE        │ 第二个停车区域（精确停车）                  │
  │         │                  │ 触发: odom 进入 (-9.25, -6.86) 附近          │
  │         │                  │ 控制: 进入精确停车微调模式（当前已注释）    │
  └─────────┴──────────────────┴────────────────────────────────────────────┘

注意:
  1. 上述坐标 (-9.25, -6.86) 是 2024 国赛赛道（19 届）的硬编码值，
     20 届有 A/B/C/D 四种赛道，比赛当天定。绝对坐标硬编码在新赛道下会
     立即翻车 → 改造时需用 reset-odom 后的相对坐标替代。
  2. 数值匹配建议用 abs(data[0] - magic) < 0.5 而非 ==，防止浮点精度问题。
  3. 改这里的协议时，必须同步修改:
       - control_node.py 同步注释块
       - track_memory_recorder.py 的 MODE_MAP
       - track_memory_viewer.py 的 MODE_COLORS
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from nav_msgs.msg import Odometry
from threading import Lock
import numpy as np
import math
import copy
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge


class PerceptionNode(Node):
    """纯感知节点 - 锥桶检测与目标点计算（使用简单聚类）"""
    
    def __init__(self):
        super().__init__('perception_node')
        
        # --- 声明参数（基于源程序main.py）---
        # ====================== V29 修复：添加黄线弯参数 ======================
        self.declare_parameter('no_cone_detection_range', 2.0)    # 扇形检测距离(m)
        self.declare_parameter('no_cone_sector_angle', 84.0)      # 扇形检测半角(度)
        self.declare_parameter('no_cone_sector_threshold', 1)     # 扇形区域内的噪声点容忍数量
        # ====================== 修复结束 ======================

        # 激光雷达滤波参数
        self.declare_parameter('min_range', 0.2)          # 最小有效极径
        self.declare_parameter('max_range', 3.5)          # 最大有效极径
        self.declare_parameter('left_width', 2.5)         # 矩形滤波左侧宽度
        self.declare_parameter('right_width', 1.0)        # 矩形滤波右侧宽度
        self.declare_parameter('forward_length', 2.5)     # 矩形滤波前方长度
        self.declare_parameter('back_length', 0.5)        # 矩形滤波后方长度
        
        # 简单聚类参数（替代KMeans）
        self.declare_parameter('cluster_radius', 0.25)      # 聚类半径（米），默认0.25（比0.2更宽松）
        self.declare_parameter('min_points_per_cluster', 5)  # 每个簇最少点数，默认5（比10更宽松）
        
        # 内外道分离参数
        self.declare_parameter('inner_dist_sq', 1.69)     # 内道扩展距离平方（1.3^2）
        self.declare_parameter('inner_y_threshold_sq', 1.21)  # 内道扩展Y方向距离平方（1.1^2），限制新点在旧点左右方向1.1米内
        self.declare_parameter('cut_length_y', 0.8)       # 削除圆的直边到圆心Y距离
        self.declare_parameter('cut_length_x', -0.7)      # 削除圆的直边到圆心X距离
        self.declare_parameter('inner_to_outer_x_threshold', 0.3)  # 内道点转外道的X方向阈值（前后0.3米）
        self.declare_parameter('inner_to_outer_y_threshold', 1.3)  # 内道点转外道的Y方向阈值（左侧1.3米内）
        
        # 圆形滤波参数
        self.declare_parameter('max_radius', 7.0)         # 圆形滤波最大半径
        self.declare_parameter('max_y_filter', 0.5)       # 外道Y坐标过滤阈值
        self.declare_parameter('min_inner_y_threshold', 0.8)  # 内道点不足时的Y坐标过滤阈值
        
        # 目标点选择参数
        self.declare_parameter('target_jump', 3)          # 目标点选择跳数（跨越2个锥桶，选择第3个点）
        
        # 直角弯几何检测参数
        self.declare_parameter('wall_detection_min_distance', 1.7)  # 墙检测最小距离（米）
        self.declare_parameter('wall_detection_max_distance', 1.9)  # 墙检测最大距离（米，控制旋钮）
        
        # 可视化参数
        self.declare_parameter('enable_mapping', True)   # 是否启用可视化（默认启用）
        self.declare_parameter('mapping_interval', 1)     # 绘图间隔（帧数，1=每帧都绘制）
        self.declare_parameter('save_path', '/tmp/map.png')  # 图片保存路径
        self.declare_parameter('show_window', True)       # 是否实时弹窗显示
        
        # 获取参数
        # ====================== V29 修复：获取黄线弯参数 ======================
        self.no_cone_detection_range = self.get_parameter('no_cone_detection_range').value
        self.no_cone_sector_angle = self.get_parameter('no_cone_sector_angle').value
        self.no_cone_sector_angle_rad = self.no_cone_sector_angle * (math.pi / 180.0)
        self.no_cone_sector_threshold = self.get_parameter('no_cone_sector_threshold').value
        # ====================== 修复结束 ======================
        
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.left_width = self.get_parameter('left_width').value
        self.right_width = self.get_parameter('right_width').value
        self.forward_length = self.get_parameter('forward_length').value
        self.back_length = self.get_parameter('back_length').value
        
        # 简单聚类参数
        self.cluster_radius = self.get_parameter('cluster_radius').value
        self.min_points_per_cluster = self.get_parameter('min_points_per_cluster').value
        
        self.inner_dist_sq = self.get_parameter('inner_dist_sq').value
        self.inner_y_threshold_sq = self.get_parameter('inner_y_threshold_sq').value
        self.cut_length_y = self.get_parameter('cut_length_y').value
        self.cut_length_x = self.get_parameter('cut_length_x').value
        self.inner_to_outer_x_threshold = self.get_parameter('inner_to_outer_x_threshold').value
        self.inner_to_outer_y_threshold = self.get_parameter('inner_to_outer_y_threshold').value
        
        self.max_radius = self.get_parameter('max_radius').value
        self.max_y_filter = self.get_parameter('max_y_filter').value
        self.min_inner_y_threshold = self.get_parameter('min_inner_y_threshold').value
        self.target_jump = self.get_parameter('target_jump').value
        
        # 直角弯几何检测参数
        self.wall_detection_min_distance = self.get_parameter('wall_detection_min_distance').value
        self.wall_detection_max_distance = self.get_parameter('wall_detection_max_distance').value
        
        self.enable_mapping = self.get_parameter('enable_mapping').value
        self.mapping_interval = self.get_parameter('mapping_interval').value
        self.save_path = self.get_parameter('save_path').value
        self.show_window = self.get_parameter('show_window').value
        
        # --- 状态变量 ---
        self.target_x = -10000.0
        self.target_y = -10000.0
        self.inner_x = []
        self.inner_y = []
        self.outer_x = []
        self.outer_y = []
        self.cone_count = 0
        
        # 视觉检测相关变量（用于直角弯检测）
        self.blue_cones_detected = False  # 是否检测到蓝色锥桶（直角弯标志）
        self.image_detection_data = [0, 0, 0, 0]  # [红灯, A/B, 黄线, 蓝色锥桶检测]
        # 用于可视化的上一帧数据（在特殊模式下保留显示）
        self.last_inner_x = []
        self.last_inner_y = []
        self.last_outer_x = []
        self.last_outer_y = []
        self.current_mode = "初始化"  # 当前模式
        self.trigger_condition = "初始化中"  # 触发条件
        self.frame_count = 0  # 帧计数器
        
        # === [新增] 目标点平滑滤波变量 ===
        self.last_target_x = 0.0
        self.last_target_y = 0.0
        # 滤波系数 alpha (0~1)
        # 0.5 表示：50% 取当前测量值，50% 取上一帧平滑值
        # 系数越小越平滑但延迟越高，建议 0.4 - 0.7 之间
        # 降低到0.5以减小目标点抖动，减少左右晃动
        self.filter_alpha = 0.5
        self.is_first_frame = True
        
        # 位置信息（用于目标区域停止功能）
        self.pos_x = 0.0        # 原始里程计坐标（用于停车区域判断，保持与硬编码坐标一致）
        self.pos_y = 0.0
        self.odom_lock = Lock()
        # 里程计归零相关状态（仅用于显示，不影响停车区域判断）
        self.odom_initialized = False   # 是否已初始化里程计零点
        self.odom_offset_x = 0.0        # 里程计初始X偏移量
        self.odom_offset_y = 0.0        # 里程计初始Y偏移量
        # 第一个停车区域状态（红灯识别区）
        self.target_stop_triggered1 = False  # 第一个区域是否已触发过停止信号（防止重复触发）
        self.target_stop_start_time1 = None  # 第一个区域停止信号开始时间
        # 第二个停车区域状态
        self.target_stop_triggered2 = False  # 第二个区域是否已触发过停止信号（防止重复触发）
        self.target_stop_start_time2 = None  # 第二个区域停止信号开始时间
        self.target_stop_signal_duration = 3.5  # 停止信号持续时间（秒），确保控制节点有时间完成停车
        
        # 可视化相关变量
        self.mapping_counter = 0
        self.valid_x = []
        self.valid_y = []
        
        # 若开启弹窗，初始化交互式绘图
        self.fig = None
        self.ax = None
        if self.enable_mapping and self.show_window:
            try:
                import matplotlib
                # 检查是否有显示环境
                import os
                if 'DISPLAY' not in os.environ or not os.environ.get('DISPLAY'):
                    self.get_logger().warn(">>> DISPLAY环境变量未设置，无法显示弹窗。将改为保存图片模式。如需显示弹窗，请设置DISPLAY环境变量（如：export DISPLAY=:0）<<<")
                    self.show_window = False
                else:
                    # 设置 matplotlib 使用非阻塞后端
                    matplotlib.use('TkAgg')
                    plt.ion()
                    self.fig, self.ax = plt.subplots(figsize=(10, 10))
                    self.fig.canvas.manager.set_window_title('Perception Node - Real-time Map')
                    # 确保窗口显示
                    plt.show(block=False)
                    plt.pause(0.1)
                    self.get_logger().info(">>> Matplotlib弹窗初始化成功 <<<")
            except ImportError as e:
                self.get_logger().warn(f">>> Matplotlib弹窗初始化失败（缺少TkAgg后端），将改为保存图片模式: {str(e)} <<<")
                self.get_logger().warn(">>> 提示：如果是在SSH远程连接，请使用X11转发（ssh -X）或设置DISPLAY环境变量 <<<")
                self.show_window = False
            except Exception as e:
                self.get_logger().warn(f">>> Matplotlib弹窗初始化失败，将改为保存图片模式: {str(e)} <<<")
                self.get_logger().warn(">>> 提示：如果是在SSH远程连接，请使用X11转发（ssh -X）或设置DISPLAY环境变量 <<<")
                self.show_window = False

        # --- QoS配置 ---
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- ROS2 订阅与发布 ---
        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 订阅里程计（用于目标区域停止功能）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_combined',
            self.odom_callback,
            qos_profile=qos_profile_sensor  # 复用 laser scan 的 QoS 配置
        )
        
        # 订阅图像检测（用于直角弯检测）
        self.image_detection_sub = self.create_subscription(
            Int16MultiArray,
            '/image_detection',
            self.image_detection_callback,
            10
        )
        
        # 发布目标点（基于源程序main.py的Float32MultiArray格式）
        self.target_pub = self.create_publisher(Float32MultiArray, '/target', 10)
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("纯感知节点已启动（基于NCSC2024源程序，使用简单聚类）")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"V7 扇形检测: 距离={self.no_cone_detection_range}m, 角度=±{self.no_cone_sector_angle}°")
        self.get_logger().info(f"激光雷达滤波: 距离[{self.min_range}, {self.max_range}]m")
        self.get_logger().info(f"矩形滤波: 左{self.left_width}m, 右{self.right_width}m, 前{self.forward_length}m")
        self.get_logger().info(f"简单聚类: 半径={self.cluster_radius}m, 最少点数={self.min_points_per_cluster}")
        self.get_logger().info(f"内外道分离: 内道扩展距离²={self.inner_dist_sq}")
        self.get_logger().info(f"圆形滤波: 最大半径={self.max_radius}m, Y过滤={self.max_y_filter}m")
        self.get_logger().info(f"目标点选择: 第{self.target_jump}个最近点")
        self.get_logger().info(f"可视化: 启用={self.enable_mapping}, 弹窗={self.show_window}, 间隔={self.mapping_interval}帧, 保存路径={self.save_path}")
        self.get_logger().info("=" * 60)
    
    def euclidean_dist(self, p1, p2):
        """计算两点间欧氏距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def simple_density_cluster(self, x_list, y_list):
        """
        简单聚类：将小弧（一个锥桶的点云）变成一个点
        返回：聚类中心列表 [[x, y], ...]
        """
        if len(x_list) == 0:
            return []
        
        points = list(zip(x_list, y_list))
        cluster_centers = []
        processed = set()
        
        for i, point in enumerate(points):
            if i in processed:
                continue
            
            # 找到所有邻居点
            neighbors = [i]
            for j, other_point in enumerate(points):
                if i != j and self.euclidean_dist(point, other_point) < self.cluster_radius:
                    neighbors.append(j)
            
            # 如果点数足够，形成一个簇
            if len(neighbors) >= self.min_points_per_cluster:
                # 计算簇中心
                cluster_points = [points[idx] for idx in neighbors]
                center_x = np.mean([p[0] for p in cluster_points])
                center_y = np.mean([p[1] for p in cluster_points])
                cluster_centers.append([center_x, center_y])
                
                # 标记为已处理
                processed.update(neighbors)
        
        return cluster_centers
    
    def euclidean_dist_sq(self, x1, y1, x2, y2):
        """计算欧氏距离平方（基于源程序main.py的euciDistSq2函数）"""
        return (x1 - x2) ** 2 + (y1 - y2) ** 2
    
    def fit_circle(self, x, y):
        """
        拟合圆形（基于源程序main.py的Fit_Circle函数）
        返回圆心(x, y)和半径
        """
        if len(x) < 3:
            return 0, 0, 0
        
        C = []
        A = []
        
        for i in range(len(x)):
            A.append([x[i], y[i], 1])
            C.append(x[i]**2 + y[i]**2)
        
        A = np.array(A)
        C = np.array(C)
        
        try:
            B = np.dot(np.dot(np.linalg.inv(np.dot(A.T, A)), A.T), C)
            center_x = B[0] / 2
            center_y = B[1] / 2
            radius = math.sqrt(B[2] + B[0]**2/4 + B[1]**2/4)
            return center_x, center_y, radius
        except:
            return 0, 0, 0
    
    def fit_circle_and_filter(self, inner_x, inner_y, out_x, out_y):
        """
        根据内道拟合的圆形来滤除错误外道（基于源程序main.py的fitCircleAndFilter函数）
        """
        if len(inner_x) <= 2:
            # 内道点小于等于两个则不做圆形滤波，仅过滤y过大的外道点
            threshold = self.min_inner_y_threshold
            indices = [i for i in range(len(out_x)) if out_y[i] <= threshold]
            real_out_x = [out_x[i] for i in indices]
            real_out_y = [out_y[i] for i in indices]
            return real_out_x, real_out_y
        
        # 根据圆形滤波过滤
        center_x, center_y, radius = self.fit_circle(inner_x, inner_y)
        
        if radius > self.max_radius:
            # 假如拟合半径大于最大半径，则认为是直道，仅对y过大的外道点滤波
            indices = [i for i in range(len(out_x)) if out_y[i] <= self.max_y_filter]
            real_out_x = [out_x[i] for i in indices]
            real_out_y = [out_y[i] for i in indices]
            return real_out_x, real_out_y
        
        # 计算原点与拟合圆的半径平方
        radius_sq = self.euclidean_dist_sq(center_x, center_y, 0, 0)
        flag = 1 if radius_sq >= radius**2 else 0  # 原点在拟合圆外/内
        
        real_out_x = []
        real_out_y = []
        
        for i in range(len(out_x)):
            current_r_sq = self.euclidean_dist_sq(out_x[i], out_y[i], center_x, center_y)
            extra_radius = 1
            
            if current_r_sq >= radius_sq and flag == 1:
                real_out_x.append(out_x[i])
                real_out_y.append(out_y[i])
            elif current_r_sq <= (radius_sq + extra_radius) and flag == 0:
                real_out_x.append(out_x[i])
                real_out_y.append(out_y[i])
        
        return real_out_x, real_out_y
    
    def gen_three_equinoxes(self, inner_x, inner_y, outer_x, outer_y):
        """
        生成三等分点（移植自line_follow2，移除距离检查以提升S弯表现）
        为每个内道点找到相配对的外道点(最近欧氏距离),从而得到三等分点
        """
        res_x = []
        res_y = []
        ratio = 0.5  # 中线（与源程序一致）
        
        if len(inner_x) == 0 or len(outer_x) == 0:
            return [], []
        
        for i in range(len(inner_x)):
            min_dist_sq = float('inf')
            min_index = -1
            
            for j in range(len(outer_x)):
                dist_sq = self.euclidean_dist_sq(inner_x[i], inner_y[i], outer_x[j], outer_y[j])
                if min_dist_sq > dist_sq:
                    min_index = j
                    min_dist_sq = dist_sq
            
            if min_index != -1:
                # 移植自line_follow2：移除距离检查，直接计算中点（提升S弯响应速度）
                res_x.append(inner_x[i] + ratio * (outer_x[min_index] - inner_x[i]))
                res_y.append(inner_y[i] + ratio * (outer_y[min_index] - inner_y[i]))
        
        return res_x, res_y
    
    def get_target_point(self, target_group_x, target_group_y):
        """
        根据三等分点选择目标点（移植自line_follow2，移除Y坐标限制以提升S弯表现）
        """
        if len(target_group_x) == 0 or len(target_group_y) == 0:
            return -20000.0, -20000.0  # 移植自line_follow2：返回-20000而不是-10000
        
        jump = int(round(self.target_jump))  # 选择第jump近的点（默认3）
        
        # 移植自line_follow2：只筛选x>=0的前方点，没有Y坐标限制（适应S弯大幅摆动）
        forward_points = [(target_group_x[i], target_group_y[i]) 
                         for i in range(len(target_group_x)) if target_group_x[i] >= 0]
        
        if not forward_points:
            return -20000.0, -20000.0  # 没有前方点
        
        # 计算距离
        dist_sq = [self.euclidean_dist_sq(p[0], p[1], 0, 0) for p in forward_points]
        
        # 快速排序
        sorted_indices = self.quick_sort(dist_sq, list(range(len(dist_sq))))
        
        # 得到目标点（移植自line_follow2逻辑）
        if len(sorted_indices) < jump:
            selected_index = sorted_indices[-1]  # 点不够，选最远的(前方)点
        else:
            selected_index = sorted_indices[jump - 1]  # 选第jump近的点
        
        return forward_points[selected_index]
    
    def get_fallback_target_point(self, inner_x, inner_y, outer_x, outer_y):
        """
        降级策略：当内外道配对失败时，基于左右锥桶分布计算目标点
        参数：
        - inner_x, inner_y: 内道锥桶点
        - outer_x, outer_y: 外道锥桶点
        返回：
        - target_x, target_y: 目标点坐标，如果无法计算则返回 -20000.0, -20000.0
        """
        # 合并所有锥桶点（内道+外道）
        all_cones = []
        for i in range(len(inner_x)):
            all_cones.append((inner_x[i], inner_y[i]))
        for i in range(len(outer_x)):
            all_cones.append((outer_x[i], outer_y[i]))
        
        if len(all_cones) == 0:
            return -20000.0, -20000.0
        
        # 只考虑前方点（X >= 0）
        forward_cones = [(x, y) for x, y in all_cones if x >= 0]
        
        if len(forward_cones) == 0:
            return -20000.0, -20000.0
        
        # 分离左右锥桶（Y>0为左侧，Y<0为右侧）
        left_cones = [(x, y) for x, y in forward_cones if y > 0]
        right_cones = [(x, y) for x, y in forward_cones if y < 0]
        center_cones = [(x, y) for x, y in forward_cones if abs(y) <= 0.1]  # 中心线附近（±0.1m）
        
        self.get_logger().info(
            f">>> [降级策略] 总锥桶数: {len(forward_cones)}, "
            f"左侧: {len(left_cones)}, 右侧: {len(right_cones)}, 中心: {len(center_cones)}",
            throttle_duration_sec=1.0
        )
        
        # 策略1：如果左右都有锥桶，计算左右中心点，取中点作为目标
        if len(left_cones) > 0 and len(right_cones) > 0:
            # 计算左侧锥桶的平均位置（只考虑前方1.5米内的点，避免过远点影响）
            left_forward = [(x, y) for x, y in left_cones if 0 <= x <= 1.5]
            right_forward = [(x, y) for x, y in right_cones if 0 <= x <= 1.5]
            
            if len(left_forward) > 0 and len(right_forward) > 0:
                left_center_x = sum(x for x, y in left_forward) / len(left_forward)
                left_center_y = sum(y for x, y in left_forward) / len(left_forward)
                right_center_x = sum(x for x, y in right_forward) / len(right_forward)
                right_center_y = sum(y for x, y in right_forward) / len(right_forward)
                
                # 取中点，稍微偏向中心线
                target_x = (left_center_x + right_center_x) / 2
                target_y = (left_center_y + right_center_y) / 2 * 0.7  # 乘以0.7，稍微偏向中心
                
                self.get_logger().info(
                    f">>> [降级策略] 左右平衡模式: 目标点({target_x:.2f}, {target_y:.2f})",
                    throttle_duration_sec=1.0
                )
                return target_x, target_y
        
        # 策略2：如果只有一侧有锥桶，向另一侧避障
        if len(left_cones) > len(right_cones) * 1.5:  # 左侧锥桶明显更多
            # 向右避障：目标点在右前方
            target_x = 1.2
            target_y = -0.4
            self.get_logger().info(
                f">>> [降级策略] 左侧锥桶多，向右避障: 目标点({target_x:.2f}, {target_y:.2f})",
                throttle_duration_sec=1.0
            )
            return target_x, target_y
        elif len(right_cones) > len(left_cones) * 1.5:  # 右侧锥桶明显更多
            # 向左避障：目标点在左前方
            target_x = 1.2
            target_y = 0.4
            self.get_logger().info(
                f">>> [降级策略] 右侧锥桶多，向左避障: 目标点({target_x:.2f}, {target_y:.2f})",
                throttle_duration_sec=1.0
            )
            return target_x, target_y
        
        # 策略3：如果左右数量相近，选择前方最近的锥桶，向另一侧偏移
        if len(forward_cones) > 0:
            # 找到前方最近的锥桶
            min_dist_sq = float('inf')
            nearest_cone = None
            for x, y in forward_cones:
                dist_sq = self.euclidean_dist_sq(x, y, 0, 0)
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    nearest_cone = (x, y)
            
            if nearest_cone:
                # 向锥桶的另一侧偏移
                offset_y = -0.3 if nearest_cone[1] > 0 else 0.3  # 如果锥桶在左侧，向右偏移；反之向左
                target_x = min(nearest_cone[0] + 0.3, 1.5)  # 稍微向前，但不超过1.5米
                target_y = nearest_cone[1] + offset_y
                
                self.get_logger().info(
                    f">>> [降级策略] 最近锥桶避障: 目标点({target_x:.2f}, {target_y:.2f})",
                    throttle_duration_sec=1.0
                )
                return target_x, target_y
        
        # 如果所有策略都失败，返回失败信号
        return -20000.0, -20000.0
    
    def quick_sort(self, arr, indices):
        """快速排序（基于源程序main.py的quick_sort函数）"""
        if len(indices) <= 1:
            return indices
        
        pivot = arr[indices[len(indices) // 2]]
        left = [i for i in indices if arr[i] < pivot]
        middle = [i for i in indices if arr[i] == pivot]
        right = [i for i in indices if arr[i] > pivot]
        
        return self.quick_sort(arr, left) + middle + self.quick_sort(arr, right)
    
    def mapping(self, x1, y1, x2, y2, x3, y3, x4, y4):
        """
        实时绘图函数（基于源程序main.py的mapping函数）
        参数：
        - x1, y1: 原始激光雷达点云数据
        - x2, y2: 内道锥桶点（红色星号）
        - x3, y3: 外道锥桶点（黄色星号）
        - x4, y4: 目标点（紫色星号）
        """
        try:
            area = 5
            # 选择绘图目标：弹窗绘制到 ax，否则用全局 plt 保存
            if self.show_window:
                if self.ax is None or self.fig is None:
                    import matplotlib
                    matplotlib.use('TkAgg')
                    plt.ion()
                    self.fig, self.ax = plt.subplots(figsize=(10, 10))
                    self.fig.canvas.manager.set_window_title('Perception Node - Real-time Map')
                    plt.show(block=False)
                    plt.pause(0.1)
                ax = self.ax
                ax.clear()  # 使用 clear() 而不是 cla()，更彻底
                ax.set_xlim(-8, 8)
                ax.set_ylim(-8, 8)

                ax.scatter(x1, y1, s=area, alpha=1, c='blue', label='Point Cloud')
                
                # 绘制黄线弯检测扇形区域
                sector_angle_deg = self.no_cone_sector_angle
                detection_range = self.no_cone_detection_range
                # matplotlib角度：0°=X轴正方向(车辆前方)，逆时针为正
                # 扇形从-84°到+84°，覆盖车辆前方168度范围
                wedge = Wedge((0, 0), detection_range, 
                             -sector_angle_deg, sector_angle_deg,
                             facecolor='cyan', alpha=0.2, edgecolor='cyan', linewidth=2,
                             label=f'Yellow Curve Zone (±{sector_angle_deg:.0f}°, {detection_range}m)')
                ax.add_patch(wedge)
                
                ax.plot([0], [0], "p", color="green", markersize=10, label='Vehicle')
                if len(x2) > 0 and len(y2) > 0:
                    ax.scatter(x2, y2, marker='*', c="red", s=20, alpha=1, label='Inner Cones')
                    # 显示内道点坐标（上方点向上，下方点向下，左边点向左，右边点向右）
                    for i in range(len(x2)):
                        px, py = x2[i], y2[i]
                        # 垂直方向：上方点向上，下方点向下
                        if py >= 0:
                            offset_y = 35  # 向上
                        else:
                            offset_y = -35  # 向下
                        # 水平方向：左边点向左，右边点向右
                        if px < 0:
                            offset_x = -30  # 向左
                        else:
                            offset_x = 30  # 向右（包括中间区域）
                        # 如果多个点在同一区域，添加额外垂直偏移避免重叠
                        offset_y += (i % 3 - 1) * 10
                        ax.annotate(f'({px:.2f},{py:.2f})', 
                                   (px, py), 
                                   xytext=(offset_x, offset_y), textcoords='offset points',
                                   fontsize=9, color='red', weight='bold',
                                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.8),
                                   arrowprops=dict(arrowstyle='->', color='red', lw=1.5, alpha=0.6))
                if len(x3) > 0 and len(y3) > 0:
                    ax.scatter(x3, y3, marker='*', c="yellow", s=20, alpha=1, label='Outer Cones')
                    # 显示外道点坐标（上方点向上，下方点向下，左边点向左，右边点向右）
                    for i in range(len(x3)):
                        px, py = x3[i], y3[i]
                        # 垂直方向：上方点向上，下方点向下
                        if py >= 0:
                            offset_y = 35  # 向上
                        else:
                            offset_y = -35  # 向下
                        # 水平方向：左边点向左，右边点向右
                        if px < 0:
                            offset_x = -30  # 向左
                        else:
                            offset_x = 30  # 向右（包括中间区域）
                        # 如果多个点在同一区域，添加额外垂直偏移避免重叠
                        offset_y += (i % 3 - 1) * 10
                        ax.annotate(f'({px:.2f},{py:.2f})', 
                                   (px, py), 
                                   xytext=(offset_x, offset_y), textcoords='offset points',
                                   fontsize=9, color='orange', weight='bold',
                                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='orange', alpha=0.8),
                                   arrowprops=dict(arrowstyle='->', color='orange', lw=1.5, alpha=0.6))
                # 显示目标点（包括特殊模式的目标点）
                if len(x4) > 0 and len(y4) > 0:
                    # 特殊目标点用不同颜色和标记显示
                    if x4[0] == -10000.0 or x4[0] == -20000.0 or x4[0] == -30000.0 or x4[0] == -40000.0:
                        # 特殊模式：用大号标记显示在原点附近
                        ax.scatter([0], [0], marker='X', c="orange", s=200, alpha=0.8, label='Special Mode')
                    elif x4[0] == -99999.0 or x4[0] == -88888.0:
                        # 停车区：用大号标记显示在原点附近
                        ax.scatter([0], [0], marker='s', c="red", s=200, alpha=0.8, label='Stop Area')
                    else:
                        # 正常目标点
                        ax.scatter(x4, y4, marker='*', c="purple", s=50, alpha=1, label='Target')

                ax.legend(loc='upper right', fontsize=8)
                ax.grid(True, alpha=0.3)
                
                # 根据target_x显示当前模式和触发条件（使用英文避免字体问题）
                mode_text = "Initializing..."
                if hasattr(self, 'target_x'):
                    if self.target_x == -99999.0:
                        mode_text = "Mode: Stop Area 1 (Red Light)"
                    elif self.target_x == -88888.0:
                        mode_text = "Mode: Stop Area 2 (Sign Recognition)"
                    elif self.target_x == -10000.0:
                        mode_text = "Mode: Yellow Curve (No Cones)"
                    elif self.target_x == -20000.0:
                        mode_text = "Mode: Right Angle Turn (Perception Failed)"
                    elif self.target_x == -30000.0:
                        mode_text = "Mode: Right Correction (No Mid Points)"
                    elif self.target_x == -40000.0:
                        mode_text = "Mode: Keep Current State"
                    else:
                        mode_text = "Mode: Normal Tracking"
                
                ax.set_title(f'Lidar Real-time Map | {mode_text}', fontsize=10, pad=10)
                ax.set_xlabel('X (m)', fontsize=9)
                ax.set_ylabel('Y (m)', fontsize=9)

                # 强制刷新显示
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                plt.pause(0.01)
            else:
                # 保存图片模式（也使用英文标签）
                plt.xlim(-8, 8)
                plt.ylim(-8, 8)
                plt.scatter(x1, y1, s=area, alpha=1, c='blue', label='Point Cloud')
                
                # 绘制黄线弯检测扇形区域
                sector_angle_deg = self.no_cone_sector_angle
                detection_range = self.no_cone_detection_range
                wedge = Wedge((0, 0), detection_range, 
                             -sector_angle_deg, sector_angle_deg,
                             facecolor='cyan', alpha=0.2, edgecolor='cyan', linewidth=2,
                             label=f'Yellow Curve Zone (±{sector_angle_deg:.0f}°, {detection_range}m)')
                plt.gca().add_patch(wedge)
                
                plt.plot([0], [0], "p", color="green", markersize=10, label='Vehicle')
                if len(x2) > 0 and len(y2) > 0:
                    plt.scatter(x2, y2, marker='*', c="red", s=20, alpha=1, label='Inner Cones')
                if len(x3) > 0 and len(y3) > 0:
                    plt.scatter(x3, y3, marker='*', c="yellow", s=20, alpha=1, label='Outer Cones')
                if len(x4) > 0 and len(y4) > 0:
                    if x4[0] == -10000.0 or x4[0] == -20000.0 or x4[0] == -30000.0 or x4[0] == -40000.0:
                        plt.scatter([0], [0], marker='X', c="orange", s=200, alpha=0.8, label='Special Mode')
                    elif x4[0] == -99999.0 or x4[0] == -88888.0:
                        plt.scatter([0], [0], marker='s', c="red", s=200, alpha=0.8, label='Stop Area')
                    else:
                        plt.scatter(x4, y4, marker='*', c="purple", s=50, alpha=1, label='Target')
                plt.legend(fontsize=8)
                plt.grid(True, alpha=0.3)
                
                # 使用英文标题
                mode_text = "Initializing..."
                if hasattr(self, 'target_x'):
                    if self.target_x == -99999.0:
                        mode_text = "Mode: Stop Area 1 (Red Light)"
                    elif self.target_x == -88888.0:
                        mode_text = "Mode: Stop Area 2 (Sign Recognition)"
                    elif self.target_x == -10000.0:
                        mode_text = "Mode: Yellow Curve (No Cones)"
                    elif self.target_x == -20000.0:
                        mode_text = "Mode: Right Angle Turn (Perception Failed)"
                    elif self.target_x == -30000.0:
                        mode_text = "Mode: Right Correction (No Mid Points)"
                    elif self.target_x == -40000.0:
                        mode_text = "Mode: Keep Current State"
                    else:
                        mode_text = "Mode: Normal Tracking"
                
                plt.title(f'Lidar Real-time Map | {mode_text}', fontsize=10)
                plt.xlabel('X (m)')
                plt.ylabel('Y (m)')
                plt.savefig(self.save_path, dpi=100, bbox_inches='tight')
                plt.cla()

        except Exception as e:
            self.get_logger().error(f'绘图失败: {str(e)}')
            if not self.show_window:
                plt.cla()
    
    def check_no_cone_sector(self, ranges, angles):
        """
        V29 新增：扇形检测逻辑
        """
        sector_angle_rad = self.no_cone_sector_angle_rad
        detection_range = self.no_cone_detection_range
        point_count = 0
        threshold = self.no_cone_sector_threshold

        for i in range(len(ranges)):
            r = ranges[i]
            theta = angles[i]
            if r <= 0 or np.isnan(r):
                continue
            if abs(theta) <= sector_angle_rad and r <= detection_range:
                point_count += 1
                if point_count > threshold:
                    return False
        return True
    
    def set_yellow_curve_signal(self, reason):
        """设置黄线弯信号（简化重复代码）"""
        self.target_x = -10000.0
        self.target_y = -10000.0
        self.trigger_condition = reason
        # 保留上一帧的锥桶点用于可视化
        if len(self.inner_x) == 0 and len(self.last_inner_x) > 0:
            self.inner_x, self.inner_y = self.last_inner_x.copy(), self.last_inner_y.copy()
            self.outer_x, self.outer_y = self.last_outer_x.copy(), self.last_outer_y.copy()
        else:
            self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
        self.cone_count = 0
    
    def set_right_angle_signal(self, reason):
        """设置直角弯信号（简化重复代码）"""
        self.target_x = -20000.0
        self.target_y = -20000.0
        self.trigger_condition = reason
        self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []

    def detect_wall_geometry(self, x_list, y_list):
        """
        几何特征检测：检查外道点是否在远处形成一堵墙（直角弯入口特征）
        参数由实际雷达图测算：远处黄色锥桶 X 差异极小(扁)，Y 跨度大(宽)。
        增强：添加挡板过滤，避免误判挡板为直角弯
        """
        # 1. 数量预筛：如果外道点太少，构不成墙
        if len(x_list) < 4:  # 提高阈值，避免挡板误判
            return False

        # 2. 距离过滤：只提取在 [min_distance, max_distance] 区间内的点，排除车身旁边的干扰和过远的点
        far_indices = [i for i, x in enumerate(x_list) if self.wall_detection_min_distance < x < self.wall_detection_max_distance]
        
        # 如果远处点不足4个，无法判断特征（提高阈值）
        if len(far_indices) < 4:
            return False
            
        far_x = [x_list[i] for i in far_indices]
        far_y = [y_list[i] for i in far_indices]
        
        # 3. 挡板过滤：检查Y坐标分布
        # 挡板通常在Y=0附近（中心线），而直角弯的墙在Y>0的左侧
        # 如果大部分点在Y<0.3（靠近中心线），可能是挡板，不是直角弯
        left_side_count = sum(1 for y in far_y if y > 0.3)  # 左侧（Y>0.3）的点数
        if left_side_count < len(far_y) * 0.6:  # 如果左侧点数少于60%，可能是挡板
            self.get_logger().info(f">>> [几何检测] 疑似挡板，左侧点数({left_side_count}/{len(far_y)})不足，跳过直角弯检测")
            return False
        
        # 4. 计算几何包围盒
        x_span = max(far_x) - min(far_x)  # 纵向厚度（墙的厚薄）
        y_span = max(far_y) - min(far_y)  # 横向跨度（墙的宽度）
        
        # 5. 核心判断逻辑（提高阈值，更严格）：
        # x_span < 0.5: 墙很扁（从0.6降低到0.5，更严格）
        # y_span > 1.8: 墙很宽（从1.5提高到1.8，更严格，确保是真正的封路墙）
        if x_span < 0.5 and y_span > 1.8:
            self.get_logger().warn(f">>> [几何检测] 发现直角弯封路墙! X厚度:{x_span:.2f}m, Y宽度:{y_span:.2f}m")
            return True
            
        return False

    def separate_inner_outer_lanes(self, dedup_points):
        """
        简化的内外道分离逻辑：
        1. 左边（Y>0）是内道，右边（Y<0）是外道
        2. 除了直角弯，前方离得远的是外道（X>1.5m的点优先归为外道）
        """
        inner_x = []
        inner_y = []
        outer_x = []
        outer_y = []
        
        if len(dedup_points) < 2:
            return inner_x, inner_y, outer_x, outer_y
        
        # 简化的分离规则：
        # 1. Y>0（左侧）的点优先归为内道
        # 2. Y<0（右侧）的点归为外道
        # 3. 前方较远的点（X>1.5m）优先归为外道（避免挡板被误判为内道）
        # 4. 中心线附近（|Y|<0.2m）的点，根据X距离判断：前方较远的归为外道
        
        for p in dedup_points:
            x, y = p[0], p[1]
            
            # 规则1：右侧（Y<0）的点直接归为外道
            if y < 0:
                outer_x.append(x)
                outer_y.append(y)
            # 规则2：前方较远的点（X>1.5m）优先归为外道（避免挡板误判）
            elif x > 1.5:
                outer_x.append(x)
                outer_y.append(y)
            # 规则3：中心线附近（|Y|<0.2m）的点，前方较远的归为外道
            elif abs(y) < 0.2 and x > 0.8:
                outer_x.append(x)
                outer_y.append(y)
            # 规则4：其他左侧点（Y>0）归为内道
            else:
                inner_x.append(x)
                inner_y.append(y)
        
        # 调试信息：显示分离结果
        self.get_logger().info(
            f">>> [简化分离] 内道: {len(inner_x)}个, 外道: {len(outer_x)}个",
            throttle_duration_sec=2.0
        )
        
        return inner_x, inner_y, outer_x, outer_y
    
    def image_detection_callback(self, msg: Int16MultiArray):
        """
        图像检测回调函数 - 接收视觉检测结果
        数据格式: [红灯, A/B, 黄线, AB车库方向, 蓝色锥桶检测]
        蓝色锥桶检测: 0=未检测，1=检测到正前方两个蓝色锥桶（一左一右，大小与距离一致）
        """
        if len(msg.data) >= 5:
            self.image_detection_data = msg.data
            # 蓝色锥桶检测结果（第5个元素，索引4）
            blue_cones = msg.data[4] if len(msg.data) > 4 else 0
            old_blue_cones = self.blue_cones_detected
            self.blue_cones_detected = (blue_cones == 1)
            
            # 如果蓝色锥桶检测状态发生变化，打印日志
            if old_blue_cones != self.blue_cones_detected:
                if self.blue_cones_detected:
                    self.get_logger().warn("🔵🔵🔵 【视觉检测】检测到正前方两个蓝色锥桶（直角弯标志）！ 🔵🔵🔵")
                else:
                    self.get_logger().info("✅ 【视觉检测】蓝色锥桶消失")
    
    def odom_callback(self, msg: Odometry):
        """[新增] 订阅里程计信息，更新机器人当前位置"""
        # 获取原始里程计坐标
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        
        # 第一次接收到里程计数据时，记录初始位置作为零点（与control_node.py保持一致）
        if not self.odom_initialized:
            self.odom_offset_x = raw_x
            self.odom_offset_y = raw_y
            self.odom_initialized = True
        
        with self.odom_lock:
            # 使用归零后的坐标（与control_node.py打印的坐标一致，确保可以用打印坐标直接设置停车区域）
            if self.odom_initialized:
                self.pos_x = raw_x - self.odom_offset_x
                self.pos_y = raw_y - self.odom_offset_y
            else:
                self.pos_x = raw_x
                self.pos_y = raw_y
    
    def scan_callback(self, msg: LaserScan):
        """
        激光雷达回调 - 基于源程序main.py的callBack函数
        """
        ranges = np.array(msg.ranges)
        num_ranges = len(ranges)
        
        self.frame_count += 1
        
        # === 新增：目标区域停止逻辑 ===
        # 1. 安全地获取当前位置
        with self.odom_lock:
            current_pos_x = self.pos_x
            current_pos_y = self.pos_y
        
        # ============ 硬编码停车区域坐标（已注释） ============
        # 注意：这里使用归零后的坐标系统（与control_node.py打印的坐标完全一致）
        # 使用方法：查看control_node.py打印的"归零显示"坐标，直接用那个坐标值来设置停车区域范围
        # 例如：如果打印显示 X=3.30, Y=3.01，想在该位置±0.2米停车，则设置为 [3.10, 3.50] x [2.81, 3.21]
        # 
        # 2. 检查是否在第一个目标停止区域（红灯识别区）
        # 修改第一个停车区域：根据打印的归零显示坐标修改下面的坐标范围
        # 注意：因为坐标系统已归零，原有的 [3.35, 5.35] x [3.05, 4.75] 需要根据新的初始位置调整
        # currently_in_area1 = (current_pos_x >= 3.35 and current_pos_x <= 4.75 and 
        #                       current_pos_y >= 3.05 and current_pos_y <= 4.75)
        
        # 3. 检查是否在第二个目标停止区域（扩大范围，提前触发，给精确移动留出时间和空间）
        # 修改第二个停车区域：根据打印的归零显示坐标修改下面的坐标范围
        # 注意：扩大检测范围，让小车提前进入精确停车模式，有足够时间移动到精确位置
        # 扩大后的范围：X: [-2.40, -1.70]，Y: [-1.50, 1.00] - 比精确停车位置范围更大
        # currently_in_area2 = (current_pos_x >= -5.00 and current_pos_x <= -1.70 and 
        #                       current_pos_y >= -1.50 and current_pos_y <= 1.00)
        # ============ 硬编码停车区域坐标结束 ============
        
        # 4. 检查是否应该发送停止信号 (-2.08, -0.78) 
        # should_send_stop_signal = False
        # stop_signal_type = None  # -99999.0 为第一个停车区，-88888.0 为第二个停车区
        
        # if currently_in_area1:
        #     if not self.target_stop_triggered1:
        #         # 首次进入第一个区域（红灯识别区），触发停止信号
        #         self.target_stop_triggered1 = True
        #         self.target_stop_start_time1 = time.time()
        #         should_send_stop_signal = True
        #         stop_signal_type = -99999.0
        #         self.get_logger().warn(">>>【已到达第一个目标停止区域（红灯识别区）】触发停止信号<<<")
        #     elif self.target_stop_start_time1 is not None:
        #         # 已触发，检查是否在信号持续时间内
        #         elapsed = time.time() - self.target_stop_start_time1
        #         if elapsed < self.target_stop_signal_duration:
        #             should_send_stop_signal = True
        #             stop_signal_type = -99999.0
        #             self.get_logger().debug(f">>> 持续发送第一个停止信号 (已持续 {elapsed:.1f}秒) <<<")
        #         else:
        #             # 超过持续时间，恢复正常感知（只打印一次）
        #             self.get_logger().info(">>> 第一个停止信号持续时间结束，恢复正常感知 <<<")
        #             self.target_stop_start_time1 = None  # 清除时间标记，避免重复打印
        #             should_send_stop_signal = False
        # elif currently_in_area2:
        #     if not self.target_stop_triggered2:
        #         # 首次进入第二个区域，触发停止信号
        #         self.target_stop_triggered2 = True
        #         self.target_stop_start_time2 = time.time()
        #         should_send_stop_signal = True
        #         stop_signal_type = -88888.0
        #         self.get_logger().warn(f">>>【进入标志识别区】<<<")
        #     elif self.target_stop_start_time2 is not None:
        #         # 已触发，检查是否在信号持续时间内
        #         elapsed = time.time() - self.target_stop_start_time2
        #         if elapsed < self.target_stop_signal_duration:
        #             should_send_stop_signal = True
        #             stop_signal_type = -88888.0
        #             self.get_logger().debug(f">>> 持续发送第二个停止信号 (已持续 {elapsed:.1f}秒) <<<")
        #         else:
        #             # 超过持续时间，恢复正常感知（只打印一次）
        #             self.get_logger().info(">>> 第二个停止信号持续时间结束，恢复正常感知 <<<")
        #             self.target_stop_start_time2 = None  # 清除时间标记，避免重复打印
        #             should_send_stop_signal = False
        # else:
        #     # 不在任何区域内，重置所有状态
        #     if self.target_stop_triggered1:
        #         self.get_logger().info(">>> 已离开第一个目标停止区域，状态重置 <<<")
        #         self.target_stop_triggered1 = False
        #         self.target_stop_start_time1 = None
        #     if self.target_stop_triggered2:
        #         self.get_logger().info(">>> 已离开第二个目标停止区域，状态重置 <<<")
        #         self.target_stop_triggered2 = False
        #         self.target_stop_start_time2 = None
        
        # 5. 如果需要发送停止信号，发送并返回
        # if should_send_stop_signal:
        #     self.target_x = stop_signal_type 
        #     self.target_y = stop_signal_type
        #     if stop_signal_type == -99999.0:
        #         self.trigger_condition = "红灯识别区"
        #     elif stop_signal_type == -88888.0:
        #         self.trigger_condition = "标志识别区"
        #     
        #     # 清理状态，防止旧数据干扰
        #     self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
        #     self.cone_count = 0
        #     
        #     # 发布停止信号
        #     self.publish_target() 
        #     
        #     # (可选) 更新可视化（使用上一帧的锥桶点数据）
        #     if self.enable_mapping:
        #         self.mapping_counter += 1
        #         if self.mapping_counter >= self.mapping_interval:
        #             # 使用上一帧的锥桶点数据用于可视化
        #             viz_inner_x = self.inner_x if len(self.inner_x) > 0 else self.last_inner_x
        #             viz_inner_y = self.inner_y if len(self.inner_y) > 0 else self.last_inner_y
        #             viz_outer_x = self.outer_x if len(self.outer_x) > 0 else self.last_outer_x
        #             viz_outer_y = self.outer_y if len(self.outer_y) > 0 else self.last_outer_y
        #             self.mapping(
        #                 self.valid_x, self.valid_y,           
        #                 viz_inner_x, viz_inner_y,           
        #                 viz_outer_x, viz_outer_y,           
        #                 [self.target_x], [self.target_y]      
        #             )
        #             self.mapping_counter = 0
        #     
        #     # 立即返回，跳过本帧所有后续的感知计算
        #     return 
        # === 停止逻辑 结束（已注释） ===
        
        if num_ranges == 0:
            self.target_x = -20000.0
            self.target_y = -20000.0
            self.trigger_condition = "激光雷达数据为空"
            self.publish_target()
            return
        
        # === 数据预处理（基于源程序main.py）===
        decimal = 2  # 保留小数位
        min_R = self.min_range
        max_R = self.max_range
        left_width = self.left_width
        right_width = self.right_width
        forward_length = self.forward_length
        back_length = self.back_length
        
        # 极坐标转笛卡尔坐标
        i = np.arange(len(ranges))
        angles = msg.angle_min + i * msg.angle_increment
        valid_i = (ranges > min_R) & (ranges < max_R)
        valid_angles = angles[valid_i]
        valid_ranges = ranges[valid_i]
        
        x = np.round(np.round(valid_ranges, decimal) * np.cos(valid_angles), decimal)
        y = np.round(np.round(valid_ranges, decimal) * np.sin(valid_angles), decimal)
        
        # 矩形滤波（Y轴正方向是左侧，负方向是右侧）
        valid_i = (x > -back_length) & (x < forward_length) & (y < left_width) & (y > -right_width)
        valid_x = x[valid_i]
        valid_y = y[valid_i]
        
        # 保存原始点云数据用于可视化
        self.valid_x = valid_x.tolist()
        self.valid_y = valid_y.tolist()
        
        # === V29 修复：黄线弯/直角弯逻辑分离 ===
        
        # 1. 优先检查矩形滤波后的点数
        # 如果矩形滤波后有点（>=3），说明不是真正的黄线弯（可能是挡板遮挡正前方，但侧面还有锥桶）
        # 直接进行聚类处理，不进行扇形检测
        if len(valid_x) >= 3:
            # 矩形滤波后有点，进行聚类和内外道分离
            # 使用简单聚类替代KMeans
            cluster_centers = self.simple_density_cluster(valid_x.tolist(), valid_y.tolist())
            dedup_points = cluster_centers  # 简单聚类直接返回去重后的中心点列表
            
            self.cone_count = len(dedup_points)
            
            # 调试信息：显示聚类结果
            self.get_logger().info(
                f">>> [聚类结果] 矩形滤波后点数: {len(valid_x)}, "
                f"聚类后锥桶数: {len(dedup_points)}, "
                f"聚类参数: 半径={self.cluster_radius}m, 最少点数={self.min_points_per_cluster}",
                throttle_duration_sec=2.0
            )
            
            # 3. 检查聚类后的点数 - 锥桶不足时归类为黄线弯
            if len(dedup_points) < 3:
                self.get_logger().warn(f">>>【黄线左转触发 - 锥桶数量不足】聚类后锥桶中心点({len(dedup_points)}个) < 3个", throttle_duration_sec=1.0)
                self.set_yellow_curve_signal(f"聚类后锥桶中心点({len(dedup_points)}个) < 3个")
            
            # 4. 进行内外道分离 (已修复 S 弯)
            elif len(dedup_points) >= 3:
                self.inner_x, self.inner_y, self.outer_x, self.outer_y = self.separate_inner_outer_lanes(dedup_points)
                
                # 更新上一帧数据（用于特殊模式下的可视化）
                self.last_inner_x, self.last_inner_y = self.inner_x.copy(), self.inner_y.copy()
                self.last_outer_x, self.last_outer_y = self.outer_x.copy(), self.outer_y.copy()
                
                # [插入开始] ==========================================
                # 优先进行视觉检测：如果检测到正前方两个蓝色锥桶（一左一右），直接判定为直角弯
                if self.blue_cones_detected:
                    self.get_logger().warn(f">>>【直角弯触发 - 视觉检测】检测到正前方两个蓝色锥桶（一左一右）", throttle_duration_sec=1.0)
                    self.target_x = -20000.0
                    self.target_y = -20000.0
                    self.trigger_condition = "视觉检测：正前方两个蓝色锥桶（直角弯标志）"
                    self.publish_target()
                    return
                
                # 其次进行几何特征检测：如果外道形成"墙"，直接判定为直角弯，跳过后续点数判断
                if self.detect_wall_geometry(self.outer_x, self.outer_y):
                    self.get_logger().warn(f">>>【直角弯触发 - 几何特征】外道形成封路墙 (Wall Detection)", throttle_duration_sec=1.0)
                    self.target_x = -20000.0  # 设置直角弯信号
                    self.target_y = -20000.0
                    self.trigger_condition = "外道几何特征显示前方封路 (Wall Detection)"
                    
                    # 保留点列表用于可视化，直接发布并返回，不再执行后续的 if/elif 点数统计
                    self.publish_target()
                    return
                # [插入结束] ==========================================
                
                # 5. 检查分离结果 - 简化判断逻辑
                inner_count = len(self.inner_x)
                outer_count = len(self.outer_x)
                
                if inner_count < 2:
                    # 内道不足的情况
                    if outer_count == 3 and inner_count == 1:
                        # 特殊情况：内道=1 且 外道=3 → 保持当前状态
                        self.target_x = -40000.0
                        self.target_y = -40000.0
                        self.trigger_condition = f"内道点({inner_count}个) = 1 且 外道点({outer_count}个) = 3"
                        self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
                    else:
                        # 内道<2 且 外道≥3 → 直角弯（内道缺失，外道有足够点）
                        self.get_logger().warn(f">>>【直角弯触发】内道点({inner_count}个) < 2 且 外道点({outer_count}个) ≥ 3", throttle_duration_sec=1.0)
                        self.set_right_angle_signal(f"内道点({inner_count}个) < 2 且 外道点({outer_count}个) ≥ 3")
                
                # 6. 外道不足的情况（内道≥2，但外道<2）
                elif outer_count < 2:
                    # 先尝试降级策略：可能是挡板场景，外道点被过滤
                    self.get_logger().warn(
                        f">>> [降级策略触发] 外道点({outer_count}个) < 2（内道点{inner_count}个），尝试使用左右锥桶分布计算目标点",
                        throttle_duration_sec=1.0
                    )
                    
                    # 保存原始内外道数据用于降级策略
                    original_inner_x = self.inner_x.copy()
                    original_inner_y = self.inner_y.copy()
                    original_outer_x = self.outer_x.copy()
                    original_outer_y = self.outer_y.copy()
                    
                    # 尝试降级策略
                    fallback_x, fallback_y = self.get_fallback_target_point(
                        original_inner_x, original_inner_y, 
                        original_outer_x, original_outer_y
                    )
                    
                    if fallback_x != -20000.0:
                        # 降级策略成功，使用降级目标点
                        self.target_x = fallback_x
                        self.target_y = fallback_y
                        self.trigger_condition = f"降级策略（外道点不足），目标点({self.target_x:.2f}, {self.target_y:.2f})"
                    else:
                        # 降级策略失败，触发向右修正
                        self.get_logger().warn(f">>>【向右修正触发】外道点({outer_count}个) < 2（内道点{inner_count}个正常）且降级策略失败", throttle_duration_sec=1.0)
                        self.target_x = -30000.0
                        self.target_y = -30000.0
                        self.trigger_condition = f"外道点({outer_count}个) < 2（内道点{inner_count}个正常）且降级策略失败"
                    self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
                
                # 7. 循迹成功（内道≥2 且 外道≥2）
                else:
                    # 保存原始内外道数据（用于降级策略）
                    original_inner_x = self.inner_x.copy()
                    original_inner_y = self.inner_y.copy()
                    original_outer_x = self.outer_x.copy()
                    original_outer_y = self.outer_y.copy()
                    
                    # 检查是否需要圆形滤波
                    flag_filter = 0
                    for y_val in self.outer_y:
                        if y_val > self.min_inner_y_threshold:
                            flag_filter = 1
                            break
                    
                    if len(self.inner_x) <= 2:
                        # 内道点小于等于两个则不做圆形滤波，仅仅过滤y过大的外道点
                        threshold = self.min_inner_y_threshold
                        indices = [i for i in range(len(self.outer_x)) if self.outer_y[i] <= threshold]
                        real_out_x = [self.outer_x[i] for i in indices]
                        real_out_y = [self.outer_y[i] for i in indices]
                    elif flag_filter == 0:
                        # 仅对于出现异常外道点时过滤
                        real_out_x = self.outer_x
                        real_out_y = self.outer_y
                    else:
                        # 根据圆形滤波过滤
                        real_out_x, real_out_y = self.fit_circle_and_filter(self.inner_x, self.inner_y, self.outer_x, self.outer_y)
                    
                    # 检查圆形滤波后外道点是否足够
                    if len(real_out_x) < 2:
                        # 圆形滤波后外道点不足，尝试降级策略
                        self.get_logger().warn(
                            f">>> [降级策略触发] 圆形滤波后外道点不足({len(real_out_x)}个)，使用左右锥桶分布计算目标点",
                            throttle_duration_sec=1.0
                        )
                        fallback_x, fallback_y = self.get_fallback_target_point(
                            original_inner_x, original_inner_y, 
                            original_outer_x, original_outer_y
                        )
                        
                        if fallback_x != -20000.0:
                            # 降级策略成功，使用降级目标点
                            self.target_x = fallback_x
                            self.target_y = fallback_y
                            self.trigger_condition = f"降级策略（圆形滤波后外道点不足），目标点({self.target_x:.2f}, {self.target_y:.2f})"
                        else:
                            # 降级策略也失败
                            self.target_x = -20000.0
                            self.target_y = -20000.0
                            self.trigger_condition = "圆形滤波后外道点不足且降级策略失败"
                    else:
                        # === 生成三等分点（基于源程序main.py）===
                        target_group_x, target_group_y = self.gen_three_equinoxes(self.inner_x, self.inner_y, real_out_x, real_out_y)
                        
                        # === 选择目标点（移植自line_follow2）===
                        self.target_x, self.target_y = self.get_target_point(target_group_x, target_group_y)
                        
                        # 调试信息：显示目标点选择结果
                        self.get_logger().info(
                            f">>> [目标点选择] 三等分点数: {len(target_group_x)}, "
                            f"目标点: ({self.target_x:.2f}, {self.target_y:.2f})",
                            throttle_duration_sec=1.0
                        )
                        
                        # 移植自line_follow2：返回值-20000表示无效目标点
                        if self.target_x == -20000.0:
                            # 检查是三等分点为空还是所有目标点X<0
                            if len(target_group_x) == 0 or len(target_group_y) == 0:
                                # 三等分点为空：尝试降级策略
                                self.get_logger().warn(
                                    ">>> [降级策略触发] 内外道配对失败，使用左右锥桶分布计算目标点",
                                    throttle_duration_sec=1.0
                                )
                                fallback_x, fallback_y = self.get_fallback_target_point(
                                    original_inner_x, original_inner_y, 
                                    original_outer_x, original_outer_y
                                )
                                
                                if fallback_x != -20000.0:
                                    # 降级策略成功，使用降级目标点
                                    self.target_x = fallback_x
                                    self.target_y = fallback_y
                                    self.trigger_condition = f"降级策略（左右锥桶分布），目标点({self.target_x:.2f}, {self.target_y:.2f})"
                                else:
                                    # 降级策略也失败
                                    self.trigger_condition = "三等分点为空且降级策略失败"
                            else:
                                self.trigger_condition = "所有目标点X<0 (没有前方目标点)"
                        elif self.target_x == -30000.0:
                            self.trigger_condition = "向右修正（三等分点为空）"
                        else:
                            # 正常循迹
                            self.trigger_condition = f"正常循迹，目标点({self.target_x:.2f}, {self.target_y:.2f})"
        
        # 2. 如果矩形滤波后点数不足（<3），再进行扇形检测
        # 这样可以避免挡板遮挡正前方时误判为黄线弯
        elif len(valid_x) < 3:
            # 先进行扇形检测
            if self.check_no_cone_sector(valid_ranges, valid_angles):
                # 扇形区域激光点数很少，判定为黄线弯
                self.get_logger().warn(f">>>【黄线左转触发】扇形区域(前方±{self.no_cone_sector_angle}°, {self.no_cone_detection_range}m)内激光点数 ≤ {self.no_cone_sector_threshold}个", throttle_duration_sec=1.0)
                self.set_yellow_curve_signal(f"扇形区域(前方±{self.no_cone_sector_angle}°, {self.no_cone_detection_range}m)内激光点数 ≤ {self.no_cone_sector_threshold}个")
            else:
                # 扇形区域有点，但矩形滤波后点数不足，可能是挡板遮挡
                # 使用降级策略，尝试基于现有点计算目标点
                self.get_logger().warn(f">>>【点数不足但扇形区域有点】矩形滤波后点数({len(valid_x)}个) < 3个，但扇形区域有点（可能是挡板遮挡），尝试降级策略", throttle_duration_sec=1.0)
                
                # 即使点数不足，也尝试简单聚类
                if len(valid_x) > 0:
                    cluster_centers = self.simple_density_cluster(valid_x.tolist(), valid_y.tolist())
                    dedup_points = cluster_centers
                    self.cone_count = len(dedup_points)
                    
                    if len(dedup_points) > 0:
                        # 使用降级策略
                        all_cones = [(dedup_points[i][0], dedup_points[i][1]) for i in range(len(dedup_points))]
                        fallback_x, fallback_y = self.get_fallback_target_point(
                            [p[0] for p in all_cones], [p[1] for p in all_cones],
                            [], []  # 没有内外道分离，使用所有点
                        )
                        
                        if fallback_x != -20000.0:
                            self.target_x = fallback_x
                            self.target_y = fallback_y
                            self.trigger_condition = f"降级策略（挡板遮挡场景），目标点({self.target_x:.2f}, {self.target_y:.2f})"
                        else:
                            self.set_yellow_curve_signal(f"矩形滤波后点数({len(valid_x)}个) < 3个且降级策略失败")
                    else:
                        self.set_yellow_curve_signal(f"矩形滤波后点数({len(valid_x)}个) < 3个")
                else:
                    self.set_yellow_curve_signal(f"矩形滤波后点数({len(valid_x)}个) < 3个")
            
            # 4. 进行内外道分离 (已修复 S 弯)
            if len(dedup_points) >= 3:
                self.inner_x, self.inner_y, self.outer_x, self.outer_y = self.separate_inner_outer_lanes(dedup_points)
                
                # 更新上一帧数据（用于特殊模式下的可视化）
                self.last_inner_x, self.last_inner_y = self.inner_x.copy(), self.inner_y.copy()
                self.last_outer_x, self.last_outer_y = self.outer_x.copy(), self.outer_y.copy()
                
                # [插入开始] ==========================================
                # 优先进行视觉检测：如果检测到正前方两个蓝色锥桶（一左一右），直接判定为直角弯
                if self.blue_cones_detected:
                    self.get_logger().warn(f">>>【直角弯触发 - 视觉检测】检测到正前方两个蓝色锥桶（一左一右）", throttle_duration_sec=1.0)
                    self.target_x = -20000.0
                    self.target_y = -20000.0
                    self.trigger_condition = "视觉检测：正前方两个蓝色锥桶（直角弯标志）"
                    self.publish_target()
                    return
                
                # 其次进行几何特征检测：如果外道形成"墙"，直接判定为直角弯，跳过后续点数判断
                if self.detect_wall_geometry(self.outer_x, self.outer_y):
                    self.get_logger().warn(f">>>【直角弯触发 - 几何特征】外道形成封路墙 (Wall Detection)", throttle_duration_sec=1.0)
                    self.target_x = -20000.0  # 设置直角弯信号
                    self.target_y = -20000.0
                    self.trigger_condition = "外道几何特征显示前方封路 (Wall Detection)"
                    
                    # 保留点列表用于可视化，直接发布并返回，不再执行后续的 if/elif 点数统计
                    self.publish_target()
                    return
                # [插入结束] ==========================================
                
                # 5. 检查分离结果 - 简化判断逻辑
                inner_count = len(self.inner_x)
                outer_count = len(self.outer_x)
                
                if inner_count < 2:
                    # 内道不足的情况
                    if outer_count == 3 and inner_count == 1:
                        # 特殊情况：内道=1 且 外道=3 → 保持当前状态
                        self.target_x = -40000.0
                        self.target_y = -40000.0
                        self.trigger_condition = f"内道点({inner_count}个) = 1 且 外道点({outer_count}个) = 3"
                        self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
                    else:
                        # 内道<2 且 外道≥3 → 直角弯（内道缺失，外道有足够点）
                        self.get_logger().warn(f">>>【直角弯触发】内道点({inner_count}个) < 2 且 外道点({outer_count}个) ≥ 3", throttle_duration_sec=1.0)
                        self.set_right_angle_signal(f"内道点({inner_count}个) < 2 且 外道点({outer_count}个) ≥ 3")
                
                # 6. 外道不足的情况（内道≥2，但外道<2）
                elif outer_count < 2:
                    # 先尝试降级策略：可能是挡板场景，外道点被过滤
                    self.get_logger().warn(
                        f">>> [降级策略触发] 外道点({outer_count}个) < 2（内道点{inner_count}个），尝试使用左右锥桶分布计算目标点",
                        throttle_duration_sec=1.0
                    )
                    
                    # 保存原始内外道数据用于降级策略
                    original_inner_x = self.inner_x.copy()
                    original_inner_y = self.inner_y.copy()
                    original_outer_x = self.outer_x.copy()
                    original_outer_y = self.outer_y.copy()
                    
                    # 尝试降级策略
                    fallback_x, fallback_y = self.get_fallback_target_point(
                        original_inner_x, original_inner_y, 
                        original_outer_x, original_outer_y
                    )
                    
                    if fallback_x != -20000.0:
                        # 降级策略成功，使用降级目标点
                        self.target_x = fallback_x
                        self.target_y = fallback_y
                        self.trigger_condition = f"降级策略（外道点不足），目标点({self.target_x:.2f}, {self.target_y:.2f})"
                    else:
                        # 降级策略失败，触发向右修正
                        self.get_logger().warn(f">>>【向右修正触发】外道点({outer_count}个) < 2（内道点{inner_count}个正常）且降级策略失败", throttle_duration_sec=1.0)
                        self.target_x = -30000.0
                        self.target_y = -30000.0
                        self.trigger_condition = f"外道点({outer_count}个) < 2（内道点{inner_count}个正常）且降级策略失败"
                    self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
                
                # 7. 循迹成功（内道≥2 且 外道≥2）
                else:
                    # 保存原始内外道数据（用于降级策略）
                    original_inner_x = self.inner_x.copy()
                    original_inner_y = self.inner_y.copy()
                    original_outer_x = self.outer_x.copy()
                    original_outer_y = self.outer_y.copy()
                    
                    # 检查是否需要圆形滤波
                    flag_filter = 0
                    for y_val in self.outer_y:
                        if y_val > self.min_inner_y_threshold:
                            flag_filter = 1
                            break
                    
                    if len(self.inner_x) <= 2:
                        # 内道点小于等于两个则不做圆形滤波，仅仅过滤y过大的外道点
                        threshold = self.min_inner_y_threshold
                        indices = [i for i in range(len(self.outer_x)) if self.outer_y[i] <= threshold]
                        real_out_x = [self.outer_x[i] for i in indices]
                        real_out_y = [self.outer_y[i] for i in indices]
                    elif flag_filter == 0:
                        # 仅对于出现异常外道点时过滤
                        real_out_x = self.outer_x
                        real_out_y = self.outer_y
                    else:
                        # 根据圆形滤波过滤
                        real_out_x, real_out_y = self.fit_circle_and_filter(self.inner_x, self.inner_y, self.outer_x, self.outer_y)
                    
                    # 检查圆形滤波后外道点是否足够
                    if len(real_out_x) < 2:
                        # 圆形滤波后外道点不足，尝试降级策略
                        self.get_logger().warn(
                            f">>> [降级策略触发] 圆形滤波后外道点不足({len(real_out_x)}个)，使用左右锥桶分布计算目标点",
                            throttle_duration_sec=1.0
                        )
                        fallback_x, fallback_y = self.get_fallback_target_point(
                            original_inner_x, original_inner_y, 
                            original_outer_x, original_outer_y
                        )
                        
                        if fallback_x != -20000.0:
                            # 降级策略成功，使用降级目标点
                            self.target_x = fallback_x
                            self.target_y = fallback_y
                            self.trigger_condition = f"降级策略（圆形滤波后外道点不足），目标点({self.target_x:.2f}, {self.target_y:.2f})"
                        else:
                            # 降级策略也失败
                            self.target_x = -20000.0
                            self.target_y = -20000.0
                            self.trigger_condition = "圆形滤波后外道点不足且降级策略失败"
                    else:
                        # === 生成三等分点（基于源程序main.py）===
                        target_group_x, target_group_y = self.gen_three_equinoxes(self.inner_x, self.inner_y, real_out_x, real_out_y)
                        
                        # === 选择目标点（移植自line_follow2）===
                        self.target_x, self.target_y = self.get_target_point(target_group_x, target_group_y)
                        
                        # 调试信息：显示目标点选择结果
                        self.get_logger().info(
                            f">>> [目标点选择] 三等分点数: {len(target_group_x)}, "
                            f"目标点: ({self.target_x:.2f}, {self.target_y:.2f})",
                            throttle_duration_sec=1.0
                        )
                        
                        # 移植自line_follow2：返回值-20000表示无效目标点
                        if self.target_x == -20000.0:
                            # 检查是三等分点为空还是所有目标点X<0
                            if len(target_group_x) == 0 or len(target_group_y) == 0:
                                # 三等分点为空：尝试降级策略
                                self.get_logger().warn(
                                    ">>> [降级策略触发] 内外道配对失败，使用左右锥桶分布计算目标点",
                                    throttle_duration_sec=1.0
                                )
                                fallback_x, fallback_y = self.get_fallback_target_point(
                                    original_inner_x, original_inner_y, 
                                    original_outer_x, original_outer_y
                                )
                                
                                if fallback_x != -20000.0:
                                    # 降级策略成功，使用降级目标点
                                    self.target_x = fallback_x
                                    self.target_y = fallback_y
                                    self.trigger_condition = f"降级策略（左右锥桶分布），目标点({self.target_x:.2f}, {self.target_y:.2f})"
                                else:
                                    # 降级策略也失败
                                    self.trigger_condition = "三等分点为空且降级策略失败"
                            else:
                                self.trigger_condition = "所有目标点X<0 (没有前方目标点)"
                        elif self.target_x == -30000.0:
                            self.trigger_condition = "向右修正（三等分点为空）"
                        else:
                            # 正常循迹
                            self.trigger_condition = f"正常循迹，目标点({self.target_x:.2f}, {self.target_y:.2f})"
        
        # === 移植自line_follow2：移除目标点平滑滤波 ===
        # line_follow2没有平滑滤波，直接使用原始目标点，响应更快，适合高速S弯
        # 注释掉平滑滤波逻辑以提升高速S弯表现
        # if self.target_x < -5000.0:
        #     self.last_target_x = 0.0
        #     self.last_target_y = 0.0
        #     self.is_first_frame = True
        # else:
        #     if self.is_first_frame:
        #         self.last_target_x = self.target_x
        #         self.last_target_y = self.target_y
        #         self.is_first_frame = False
        #     else:
        #         self.target_x = self.last_target_x * (1 - self.filter_alpha) + self.target_x * self.filter_alpha
        #         self.target_y = self.last_target_y * (1 - self.filter_alpha) + self.target_y * self.filter_alpha
        #         self.last_target_x = self.target_x
        #         self.last_target_y = self.target_y
        
        # 发布目标点
        self.publish_target()
        
        # 实时绘图（使用当前帧的数据，包括特殊模式下保留的上一帧锥桶点）
        if self.enable_mapping:
            self.mapping_counter += 1
            if self.mapping_counter >= self.mapping_interval:
                # 确保使用最新的数据
                viz_inner_x = self.inner_x if len(self.inner_x) > 0 else self.last_inner_x
                viz_inner_y = self.inner_y if len(self.inner_y) > 0 else self.last_inner_y
                viz_outer_x = self.outer_x if len(self.outer_x) > 0 else self.last_outer_x
                viz_outer_y = self.outer_y if len(self.outer_y) > 0 else self.last_outer_y
                self.mapping(
                    self.valid_x, self.valid_y,           
                    viz_inner_x, viz_inner_y,           
                    viz_outer_x, viz_outer_y,           
                    [self.target_x], [self.target_y]      
                )
                self.mapping_counter = 0
        
        # 调试信息
        # if self.target_x == -99999.0:
        #     self.get_logger().info("【模式】: 已到达第一个目标停止区域（红灯识别区）", throttle_duration_sec=0.5)
        # elif self.target_x == -88888.0:
        #     self.get_logger().info("【模式】: 已到达第二个目标停止区域(-9.25, -6.86)", throttle_duration_sec=0.5)
        # elif self.target_x == -10000.0:
        #     print(f"[DEBUG] target_x={self.target_x}, 黄线弯模式被激活", flush=True)
        #     self.get_logger().info("【模式】: 黄线弯模式", throttle_duration_sec=0.5)
        # elif self.target_x == -20000.0:
        #     self.get_logger().info("【模式】: 直角弯/循迹失败", throttle_duration_sec=0.5)
        # elif self.target_x == -30000.0:
        #     self.get_logger().info("【模式】: 循迹失败 (仅内道)，触发向右修正", throttle_duration_sec=0.5)
        # elif self.target_x == -40000.0:
        #     self.get_logger().info("【模式】: 内道不足，保持当前状态", throttle_duration_sec=0.5)
        # else:
        #     s_curve_detected = abs(self.target_y) > 0.6
        #     curve_type = "S弯" if s_curve_detected else "普通弯道"
        #     self.get_logger().info(
        #         f'【模式】: 正常循迹 | 目标点: X={self.target_x:.2f}, Y={self.target_y:.2f} | 锥桶: {self.cone_count} | '
        #         f'内: {len(self.inner_x)}, 外: {len(self.outer_x)} | {curve_type}',
        #         throttle_duration_sec=0.5
        #     )
    
    def print_mode_info(self):
        """打印当前模式信息到控制台"""
        if self.frame_count % self.mapping_interval == 0:
            self.get_logger().info("=" * 80)
            self.get_logger().info(f"【当前模式】: {self.current_mode}")
            self.get_logger().info(f"【触发条件】: {self.trigger_condition}")
            self.get_logger().info(f"【目标点】: ({self.target_x:.2f}, {self.target_y:.2f})")
            self.get_logger().info(f"【锥桶数量】: {self.cone_count}个")
            if self.target_x == -10000:
                self.get_logger().info("【控制指令】: → 黄线弯左转（固定角度，保持当前速度）")
            elif self.target_x == -20000:
                self.get_logger().info("【控制指令】: → 直角弯左转（首帧保持速度，后续20档）")
            elif self.target_x == -30000:
                self.get_logger().info("【控制指令】: → 向右修正（三等分点为空）")
            elif self.target_x == -40000:
                self.get_logger().info("【控制指令】: → 保持当前状态")
            else:
                self.get_logger().info("【控制指令】: → PID循迹控制")
            self.get_logger().info("=" * 80)
    
    def publish_target(self):
        """发布目标点（基于源程序main.py的发布格式）"""
        # 打印当前模式信息到控制台
        self.print_mode_info()
        
        msg = Float32MultiArray()
        # V7: 修改逻辑，允许 -10000 和 -20000 通过
        target_x = float(self.target_x)
        target_y = float(self.target_y)
        msg.data = [target_x, target_y]
        self.target_pub.publish(msg)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

