#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯感知节点 - 基于NCSC2024源程序main.py
- 激光雷达锥桶检测与聚类
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
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from threading import Lock
import numpy as np
from sklearn.cluster import KMeans
import math
import copy
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge


class PerceptionNode(Node):
    """纯感知节点 - 锥桶检测与目标点计算"""
    
    def __init__(self):
        super().__init__('perception_node')
        
        # --- 声明ROS2参数（可视化相关参数可从命令行传入）---
        self.declare_parameter('enable_mapping', False)
        self.declare_parameter('show_window', True)
        self.declare_parameter('mapping_interval', 5)
        
        # --- 参数硬编码（所有参数已从配置文件移动到代码中）---
        # ====================== 扇形检测参数 ======================
        self.no_cone_detection_range = 2.0    # 扇形检测距离(m)
        self.no_cone_sector_angle = 84.0      # 扇形检测半角(度)
        self.no_cone_sector_threshold = 1     # 扇形区域内的噪声点容忍数量
        self.no_cone_sector_angle_rad = self.no_cone_sector_angle * (math.pi / 180.0)

        # 激光雷达校准参数（修正位置不准确问题）
        self.lidar_angle_offset = 0.0      # 角度偏移（弧度），正值为逆时针旋转
        self.lidar_distance_offset = 0.0   # 距离偏移（米），正值为增加距离
        self.lidar_x_offset = 0.0         # X方向偏移（米），正值为前方
        self.lidar_y_offset = 0.0         # Y方向偏移（米），正值为左侧
        
        # 激光雷达滤波参数（对齐line_follow2）
        self.min_range = 0.2          # 最小有效极径
        self.max_range = 3.5          # 最大有效极径
        self.left_width = 2.5         # 矩形滤波左侧宽度（对齐line_follow2的rWidth=1.8）
        self.right_width = 1.0        # 矩形滤波右侧宽度
        self.forward_length = 2.5     # 矩形滤波前方长度
        self.back_length = 0.5        # 矩形滤波后方长度
        
        # KMeans聚类参数
        self.n_clusters = 10          # 聚类个数
        self.dedup_radius = 0.25      # 中心点去重半径
        self.kmeans_max_iter = 300    # KMeans最大迭代次数（高速优化：默认300，降低到100以提升速度）
        self.min_cone_distance = 0.3  # 最小锥桶距离（米），过滤掉距离车辆太近的虚假点（如车辆自身反射）
        
        # 内外道分离参数（对齐line_follow2）
        self.inner_dist_sq = 1.69     # 内道扩展距离平方（1.4^2，对齐line_follow2的innerDist）
        self.inner_y_threshold_sq = 1.21  # 内道扩展Y方向距离平方（1.1^2），限制新点在旧点左右方向1.1米内
        self.cut_length_y = 0.82      # 削除圆的直边到圆心Y距离（从0.7改为0.82，允许更大的Y方向扩展）
        self.cut_length_x = -0.7      # 削除圆的直边到圆心X距离（对齐line_follow2的cutLengthX=-0.7）
        self.inner_to_outer_x_threshold = 0.3  # 内道点转外道的X方向阈值（前后0.3米）
        self.inner_to_outer_y_threshold = 1.3  # 内道点转外道的Y方向阈值（左侧1.3米内）
        
        # 圆形滤波参数
        self.max_radius = 7.0         # 圆形滤波最大半径
        self.max_y_filter = 0.5       # 外道Y坐标过滤阈值
        self.min_inner_y_threshold = 0.8  # 内道点不足时的Y坐标过滤阈值
        
        # 目标点选择参数
        self.target_jump = 3          # 目标点选择跳数（实际使用时会减1，配置3实际选择第2近的点）
        
        # 直角弯几何检测参数（已注释：暂时禁用几何检测功能）
        # self.wall_detection_min_distance = 1.3  # 墙检测最小距离（米，从1.6放宽到1.3）
        # self.wall_detection_max_distance = 2.2  # 墙检测最大距离（米，从1.9放宽到2.2，扩大检测范围）
        
        # 可视化参数（从ROS2参数系统读取）
        self.enable_mapping = self.get_parameter('enable_mapping').get_parameter_value().bool_value
        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.mapping_interval = self.get_parameter('mapping_interval').get_parameter_value().integer_value
        self.save_path = '/tmp/map.png'  # 图片保存路径
        
        # --- 状态变量 ---
        self.target_x = -10000.0
        self.target_y = -10000.0
        self.inner_x = []
        self.inner_y = []
        self.outer_x = []
        self.outer_y = []
        self.cone_count = 0
        # 用于可视化的上一帧数据（在特殊模式下保留显示）
        self.last_inner_x = []
        self.last_inner_y = []
        self.last_outer_x = []
        self.last_outer_y = []
        self.current_mode = "初始化"  # 当前模式
        self.trigger_condition = "初始化中"  # 触发条件
        self.frame_count = 0  # 帧计数器
        
        
        # 位置信息（用于目标区域停止功能）
        self.pos_x = 0.0        # 原始里程计坐标（用于停车区域判断，保持与硬编码坐标一致）
        self.pos_y = 0.0
        self.yaw = 0.0           # 小车姿态角（yaw角，弧度）
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
            qos_profile_sensor  # 复用 laser scan 的 QoS 配置
        )
        
        # 发布目标点（基于源程序main.py的Float32MultiArray格式）
        self.target_pub = self.create_publisher(Float32MultiArray, '/target', 10)
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("纯感知节点已启动（基于NCSC2024源程序）")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"扇形检测: 距离={self.no_cone_detection_range}m, 角度=±{self.no_cone_sector_angle}°")
        self.get_logger().info(f"激光雷达滤波: 距离[{self.min_range}, {self.max_range}]m")
        self.get_logger().info(f"矩形滤波: 左{self.left_width}m, 右{self.right_width}m, 前{self.forward_length}m")
        self.get_logger().info(f"KMeans聚类: {self.n_clusters}个簇, 去重半径{self.dedup_radius}m, 最大迭代{self.kmeans_max_iter}, 最小锥桶距离{self.min_cone_distance}m")
        self.get_logger().info(f"内外道分离: 内道扩展距离²={self.inner_dist_sq}, cut_length_y={self.cut_length_y}")
        self.get_logger().info(f"圆形滤波: 最大半径={self.max_radius}m, Y过滤={self.max_y_filter}m")
        actual_jump = max(1, int(round(self.target_jump)) - 1)  # 实际使用的跳数（配置值减1）
        self.get_logger().info(f"目标点选择: 跳数={self.target_jump}（实际使用第{actual_jump}个最近点）")
        # 已注释：直角弯几何检测功能暂时禁用
        # self.get_logger().info(f"直角弯检测: 墙检测距离[{self.wall_detection_min_distance}, {self.wall_detection_max_distance}]m")
        self.get_logger().info(f"可视化: 启用={self.enable_mapping}, 弹窗={self.show_window}, 间隔={self.mapping_interval}帧")
        self.get_logger().info("=" * 60)
    
    def correct_center(self, centers):
        """
        中心点去重（基于源程序main.py的correctCenter函数）
        输入中心点列表，输出去重后的中心点列表
        """
        dedup = []
        not_processed = [i for i in range(len(centers))]
        
        for i in range(len(centers)):
            if i in not_processed:
                not_processed.remove(i)
                dedup_x = []
                dedup_y = []
                
                for j in range(len(centers)):
                    if j in not_processed:
                        if math.dist(centers[i], centers[j]) > self.dedup_radius:
                            continue
                        not_processed.remove(j)
                        dedup_x.append(centers[j][0])
                        dedup_y.append(centers[j][1])
                
                x = round((sum(dedup_x) + centers[i][0]) / (len(dedup_x) + 1), 3)
                y = round((sum(dedup_y) + centers[i][1]) / (len(dedup_y) + 1), 3)
                if [x, y] not in dedup:
                    dedup.append([x, y])
        
        return dedup
    
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
        生成三等分点
        为每个内道点找到相配对的外道点(最近欧氏距离),从而得到三等分点
        """
        res_x = []
        res_y = []
        ratio = 0.5  # 中线
        
        if len(inner_x) == 0 or len(outer_x) == 0:
            return [], []
        
        # 优化：使用numpy数组加速计算
        inner_points = np.array([[inner_x[i], inner_y[i]] for i in range(len(inner_x))])
        outer_points = np.array([[outer_x[j], outer_y[j]] for j in range(len(outer_x))])
        
        for i in range(len(inner_x)):
            # 使用numpy向量化计算距离，比循环快
            inner_p = inner_points[i]
            dists_sq = np.sum((outer_points - inner_p) ** 2, axis=1)
            min_index = np.argmin(dists_sq)
            
            # 直接计算中点
            res_x.append(inner_x[i] + ratio * (outer_x[min_index] - inner_x[i]))
            res_y.append(inner_y[i] + ratio * (outer_y[min_index] - inner_y[i]))
        
        return res_x, res_y
    
    def get_target_point(self, target_group_x, target_group_y):
        """
        根据三等分点选择目标点
        """
        if len(target_group_x) == 0 or len(target_group_y) == 0:
            return -20000.0, -20000.0
        
        jump = max(1, int(round(self.target_jump)) - 1)
        
        # === 修改开始：增加最小预瞄距离过滤 ===
        # 强制忽略车头前方 0.8米 内的目标点，防止盯着脚下看
        # 你可以根据车速调整这个值：车速越快，这个值要越大(0.8 ~ 1.5)
        min_lookahead = 0.8 
        
        forward_points = [(target_group_x[i], target_group_y[i]) 
                         for i in range(len(target_group_x)) 
                         if target_group_x[i] >= min_lookahead] # 这里从 0 改为 min_lookahead
        
        # 如果过滤完没点子了（比如到了死胡同或者只有近处有点），那还是得降级用近处的点
        if not forward_points:
            # 降级逻辑：如果没有远点，就找所有前方点(x>=0)
            forward_points = [(target_group_x[i], target_group_y[i]) 
                             for i in range(len(target_group_x)) if target_group_x[i] >= 0]
        
        if not forward_points:
            return -20000.0, -20000.0
        # === 修改结束 ===
        
        # 优化：使用numpy.argsort替代快速排序，速度更快
        dist_sq = np.array([self.euclidean_dist_sq(p[0], p[1], 0, 0) for p in forward_points])
        sorted_indices = np.argsort(dist_sq).tolist()
        
        # 得到目标点
        if len(sorted_indices) < jump:
            selected_index = sorted_indices[-1]  # 点不够，选最远的(前方)点
        else:
            selected_index = sorted_indices[jump - 1]  # 选第jump近的点
        
        return forward_points[selected_index]
    
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
                
                # 显示小车姿态角和目标点坐标信息（最后绘制，确保不被遮挡）
                info_text = []
                # 显示小车当前位置和姿态角
                try:
                    with self.odom_lock:
                        pos_x = self.pos_x
                        pos_y = self.pos_y
                        yaw = self.yaw
                    
                    yaw_deg = math.degrees(yaw) if yaw is not None else 0.0
                    info_text.append(f'Vehicle Position: ({pos_x:.2f}, {pos_y:.2f}) m')
                    info_text.append(f'Vehicle Yaw: {yaw_deg:.1f}deg ({yaw:.3f}rad)')
                except Exception as e:
                    info_text.append(f'Vehicle Info: Error ({str(e)})')
                
                # 显示目标点坐标
                if hasattr(self, 'target_x') and hasattr(self, 'target_y'):
                    if self.target_x == -10000.0:
                        info_text.append(f'Target: Yellow Curve Mode')
                    elif self.target_x == -20000.0:
                        info_text.append(f'Target: Right Angle Turn Mode')
                    elif self.target_x == -30000.0:
                        info_text.append(f'Target: Right Correction Mode')
                    elif self.target_x == -40000.0:
                        info_text.append(f'Target: Keep Current State')
                    elif self.target_x == -99999.0:
                        info_text.append(f'Target: Stop Area 1 (Red Light)')
                    elif self.target_x == -88888.0:
                        info_text.append(f'Target: Stop Area 2')
                    else:
                        info_text.append(f'Target: ({self.target_x:.2f}, {self.target_y:.2f}) m')
                
                # 在左上角显示信息文本框（最后绘制，确保不被覆盖）
                if len(info_text) > 0:
                    info_str = '\n'.join(info_text)
                    # 添加调试日志
                    self.get_logger().info(f'准备显示信息: {info_str[:50]}...', throttle_duration_sec=2.0)
                    try:
                        # 使用绝对坐标位置（在数据坐标系中）而不是相对坐标
                        # 在左上角(-7, 7)位置显示文本
                        ax.text(-7.5, 7.5, info_str, 
                               fontsize=9,
                               verticalalignment='top',
                               horizontalalignment='left',
                               bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.95, edgecolor='red', linewidth=2),
                               zorder=1000)  # 确保在最上层显示
                        self.get_logger().debug('信息文本已添加', throttle_duration_sec=5.0)
                    except Exception as e:
                        self.get_logger().error(f'显示信息文本失败: {str(e)}')

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
                
                # 显示小车姿态角和目标点坐标信息（保存图片模式）
                info_text = []
                # 显示小车当前位置和姿态角
                with self.odom_lock:
                    pos_x = self.pos_x
                    pos_y = self.pos_y
                    yaw = self.yaw
                
                yaw_deg = math.degrees(yaw)
                info_text.append(f'Vehicle Position: ({pos_x:.2f}, {pos_y:.2f}) m')
                info_text.append(f'Vehicle Yaw: {yaw_deg:.1f}° ({yaw:.3f} rad)')
                
                # 显示目标点坐标
                if hasattr(self, 'target_x') and hasattr(self, 'target_y'):
                    if self.target_x == -10000.0:
                        info_text.append(f'Target: Yellow Curve Mode')
                    elif self.target_x == -20000.0:
                        info_text.append(f'Target: Right Angle Turn Mode')
                    elif self.target_x == -30000.0:
                        info_text.append(f'Target: Right Correction Mode')
                    elif self.target_x == -40000.0:
                        info_text.append(f'Target: Keep Current State')
                    elif self.target_x == -99999.0:
                        info_text.append(f'Target: Stop Area 1 (Red Light)')
                    elif self.target_x == -88888.0:
                        info_text.append(f'Target: Stop Area 2')
                    else:
                        info_text.append(f'Target: ({self.target_x:.2f}, {self.target_y:.2f}) m')
                
                # 在左上角显示信息文本框（保存图片模式）
                if len(info_text) > 0:
                    info_str = '\n'.join(info_text)
                    try:
                        # 使用绝对坐标位置
                        plt.text(-7.5, 7.5, info_str, 
                                fontsize=9,
                                verticalalignment='top',
                                horizontalalignment='left',
                                bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.95, edgecolor='red', linewidth=2))
                    except Exception as e:
                        self.get_logger().warn(f'显示信息文本失败: {str(e)}')
                
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
    
    def to_global_coordinate(self, local_x, local_y):
        """
        将车体坐标系下的点转换到全局坐标系
        参数：
            local_x, local_y: 车体坐标系下的坐标（雷达坐标系）
        返回：
            global_x, global_y: 全局坐标系下的坐标
        """
        with self.odom_lock:
            car_x = self.pos_x
            car_y = self.pos_y
            car_yaw = self.yaw
        
        # 旋转矩阵：车体坐标系 → 全局坐标系
        cos_yaw = math.cos(car_yaw)
        sin_yaw = math.sin(car_yaw)
        
        # 先旋转，再平移
        global_x = car_x + local_x * cos_yaw - local_y * sin_yaw
        global_y = car_y + local_x * sin_yaw + local_y * cos_yaw
        
        return global_x, global_y
    
    def to_local_coordinate(self, global_x, global_y):
        """
        将全局坐标系下的点转换到车体坐标系
        参数：
            global_x, global_y: 全局坐标系下的坐标
        返回：
            local_x, local_y: 车体坐标系下的坐标
        """
        with self.odom_lock:
            car_x = self.pos_x
            car_y = self.pos_y
            car_yaw = self.yaw
        
        # 先平移，再旋转
        dx = global_x - car_x
        dy = global_y - car_y
        
        cos_yaw = math.cos(car_yaw)
        sin_yaw = math.sin(car_yaw)
        
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        return local_x, local_y

    # 已注释：直角弯几何检测功能暂时禁用
    # def detect_wall_geometry(self, x_list, y_list):
    #     """
    #     几何特征检测：检查外道点是否在远处形成一堵墙（直角弯入口特征）
    #     参数由实际雷达图测算：远处黄色锥桶 X 差异极小(扁)，Y 跨度大(宽)。
    #     """
    #     # 1. 数量预筛：如果外道点太少，构不成墙
    #     if len(x_list) < 3:
    #         self.get_logger().debug(f">>> [几何检测] 外道点数量不足: {len(x_list)} < 3", throttle_duration_sec=2.0)
    #         return False
    #
    #     # 2. 距离过滤：只提取在 [min_distance, max_distance] 区间内的点，排除车身旁边的干扰和过远的点
    #     far_indices = [i for i, x in enumerate(x_list) if self.wall_detection_min_distance < x < self.wall_detection_max_distance]
    #     
    #     # 如果远处点不足3个，无法判断特征
    #     if len(far_indices) < 3:
    #         self.get_logger().debug(
    #             f">>> [几何检测] 距离范围内的点不足: 外道总数={len(x_list)}, "
    #             f"距离范围[{self.wall_detection_min_distance}, {self.wall_detection_max_distance}]m内的点={len(far_indices)} < 3",
    #             throttle_duration_sec=2.0
    #         )
    #         return False
    #         
    #     far_x = [x_list[i] for i in far_indices]
    #     far_y = [y_list[i] for i in far_indices]
    #     
    #     # 3. 计算几何包围盒
    #     x_span = max(far_x) - min(far_x)  # 纵向厚度（墙的厚薄）
    #     y_span = max(far_y) - min(far_y)  # 横向跨度（墙的宽度）
    #     
    #     # 4. 核心判断逻辑：
    #     # x_span < 0.6: 墙很扁（雷达图中约为0.13m，设0.6余量充足）
    #     # y_span > 1.5: 墙很宽（雷达图中约为2.5m，设1.5确保封路）
    #     self.get_logger().debug(
    #         f">>> [几何检测] 外道点分析: 总数={len(x_list)}, 范围内={len(far_indices)}, "
    #         f"X厚度={x_span:.2f}m (<0.6?), Y宽度={y_span:.2f}m (>1.5?)",
    #         throttle_duration_sec=1.0
    #     )
    #     
    #     if x_span < 0.6 and y_span > 1.5:
    #         self.get_logger().warn(f">>> [几何检测] 发现直角弯封路墙! X厚度:{x_span:.2f}m, Y宽度:{y_span:.2f}m")
    #         return True
    #     else:
    #         self.get_logger().debug(
    #             f">>> [几何检测] 未满足条件: X厚度={x_span:.2f}m (需要<0.6), Y宽度={y_span:.2f}m (需要>1.5)",
    #             throttle_duration_sec=1.0
    #         )
    #         
    #     return False

    def separate_inner_outer_lanes(self, dedup_points):
        """
        内外道分离（基于源程序main.py的算法）
        先筛选出y>0即小车左侧的点，然后对左侧的点进行扩张，当扩张到长度不变时停止，其他点即为外道
        """
        inner_x = []
        inner_y = []
        outer_x = []
        outer_y = []
        
        if len(dedup_points) < 2:
            return inner_x, inner_y, outer_x, outer_y
        
        not_processed = [i for i in range(len(dedup_points))]
        inner_p = []  # 内圈点
        
        # 固定种子点搜索区域：左侧（Y > 0）
        seed_y_min = 0.0  # 种子点搜索 Y 最小
        seed_y_max = 2.0  # 种子点搜索 Y 最大
        seed_x_box = 1.0  # 种子点搜索 X 范围
        
        minimum_length_sq2 = 10000
        minimum_point = ()
        minimum_i = None
        
        # 在左侧区域找最近的种子点
        for i in range(len(dedup_points)):
            p = dedup_points[i]
            length_sq2 = self.euclidean_dist_sq(p[0], p[1], 0, 0)
            
            if (p[1] > seed_y_min and p[1] < seed_y_max and 
                p[0] > -seed_x_box and p[0] < seed_x_box and 
                length_sq2 < minimum_length_sq2):
                
                minimum_point = p
                minimum_i = i
                minimum_length_sq2 = length_sq2
        # 修改：使用元组 (point, parent_index) 存储内道点，记录每个点的父节点
        # parent_index = -1 表示种子点（没有父节点）
        inner_p_with_parent = []  # 存储 (point, parent_index) 元组
        
        if minimum_i is not None and minimum_point != ():
            inner_p_with_parent.append((minimum_point, -1))  # 种子点的父节点索引为-1
            not_processed.remove(minimum_i)
        
        prev_length = 0
        current_length = len(inner_p_with_parent)
        
        # 长度有改变，表示需要继续扩展
        while prev_length != current_length:
            prev_length = current_length
            ntp = copy.deepcopy(not_processed)
            inp_with_parent = copy.deepcopy(inner_p_with_parent)
            
            for i in not_processed:
                p = dedup_points[i]
                x1 = p[0]
                y1 = p[1]
                
                for j in range(len(inner_p_with_parent)):
                    p1, parent_idx = inner_p_with_parent[j]  # 解包：点和父节点索引
                    # 内道点生长条件：
                    # 1. 距离条件：新点与已有内道点的距离 <= 1.35米（inner_dist_sq=1.8225）
                    # 2. Y方向限制：新点与它的直接来源点（父节点）的Y方向距离 <= 1.1米
                    # 3. 方向过滤：不在削除方向（避免直道边缘误判）
                    # 4. 前后条件：新点可以在已有内道点的前方，或在后方0.5米内（增大范围）
                    y_dist_sq = (y1 - p1[1]) ** 2  # Y方向距离平方（与父节点）
                    if (self.euclidean_dist_sq(x1, y1, p1[0], p1[1]) <= self.inner_dist_sq and 
                        y_dist_sq <= self.inner_y_threshold_sq and  # Y方向距离限制（只检查与父节点）
                        not (p1[1] - y1 > self.cut_length_y and x1 - p1[0] > self.cut_length_x) and 
                        x1 - p1[0] > -0.5):  # 允许后方0.5米内的点加入内道（从0.3米增大到0.5米）
                        ntp.remove(i)
                        inp_with_parent.append((p, j))  # 记录新点和它的父节点索引j
                        break
            
            not_processed = ntp
            inner_p_with_parent = inp_with_parent
            current_length = len(inner_p_with_parent)
        
        # 提取内道点坐标
        for point, parent_idx in inner_p_with_parent:
            inner_x.append(point[0])
            inner_y.append(point[1])
        
        # 剩余点作为外道
        for i in not_processed:
            p = dedup_points[i]
            outer_x.append(p[0])
            outer_y.append(p[1])
        
        # 后处理：检查内道点，如果左侧且前后0.2m内有外道点，则移到外道
        inner_x_filtered = []
        inner_y_filtered = []
        
        for i in range(len(inner_x)):
            inner_px = inner_x[i]
            inner_py = inner_y[i]
            should_move_to_outer = False
            
            # 新规则：如果这个点后方0.8m左右0.1m内有个内道点，那么这个点就不用转为外道点
            has_inner_point_behind = False
            for k in range(len(inner_x)):
                if k == i:  # 跳过自己
                    continue
                other_px = inner_x[k]
                other_py = inner_y[k]
                # 检查是否在后方0.1-0.7m，左右0.1m内
                x_dist = inner_px - other_px  # 正值表示other点在后方
                y_dist_abs = abs(other_py - inner_py)  # Y方向距离
                if (0.1 <= x_dist <= 0.7 and  # 后方0.1-0.7m内
                    y_dist_abs <= 0.1):  # 左右0.1m内
                    has_inner_point_behind = True
                    self.get_logger().info(
                        f">>> [内道转外道保护] 点({inner_px:.2f}, {inner_py:.2f})后方有内道点"
                        f"({other_px:.2f}, {other_py:.2f})，距离X={x_dist:.2f}m Y={y_dist_abs:.2f}m，保留在内道 <<<"
                    )
                    break
            
            # 如果后方有内道点，则不转为外道点
            if has_inner_point_behind:
                should_move_to_outer = False
            else:
                # 检查是否有外道点在该内道点的左侧（Y更大）且前后0.3m内，且左侧1.3m内
                for j in range(len(outer_x)):
                    outer_px = outer_x[j]
                    outer_py = outer_y[j]
                    
                    # 条件1：外道点在内道点的左侧（Y更大）
                    # 条件2：外道点在内道点的前后0.3m内（X方向）
                    # 条件3：外道点在内道点的左侧1.3m内（Y方向距离不超过1.3m）
                    y_dist = outer_py - inner_py  # Y方向距离（正值表示在左侧）
                    if (outer_py > inner_py and  # 左侧（Y更大）
                        abs(outer_px - inner_px) <= self.inner_to_outer_x_threshold and  # 前后0.3m内
                        y_dist <= self.inner_to_outer_y_threshold):  # 左侧1.3m内
                        should_move_to_outer = True
                        break
            
            if should_move_to_outer:
                # 移到外道
                outer_x.append(inner_px)
                outer_y.append(inner_py)
            else:
                # 保留在内道
                inner_x_filtered.append(inner_px)
                inner_y_filtered.append(inner_py)
        
        return inner_x_filtered, inner_y_filtered, outer_x, outer_y
    
    def odom_callback(self, msg: Odometry):
        """订阅里程计信息，更新机器人当前位置和姿态"""
        # 获取原始里程计坐标
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        
        # 从四元数提取姿态角（yaw）
        orientation = msg.pose.pose.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        # 四元数转欧拉角（只提取yaw角）
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
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
            # 保存姿态角
            self.yaw = yaw
    
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
        
        # 极坐标转笛卡尔坐标（添加校准修正）
        i = np.arange(len(ranges))
        angles = msg.angle_min + i * msg.angle_increment
        
        # 应用角度偏移校准
        angles_corrected = angles + self.lidar_angle_offset
        
        valid_i = (ranges > min_R) & (ranges < max_R)
        valid_angles = angles_corrected[valid_i]
        valid_ranges = ranges[valid_i]
        
        # 应用距离偏移校准
        valid_ranges_corrected = valid_ranges + self.lidar_distance_offset
        
        # 转换为笛卡尔坐标
        x = np.round(np.round(valid_ranges_corrected, decimal) * np.cos(valid_angles), decimal)
        y = np.round(np.round(valid_ranges_corrected, decimal) * np.sin(valid_angles), decimal)
        
        # 应用X、Y方向偏移校准
        x = x + self.lidar_x_offset
        y = y + self.lidar_y_offset
        
        # 矩形滤波（Y轴正方向是左侧，负方向是右侧）
        valid_i = (x > -back_length) & (x < forward_length) & (y < left_width) & (y > -right_width)
        valid_x = x[valid_i]
        valid_y = y[valid_i]
        
        points = np.column_stack((valid_x, valid_y))
        
        
        # 保存原始点云数据用于可视化
        self.valid_x = valid_x.tolist()
        self.valid_y = valid_y.tolist()
        
        # === V29 修复：黄线弯/直角弯逻辑分离 ===
        
        # 1. 优先进行黄线弯（扇形）检测
        if self.check_no_cone_sector(valid_ranges, valid_angles):
            self.get_logger().warn(f">>>【黄线左转触发】扇形区域(前方±{self.no_cone_sector_angle}°, {self.no_cone_detection_range}m)内激光点数 ≤ {self.no_cone_sector_threshold}个", throttle_duration_sec=1.0)
            self.set_yellow_curve_signal(f"扇形区域(前方±{self.no_cone_sector_angle}°, {self.no_cone_detection_range}m)内激光点数 ≤ {self.no_cone_sector_threshold}个")
        
        # 2. 如果不是黄线弯，才进行KMeans聚类
        elif len(points) < 3:
            # 点数太少（少于3个），无法进行有效聚类，归类为黄线弯
            self.get_logger().warn(f">>>【黄线左转触发 - 点数不足】矩形滤波后点数({len(points)}个) < 3个", throttle_duration_sec=1.0)
            self.set_yellow_curve_signal(f"矩形滤波后点数({len(points)}个) < 3个")
        else:
            # 动态调整聚类数量：如果点数少于配置的簇数，使用实际点数作为簇数
            actual_n_clusters = min(self.n_clusters, len(points))
            if actual_n_clusters < self.n_clusters:
                self.get_logger().info(f">>> [动态调整聚类数] 点数({len(points)}个) < 配置簇数({self.n_clusters}个)，调整为{actual_n_clusters}个簇 <<<", throttle_duration_sec=2.0)
            
            # 高速优化：减少迭代次数，只运行一次（n_init=1）
            # 这样可以大幅减少计算时间，在高速状态下更及时响应
            cluster = KMeans(
                n_clusters=actual_n_clusters, 
                max_iter=self.kmeans_max_iter,  # 从默认300降到100，提升速度
                n_init=1,                        # 只运行一次，不重复初始化
                init="k-means++",                # 使用k-means++初始化（精度更好）
                random_state=0
            )
            cluster.fit(points)
            dedup_points = self.correct_center(cluster.cluster_centers_.tolist())
            
            # 过滤掉距离车辆太近的虚假点（如车辆自身反射、地面反射等）
            filtered_points = []
            for point in dedup_points:
                distance = math.sqrt(point[0]**2 + point[1]**2)
                if distance >= self.min_cone_distance:
                    filtered_points.append(point)
                else:
                    self.get_logger().debug(
                        f">>> [过滤虚假点] 点({point[0]:.2f}, {point[1]:.2f}) 距离={distance:.2f}m < {self.min_cone_distance}m，已过滤",
                        throttle_duration_sec=2.0
                    )
            
            dedup_points = filtered_points
            self.cone_count = len(dedup_points)
            
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
                # 已注释：暂时禁用几何检测功能
                # 优先进行几何特征检测：如果外道形成"墙"，直接判定为直角弯，跳过后续点数判断
                # if self.detect_wall_geometry(self.outer_x, self.outer_y):
                #     self.get_logger().warn(f">>>【直角弯触发 - 几何特征】外道形成封路墙 (Wall Detection)", throttle_duration_sec=1.0)
                #     self.target_x = -20000.0  # 设置直角弯信号
                #     self.target_y = -20000.0
                #     self.trigger_condition = "外道几何特征显示前方封路 (Wall Detection)"
                #     
                #     # 保留点列表用于可视化，直接发布并返回，不再执行后续的 if/elif 点数统计
                #     self.publish_target()
                #     return
                # [插入结束] ==========================================
                
                # 5. 检查分离结果 - 简化判断逻辑
                inner_count = len(self.inner_x)
                outer_count = len(self.outer_x)
                
                if inner_count < 2:
                    # 内道不足的情况
                    if outer_count < 2:
                        # 内道<2 且 外道<2 → 黄线弯（两侧锥桶都少）
                        self.get_logger().warn(f">>>【黄线左转触发】内道点({inner_count}个) < 2 且 外道点({outer_count}个) < 2", throttle_duration_sec=1.0)
                        self.set_yellow_curve_signal(f"内道点({inner_count}个) < 2 且 外道点({outer_count}个) < 2")
                    elif outer_count == 2:
                        # 内道<2 且 外道=2 → 保持当前状态
                        self.target_x = -40000.0
                        self.target_y = -40000.0
                        self.trigger_condition = f"内道点({inner_count}个) < 2 且 外道点({outer_count}个) = 2"
                        self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
                    else:
                        # 内道<2 且 外道≥3 → 直角弯（内道缺失，外道有足够点）
                        self.get_logger().warn(f">>>【直角弯触发】内道点({inner_count}个) < 2 且 外道点({outer_count}个) ≥ 3", throttle_duration_sec=1.0)
                        self.set_right_angle_signal(f"内道点({inner_count}个) < 2 且 外道点({outer_count}个) ≥ 3")
                
                # 6. 外道不足的情况（内道≥2，但外道<2）
                elif outer_count < 2:
                    self.get_logger().warn(f">>>【向右修正触发】外道点({outer_count}个) < 2（内道点{inner_count}个正常）", throttle_duration_sec=1.0)
                    self.target_x = -30000.0
                    self.target_y = -30000.0
                    self.trigger_condition = f"外道点({outer_count}个) < 2（内道点{inner_count}个正常）"
                    self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
                
                # 7. 循迹成功（内道≥2 且 外道≥2）
                else:
                    # 简化圆形滤波判断，减少延迟
                    # 只在明显异常时才进行圆形滤波
                    if len(self.inner_x) <= 2:
                        # 内道点少，仅过滤y过大的外道点
                        threshold = self.min_inner_y_threshold
                        indices = [i for i in range(len(self.outer_x)) if self.outer_y[i] <= threshold]
                        real_out_x = [self.outer_x[i] for i in indices]
                        real_out_y = [self.outer_y[i] for i in indices]
                    elif any(y > self.min_inner_y_threshold for y in self.outer_y):
                        # 有异常外道点，才进行圆形滤波
                        real_out_x, real_out_y = self.fit_circle_and_filter(self.inner_x, self.inner_y, self.outer_x, self.outer_y)
                    else:
                        # 正常情况，直接使用
                        real_out_x = self.outer_x
                        real_out_y = self.outer_y
                    
                    # === 生成三等分点（基于源程序main.py）===
                    target_group_x, target_group_y = self.gen_three_equinoxes(self.inner_x, self.inner_y, real_out_x, real_out_y)
                    
                    # === 选择目标点（移植自line_follow2）===
                    self.target_x, self.target_y = self.get_target_point(target_group_x, target_group_y)
                    # 移植自line_follow2：返回值-20000表示无效目标点
                    if self.target_x == -20000.0:
                        # 检查是三等分点为空还是所有目标点X<0
                        if len(target_group_x) == 0 or len(target_group_y) == 0:
                            self.trigger_condition = "三等分点为空 (内外道配对失败或外道点被过滤)"
                        else:
                            self.trigger_condition = "所有目标点X<0 (没有前方目标点)"
                    elif self.target_x == -30000.0:
                        self.trigger_condition = "向右修正（三等分点为空）"
                    else:
                        # 正常循迹
                        self.trigger_condition = f"正常循迹，目标点({self.target_x:.2f}, {self.target_y:.2f})"
            
            else: 
                self.target_x = -20000.0 # V29: 改为直角弯/失败信号
                self.target_y = -20000.0
                self.trigger_condition = "圆形滤波后点数不足"
                self.inner_x, self.inner_y, self.outer_x, self.outer_y = [], [], [], []
        
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
        #     self.get_logger().info(
        #         f'【模式】: 正常循迹 | 目标点: X={self.target_x:.2f}, Y={self.target_y:.2f} | 锥桶: {self.cone_count} | '
        #         f'内: {len(self.inner_x)}, 外: {len(self.outer_x)}',
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