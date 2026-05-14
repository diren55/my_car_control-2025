#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化感知节点 - 基于简单聚类和连线方法
- 使用基于密度的简单聚类替代KMeans
- 区分锥桶和挡板
- 内外道连线可视化
- 只处理左右1m范围内的点
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os
import copy


class SimplePerceptionNode(Node):
    """简化感知节点 - 基于简单聚类和连线方法"""
    
    def __init__(self):
        super().__init__('simple_perception_node')
        
        # 声明参数
        self.declare_parameter('min_range', 0.2)          # 最小有效极径
        self.declare_parameter('max_range', 3.5)          # 最大有效极径
        self.declare_parameter('forward_length', 3.0)     # 前方检测长度
        self.declare_parameter('back_length', 0.5)         # 后方检测长度
        self.declare_parameter('lateral_range_left', 1.5)  # 左侧检测范围（米）
        self.declare_parameter('lateral_range_right', 1.0)  # 右侧检测范围（米）
        
        # 简单聚类参数（考虑锥桶直径30cm）
        self.declare_parameter('cluster_radius', 0.2)      # 聚类半径（米）
        self.declare_parameter('min_points_per_cluster', 10)  # 每个簇最少点数
        
        # 直角弯检测参数（新逻辑：左侧有清晰道路，两侧有锥桶）
        self.declare_parameter('right_angle_left_road_min', 0.5)  # 左侧道路检测最小距离（米）
        self.declare_parameter('right_angle_left_road_max', 2.0)  # 左侧道路检测最大距离（米）
        self.declare_parameter('right_angle_left_road_width', 0.8)  # 左侧道路宽度（米，两侧锥桶之间的宽度）
        self.declare_parameter('right_angle_min_cones_per_side', 2)  # 左侧道路每侧最少锥桶数
        self.declare_parameter('right_angle_road_center_y', 0.6)  # 左侧道路中心Y坐标（米，从中心线到道路中心的距离）
        
        # 左右避障参数
        self.declare_parameter('avoidance_lateral_range', 1.0)  # 左右避障检测范围（米）
        self.declare_parameter('avoidance_forward_range', 1.5)  # 前方避障检测范围（米）
        self.declare_parameter('target_forward_distance', 1.0)  # 目标点前方距离（米）
        
        # 目标点选择参数（已移除jump，直接选最近的点）
        
        # 可视化参数
        self.declare_parameter('enable_visualization', True)   # 是否启用可视化
        self.declare_parameter('update_interval', 1)           # 更新间隔（帧数）
        
        # 获取参数
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.forward_length = self.get_parameter('forward_length').value
        self.back_length = self.get_parameter('back_length').value
        self.lateral_range_left = self.get_parameter('lateral_range_left').value
        self.lateral_range_right = self.get_parameter('lateral_range_right').value
        
        self.cluster_radius = self.get_parameter('cluster_radius').value
        self.min_points_per_cluster = self.get_parameter('min_points_per_cluster').value
        
        self.right_angle_left_road_min = self.get_parameter('right_angle_left_road_min').value
        self.right_angle_left_road_max = self.get_parameter('right_angle_left_road_max').value
        self.right_angle_left_road_width = self.get_parameter('right_angle_left_road_width').value
        self.right_angle_min_cones_per_side = self.get_parameter('right_angle_min_cones_per_side').value
        self.right_angle_road_center_y = self.get_parameter('right_angle_road_center_y').value
        
        self.avoidance_lateral_range = self.get_parameter('avoidance_lateral_range').value
        self.avoidance_forward_range = self.get_parameter('avoidance_forward_range').value
        self.target_forward_distance = self.get_parameter('target_forward_distance').value
        
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.update_interval = self.get_parameter('update_interval').value
        
        # 状态变量
        self.frame_count = 0
        self.inner_points = []  # 内道点列表
        self.outer_points = []  # 外道点列表
        self.center_points = []  # 中心点列表
        self.target_x = 0.0
        self.target_y = 0.0
        self.is_right_angle_turn = False  # 是否是直角弯
        self.right_angle_points = []  # 直角弯的四个点
        # 用于可视化的中间数据
        self.filtered_x = []  # 距离滤波后的点
        self.filtered_y = []
        self.rect_x = []  # 矩形滤波后的点
        self.rect_y = []
        self.cluster_centers = []  # 聚类中心（包含点数）
        
        # 初始化可视化
        self.fig = None
        self.ax = None
        self.show_window = False
        
        if self.enable_visualization:
            try:
                import matplotlib
                if 'DISPLAY' not in os.environ or not os.environ.get('DISPLAY'):
                    self.get_logger().warn(">>> DISPLAY环境变量未设置，无法显示弹窗 <<<")
                else:
                    matplotlib.use('TkAgg')
                    plt.ion()
                    self.fig, self.ax = plt.subplots(figsize=(12, 12))
                    self.fig.canvas.manager.set_window_title('Simple Perception - Path Visualization')
                    plt.show(block=False)
                    plt.pause(0.1)
                    self.show_window = True
                    self.get_logger().info(">>> 可视化窗口初始化成功 <<<")
            except Exception as e:
                self.get_logger().error(f">>> 可视化初始化失败: {str(e)} <<<")
        
        # QoS配置
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 发布目标点
        self.target_pub = self.create_publisher(Float32MultiArray, '/target', 10)
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("简化感知节点已启动")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"距离滤波: [{self.min_range}, {self.max_range}]m")
        self.get_logger().info(f"矩形滤波: 前{self.forward_length}m, 后{self.back_length}m, 左{self.lateral_range_left}m, 右{self.lateral_range_right}m")
        self.get_logger().info(f"聚类参数: 半径={self.cluster_radius}m, 最少点数={self.min_points_per_cluster}")
        self.get_logger().info(f"直角弯检测: 左侧道路距离[{self.right_angle_left_road_min}, {self.right_angle_left_road_max}]m, "
                              f"道路宽度={self.right_angle_left_road_width}m, "
                              f"道路中心Y={self.right_angle_road_center_y}m, "
                              f"每侧最少锥桶数={self.right_angle_min_cones_per_side}")
        self.get_logger().info(f"左右避障: 检测范围={self.avoidance_lateral_range}m, "
                              f"前方范围={self.avoidance_forward_range}m, "
                              f"目标距离={self.target_forward_distance}m")
        self.get_logger().info("=" * 60)
    
    def euclidean_dist(self, p1, p2):
        """计算两点间欧氏距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def simple_density_cluster(self, x_list, y_list):
        """
        简单聚类：将小弧（一个锥桶的点云）变成一个点
        返回：聚类中心列表 [(x, y, point_count), ...]
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
                cluster_centers.append((center_x, center_y, len(neighbors)))
                
                # 标记为已处理
                processed.update(neighbors)
        
        return cluster_centers
    
    def detect_right_angle_turn(self, cones):
        """
        检测直角弯 - 新逻辑：检测左侧是否有清晰的道路（两侧有锥桶）
        特征：左侧有清晰的道路通道（内侧和外侧都有锥桶，中间是空的）
        返回：是否是直角弯，以及左侧道路上的锥桶点
        """
        if len(cones) < self.right_angle_min_cones_per_side * 2:
            return False, []
        
        # 筛选左侧一定距离范围内的锥桶（0.5-2米）
        left_road_cones = [(x, y) for x, y in cones
                          if self.right_angle_left_road_min < x < self.right_angle_left_road_max
                          and y > 0 and y < 1.5]  # 左侧Y>0
        
        if len(left_road_cones) < self.right_angle_min_cones_per_side * 2:
            return False, []
        
        # 将左侧锥桶分为内侧和外侧
        # 内侧：Y坐标较小（靠近中心线）
        # 外侧：Y坐标较大（远离中心线）
        road_center_y = self.right_angle_road_center_y
        road_half_width = self.right_angle_left_road_width / 2
        
        inner_side_cones = [(x, y) for x, y in left_road_cones
                           if road_center_y - road_half_width < y < road_center_y + road_half_width]
        outer_side_cones = [(x, y) for x, y in left_road_cones
                           if y > road_center_y + road_half_width]
        
        # 检查是否形成清晰的道路：
        # 1. 内侧有足够的锥桶（至少2个）
        # 2. 外侧有足够的锥桶（至少2个）
        # 3. 中间区域（道路中心）相对空旷
        
        if len(inner_side_cones) < self.right_angle_min_cones_per_side:
            return False, []
        
        if len(outer_side_cones) < self.right_angle_min_cones_per_side:
            return False, []
        
        # 检查道路中心是否相对空旷（中心区域锥桶较少）
        center_zone_cones = [(x, y) for x, y in left_road_cones
                           if abs(y - road_center_y) < road_half_width * 0.5]
        
        # 如果中心区域锥桶太多，可能不是清晰的道路
        if len(center_zone_cones) > len(inner_side_cones) + len(outer_side_cones):
            return False, []
        
        # 满足条件：左侧有清晰的道路（两侧有锥桶，中间相对空旷）
        self.get_logger().info(
            f"检测到直角弯（左侧有清晰道路）: "
            f"内侧={len(inner_side_cones)}个点, 外侧={len(outer_side_cones)}个点, "
            f"中心区域={len(center_zone_cones)}个点",
            throttle_duration_sec=1.0
        )
        
        # 返回左侧道路上的所有锥桶点（按X坐标排序）
        sorted_road_cones = sorted(left_road_cones, key=lambda p: p[0])
        
        return True, sorted_road_cones
    
    def separate_inner_outer(self, cones):
        """
        分离内外道
        简单方法：根据Y坐标判断（Y>0为左侧/内道，Y<0为右侧/外道）
        但需要考虑转向，使用更智能的方法
        """
        if len(cones) == 0:
            return [], []
        
        # 统计左右两侧点数（cones现在是(x, y)元组）
        left_count = sum(1 for c in cones if len(c) >= 2 and c[1] > 0.1)
        right_count = sum(1 for c in cones if len(c) >= 2 and c[1] < -0.1)
        
        # 判断转向
        is_right_turn = (right_count > left_count + 2) and (right_count > 0)
        
        inner_points = []
        outer_points = []
        
        for cone in cones:
            # cones现在是(x, y)元组
            if len(cone) < 2:
                continue
            x, y = cone[0], cone[1]
            
            # 根据转向判断内外道
            if is_right_turn:
                # 右转：内道在右侧（Y<0）
                if y < -0.1:
                    inner_points.append((x, y))
                else:
                    outer_points.append((x, y))
            else:
                # 左转或直道：内道在左侧（Y>0）
                if y > 0.1:
                    inner_points.append((x, y))
                else:
                    outer_points.append((x, y))
        
        return inner_points, outer_points
    
    def create_path_lines(self, inner_points, outer_points):
        """
        创建路径连线（按距离排序，连最近的点）
        返回：内道路径点、外道路径点、中心路径点
        """
        # 按距离排序（从车辆位置开始，最近的点先连）
        inner_sorted = sorted(inner_points, key=lambda p: math.sqrt(p[0]**2 + p[1]**2))
        outer_sorted = sorted(outer_points, key=lambda p: math.sqrt(p[0]**2 + p[1]**2))
        
        # 连线：按距离顺序连接所有点（不跳过任何点）
        inner_connected = self.connect_nearest_points(inner_sorted)
        outer_connected = self.connect_nearest_points(outer_sorted)
        
        # 生成中心点（基于连接后的点）
        center_points = []
        min_len = min(len(inner_connected), len(outer_connected))
        
        for i in range(min_len):
            center_x = (inner_connected[i][0] + outer_connected[i][0]) / 2
            center_y = (inner_connected[i][1] + outer_connected[i][1]) / 2
            center_points.append((center_x, center_y))
        
        return inner_connected, outer_connected, center_points
    
    def connect_nearest_points(self, points):
        """
        连线：按距离顺序连接所有点，总是连最近的点
        添加约束：避免横向连接（左右两侧的点不应该连接）
        """
        if len(points) <= 1:
            return points
        
        # 从最近的点开始
        connected = [points[0]]  # 第一个点（最近的）
        remaining = points[1:]  # 剩余的点
        
        # 贪心算法：每次找距离当前路径末端最近的点
        while len(remaining) > 0:
            if len(connected) == 0:
                break
            
            last_point = connected[-1]
            min_dist = float('inf')
            nearest_idx = 0
            
            # 找到距离最后一个点最近的点（但要满足约束条件）
            for i, point in enumerate(remaining):
                dist = self.euclidean_dist(last_point, point)
                
                # 约束1：如果两个点在中线两侧，且距离较远，不连接
                # 这样可以避免左侧第三个锥桶和右侧锥桶连接
                if (last_point[1] * point[1] < 0):  # 一个在左侧(Y>0)，一个在右侧(Y<0)
                    if dist > 0.8:  # 距离较远，不连接
                        continue
                
                # 约束2：如果连线主要是横向的（Y方向变化大，X方向变化小），可能是错误连接
                dx = abs(point[0] - last_point[0])
                dy = abs(point[1] - last_point[1])
                if dy > dx * 1.5 and dist > 0.6:  # 横向连线且距离较远，不连接
                    continue
                
                # 约束3：如果连线会穿过车辆前方路径（X>0的区域），且是横向的，不连接
                if last_point[0] > 0.5 and point[0] > 0.5:  # 都在车辆前方
                    if dy > 0.8 and dx < 0.5:  # 横向跨度大，纵向跨度小，可能是错误连接
                        continue
                
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
            
            # 如果找到了满足条件的点，连接它
            if min_dist < float('inf'):
                connected.append(remaining[nearest_idx])
                remaining.pop(nearest_idx)
            else:
                # 如果找不到满足条件的点，选择最近的点（即使不满足约束）
                # 这样可以确保所有点都被连接
                min_dist = float('inf')
                nearest_idx = 0
                for i, point in enumerate(remaining):
                    dist = self.euclidean_dist(last_point, point)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_idx = i
                connected.append(remaining[nearest_idx])
                remaining.pop(nearest_idx)
        
        return connected
    
    def select_target_point_avoidance(self, cones):
        """
        基于左右两侧锥桶的避障逻辑
        根据左右1米范围内的锥桶来决定目标点位置
        返回：目标点坐标
        """
        if len(cones) == 0:
            return -10000.0, -10000.0
        
        # 筛选左右1米范围内、前方1.5米内的锥桶
        avoidance_cones = [(x, y) for x, y in cones
                          if abs(y) < self.avoidance_lateral_range
                          and x > 0 and x < self.avoidance_forward_range]
        
        if len(avoidance_cones) == 0:
            # 没有检测到锥桶，直行
            return self.target_forward_distance, 0.0
        
        # 分离左右两侧的锥桶
        left_cones = [(x, y) for x, y in avoidance_cones if y > 0]
        right_cones = [(x, y) for x, y in avoidance_cones if y < 0]
        
        # 计算左右两侧的平均Y坐标（用于判断偏向）
        left_avg_y = np.mean([y for x, y in left_cones]) if len(left_cones) > 0 else 0.0
        right_avg_y = np.mean([y for x, y in right_cones]) if len(right_cones) > 0 else 0.0
        
        # 计算左右两侧的平均X坐标（用于判断距离）
        left_avg_x = np.mean([x for x, y in left_cones]) if len(left_cones) > 0 else float('inf')
        right_avg_x = np.mean([x for x, y in right_cones]) if len(right_cones) > 0 else float('inf')
        
        # 计算目标点Y坐标（避障逻辑）
        target_y = 0.0
        
        if len(left_cones) > 0 and len(right_cones) > 0:
            # 两侧都有锥桶：走中间，稍微偏向锥桶少的一侧
            if len(left_cones) < len(right_cones):
                # 左侧锥桶少，稍微偏左
                target_y = (left_avg_y + right_avg_y) / 2 - 0.1
            elif len(right_cones) < len(left_cones):
                # 右侧锥桶少，稍微偏右
                target_y = (left_avg_y + right_avg_y) / 2 + 0.1
            else:
                # 两侧锥桶数相等，走正中间
                target_y = (left_avg_y + right_avg_y) / 2
        elif len(left_cones) > 0:
            # 只有左侧有锥桶：向右避让
            target_y = right_avg_y - 0.2 if len(right_cones) > 0 else -0.2
        elif len(right_cones) > 0:
            # 只有右侧有锥桶：向左避让
            target_y = left_avg_y + 0.2 if len(left_cones) > 0 else 0.2
        else:
            # 没有锥桶，直行
            target_y = 0.0
        
        # 限制目标点Y坐标在合理范围内（避免过度偏移）
        target_y = max(-0.8, min(0.8, target_y))
        
        # 目标点X坐标：使用前方距离参数
        target_x = self.target_forward_distance
        
        # 如果前方有锥桶，稍微调整X坐标（避免太近）
        if len(avoidance_cones) > 0:
            min_x = min([x for x, y in avoidance_cones])
            if min_x < target_x:
                target_x = min_x - 0.2  # 保持0.2米安全距离
                target_x = max(0.5, target_x)  # 至少0.5米
        
        return target_x, target_y
    
    def select_target_point(self, center_points):
        """
        选择目标点（保留旧方法，用于兼容）
        直接选择最近的点（不使用jump）
        """
        if len(center_points) == 0:
            return -10000.0, -10000.0
        
        # 筛选前方点
        forward_points = [(x, y) for x, y in center_points if x >= 0]
        
        if len(forward_points) == 0:
            return -10000.0, -10000.0
        
        # 按距离排序，选择最近的点
        distances = [math.sqrt(x**2 + y**2) for x, y in forward_points]
        nearest_idx = np.argmin(distances)
        
        return forward_points[nearest_idx]
    
    def scan_callback(self, msg: LaserScan):
        """激光雷达回调函数"""
        self.frame_count += 1
        
        if len(msg.ranges) == 0:
            return
        
        # === 步骤1: 极坐标转笛卡尔坐标 ===
        ranges = np.array(msg.ranges)
        i = np.arange(len(ranges))
        angles = msg.angle_min + i * msg.angle_increment
        
        # 距离滤波
        valid_i = (ranges > self.min_range) & (ranges < self.max_range)
        valid_angles = angles[valid_i]
        valid_ranges = ranges[valid_i]
        
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # 保存距离滤波后的点（用于可视化）
        self.filtered_x = x.tolist() if isinstance(x, np.ndarray) else x
        self.filtered_y = y.tolist() if isinstance(y, np.ndarray) else y
        
        # === 步骤2: 矩形滤波（左侧1.5m，右侧1.0m）===
        rect_i = (x > -self.back_length) & (x < self.forward_length) & \
                 (y > -self.lateral_range_right) & (y < self.lateral_range_left)
        rect_x = x[rect_i]
        rect_y = y[rect_i]
        
        # 保存矩形滤波后的点（用于可视化）
        self.rect_x = rect_x.tolist() if isinstance(rect_x, np.ndarray) else rect_x
        self.rect_y = rect_y.tolist() if isinstance(rect_y, np.ndarray) else rect_y
        
        if len(rect_x) < 3:
            self.target_x = -10000.0
            self.target_y = -10000.0
            self.publish_target()
            return
        
        # === 步骤3: 简单聚类（将小弧变成点）===
        cluster_centers = self.simple_density_cluster(rect_x, rect_y)
        
        # 保存聚类中心（用于可视化）
        self.cluster_centers = cluster_centers
        
        if len(cluster_centers) == 0:
            self.target_x = -10000.0
            self.target_y = -10000.0
            self.publish_target()
            return
        
        # === 步骤4: 提取锥桶位置（所有点都当作锥桶处理，挡板会被聚成多个点）===
        processed_cones = [(c[0], c[1]) for c in cluster_centers]
        
        # 如果没有锥桶，发布无效目标点
        if len(processed_cones) == 0:
            self.target_x = -10000.0
            self.target_y = -10000.0
            self.publish_target()
            return
        
        # 如果只有1个锥桶，使用避障逻辑生成目标点
        if len(processed_cones) == 1:
            self.target_x, self.target_y = self.select_target_point_avoidance(processed_cones)
            self.inner_points = []
            self.outer_points = []
            self.center_points = [(self.target_x, self.target_y)]
            self.publish_target()
            return
        
        # === 步骤6: 检测直角弯 ===
        self.is_right_angle_turn, self.right_angle_points = self.detect_right_angle_turn(processed_cones)
        
        if self.is_right_angle_turn:
            # 直角弯模式：基于左侧清晰道路
            self.get_logger().warn(">>> 检测到直角弯（左侧有清晰道路）！ <<<", throttle_duration_sec=1.0)
            
            # 左侧道路上的锥桶点（已按X坐标排序）
            road_cones = self.right_angle_points
            
            # 将道路锥桶分为内侧和外侧
            road_center_y = self.right_angle_road_center_y
            road_half_width = self.right_angle_left_road_width / 2
            
            inner_side_cones = [(x, y) for x, y in road_cones
                               if road_center_y - road_half_width < y < road_center_y + road_half_width]
            outer_side_cones = [(x, y) for x, y in road_cones
                               if y > road_center_y + road_half_width]
            
            if len(inner_side_cones) > 0 and len(outer_side_cones) > 0:
                # 计算道路中心路径：取内侧和外侧锥桶的中点
                # 找到最近的内侧和外侧锥桶
                inner_sorted = sorted(inner_side_cones, key=lambda p: p[0])
                outer_sorted = sorted(outer_side_cones, key=lambda p: p[0])
                
                # 选择最近的一对（内侧和外侧）
                if len(inner_sorted) > 0 and len(outer_sorted) > 0:
                    nearest_inner = inner_sorted[0]
                    # 找到与最近内侧锥桶X坐标最接近的外侧锥桶
                    nearest_outer = min(outer_sorted, key=lambda p: abs(p[0] - nearest_inner[0]))
                    
                    # 目标点：内侧和外侧锥桶的中点（道路中心）
                    self.target_x = (nearest_inner[0] + nearest_outer[0]) / 2
                    self.target_y = (nearest_inner[1] + nearest_outer[1]) / 2
                    
                    # 限制目标点位置
                    self.target_x = max(0.5, min(2.0, self.target_x))
                    self.target_y = max(0.3, min(1.2, self.target_y))
                    
                    # 保存用于可视化
                    self.inner_points = inner_sorted
                    self.outer_points = outer_sorted
                    self.center_points = [(self.target_x, self.target_y)]
                else:
                    # 如果无法配对，使用避障逻辑
                    self.target_x, self.target_y = self.select_target_point_avoidance(processed_cones)
                    self.inner_points = inner_side_cones
                    self.outer_points = outer_side_cones
                    self.center_points = [(self.target_x, self.target_y)]
            else:
                # 如果无法分离内外侧，使用避障逻辑
                self.target_x, self.target_y = self.select_target_point_avoidance(processed_cones)
                self.inner_points = road_cones
                self.outer_points = []
                self.center_points = [(self.target_x, self.target_y)]
        else:
            # === 普通模式：基于左右两侧锥桶的避障逻辑 ===
            # 使用新的避障方法，根据左右1米范围内的锥桶来决定目标点
            self.target_x, self.target_y = self.select_target_point_avoidance(processed_cones)
            
            # 保存用于可视化（显示左右两侧的锥桶）
            left_cones = [(x, y) for x, y in processed_cones 
                         if abs(y) < self.avoidance_lateral_range and x > 0 and x < self.avoidance_forward_range and y > 0]
            right_cones = [(x, y) for x, y in processed_cones 
                          if abs(y) < self.avoidance_lateral_range and x > 0 and x < self.avoidance_forward_range and y < 0]
            
            self.inner_points = left_cones
            self.outer_points = right_cones
            self.center_points = [(self.target_x, self.target_y)]
        
        # === 发布目标点 ===
        self.publish_target()
        
        # === 可视化 ===
        if self.enable_visualization and self.frame_count % self.update_interval == 0:
            self.update_visualization(processed_cones)
    
    def update_visualization(self, cones):
        """更新可视化"""
        if not self.show_window or self.ax is None:
            return
        
        try:
            self.ax.clear()
            self.ax.set_xlim(-2, 4)
            self.ax.set_ylim(-2, 2)
            self.ax.set_aspect('equal')
            
            # 绘制车辆位置
            self.ax.plot([0], [0], "p", color="green", markersize=15, label='Vehicle', zorder=10)
            
            # 绘制距离滤波后的点（蓝色，像 raw_lidar_viewer）
            if len(self.filtered_x) > 0:
                self.ax.scatter(self.filtered_x, self.filtered_y, 
                              s=5, alpha=0.6, c='blue', 
                              label=f'Distance Filtered ({len(self.filtered_x)})', zorder=1)
            
            # 绘制矩形滤波后的点（红色，重点显示，像 raw_lidar_viewer）
            if len(self.rect_x) > 0:
                self.ax.scatter(self.rect_x, self.rect_y, 
                              s=8, alpha=0.8, c='red', marker='o',
                              label=f'Rectangle Filtered ({len(self.rect_x)})', zorder=2)
            
            # 绘制聚类中心（蓝色星号，带点数标注，像 raw_lidar_viewer）
            if len(self.cluster_centers) > 0:
                cluster_x = [c[0] for c in self.cluster_centers]
                cluster_y = [c[1] for c in self.cluster_centers]
                self.ax.scatter(cluster_x, cluster_y, 
                              s=150, alpha=0.9, c='blue', marker='*',
                              edgecolors='darkblue', linewidths=2,
                              label=f'Cluster Centers ({len(self.cluster_centers)})', zorder=3)
                
                # 显示每个簇的点数（像 raw_lidar_viewer）
                for i, (cx, cy, count) in enumerate(self.cluster_centers):
                    self.ax.annotate(f'{count}', (cx, cy),
                                   xytext=(5, 5), textcoords='offset points',
                                   fontsize=8, color='blue', weight='bold',
                                   bbox=dict(boxstyle='round,pad=0.3', 
                                           facecolor='white', edgecolor='blue', alpha=0.7))
            
            # 绘制内道点连线（红色）
            if len(self.inner_points) > 1:
                inner_x = [p[0] for p in self.inner_points]
                inner_y = [p[1] for p in self.inner_points]
                self.ax.plot(inner_x, inner_y, 'r-', linewidth=2, alpha=0.7, 
                           label=f'Inner Path ({len(self.inner_points)} points)', zorder=4)
                self.ax.scatter(inner_x, inner_y, s=80, c='red', marker='s', 
                              edgecolors='darkred', linewidths=1.5, zorder=5)
            
            # 绘制外道点连线（黄色）
            if len(self.outer_points) > 1:
                outer_x = [p[0] for p in self.outer_points]
                outer_y = [p[1] for p in self.outer_points]
                self.ax.plot(outer_x, outer_y, 'y-', linewidth=2, alpha=0.7,
                           label=f'Outer Path ({len(self.outer_points)} points)', zorder=4)
                self.ax.scatter(outer_x, outer_y, s=80, c='yellow', marker='s',
                              edgecolors='orange', linewidths=1.5, zorder=5)
            
            # 绘制中心点连线（紫色）
            if len(self.center_points) > 1:
                center_x = [p[0] for p in self.center_points]
                center_y = [p[1] for p in self.center_points]
                self.ax.plot(center_x, center_y, 'm-', linewidth=3, alpha=0.8,
                           label=f'Center Path ({len(self.center_points)} points)', zorder=6)
                self.ax.scatter(center_x, center_y, s=100, c='magenta', marker='*',
                              edgecolors='purple', linewidths=2, zorder=7)
            elif len(self.center_points) == 1:
                # 单个中心点（可能是直角弯）
                center_x = [self.center_points[0][0]]
                center_y = [self.center_points[0][1]]
                self.ax.scatter(center_x, center_y, s=100, c='magenta', marker='*',
                              edgecolors='purple', linewidths=2, zorder=7,
                              label='Center Point')
            
            # 绘制直角弯（如果检测到）- 显示墙
            if self.is_right_angle_turn and len(self.right_angle_points) > 0:
                # 绘制墙上的点（连线显示墙的形状）
                wall_x = [p[0] for p in self.right_angle_points]
                wall_y = [p[1] for p in self.right_angle_points]
                self.ax.plot(wall_x, wall_y, 'g--', linewidth=3, alpha=0.6,
                           label=f'Right Angle Turn (Wall, {len(self.right_angle_points)} points)', zorder=6)
                self.ax.scatter(wall_x, wall_y,
                              s=150, c='green', marker='D',
                              edgecolors='darkgreen', linewidths=2, zorder=7,
                              label='Wall Points')
                
                # 标注墙的宽度和厚度
                if len(self.right_angle_points) >= 2:
                    x_coords = [p[0] for p in self.right_angle_points]
                    y_coords = [p[1] for p in self.right_angle_points]
                    wall_width = max(y_coords) - min(y_coords)
                    wall_thickness = max(x_coords) - min(x_coords)
                    wall_center_x = np.mean(x_coords)
                    wall_center_y = np.mean(y_coords)
                    self.ax.annotate(f'Wall\nWidth={wall_width:.2f}m\nThickness={wall_thickness:.2f}m',
                                   (wall_center_x, wall_center_y),
                                   xytext=(30, 30), textcoords='offset points',
                                   fontsize=9, color='green', weight='bold',
                                   bbox=dict(boxstyle='round,pad=0.5', facecolor='white', 
                                           edgecolor='green', alpha=0.8))
            
            # 绘制目标点（大号紫色星号）
            if self.target_x > -9000:
                self.ax.scatter([self.target_x], [self.target_y], s=300, c='purple', 
                              marker='*', edgecolors='darkviolet', linewidths=3,
                              label='Target Point', zorder=8)
                # 添加目标点标注
                self.ax.annotate(f'Target\n({self.target_x:.2f}, {self.target_y:.2f})',
                               (self.target_x, self.target_y),
                               xytext=(20, 20), textcoords='offset points',
                               fontsize=10, color='purple', weight='bold',
                               bbox=dict(boxstyle='round,pad=0.5', facecolor='white', 
                                       edgecolor='purple', alpha=0.8),
                               arrowprops=dict(arrowstyle='->', color='purple', lw=2))
            
            # 绘制检测区域框
            rect = Rectangle(
                (-self.back_length, -self.lateral_range_right),
                self.forward_length + self.back_length,
                self.lateral_range_left + self.lateral_range_right,
                linewidth=2, edgecolor='cyan', facecolor='none', linestyle='--',
                label=f'Detection Region (Left: {self.lateral_range_left}m, Right: {self.lateral_range_right}m)'
            )
            self.ax.add_patch(rect)
            
            # 添加坐标轴
            self.ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
            self.ax.axvline(x=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
            
            # 添加方向指示
            self.ax.arrow(0, 0, 0.3, 0, head_width=0.1, head_length=0.1,
                        fc='green', ec='green', linewidth=2, zorder=9)
            self.ax.text(0.4, 0.1, 'Forward', fontsize=10, color='green', weight='bold')
            
            # 图例和标题
            self.ax.legend(loc='upper right', fontsize=9)
            self.ax.grid(True, alpha=0.3)
            mode_text = "Right Angle Turn" if self.is_right_angle_turn else "Normal Path"
            self.ax.set_title(
                f'Simple Perception - Path Visualization | Frame: {self.frame_count} | Mode: {mode_text} | '
                f'Cones: {len(cones)}, Inner: {len(self.inner_points)}, Outer: {len(self.outer_points)}',
                fontsize=11, pad=10
            )
            self.ax.set_xlabel('X (m) - Forward', fontsize=10)
            self.ax.set_ylabel('Y (m) - Left', fontsize=10)
            
            # 强制刷新
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.01)
        
        except Exception as e:
            self.get_logger().error(f'可视化更新失败: {str(e)}')
    
    def publish_target(self):
        """发布目标点"""
        msg = Float32MultiArray()
        msg.data = [float(self.target_x), float(self.target_y)]
        self.target_pub.publish(msg)
        
        if self.target_x > -9000:
            self.get_logger().info(
                f"目标点: ({self.target_x:.2f}, {self.target_y:.2f}) | "
                f"内道: {len(self.inner_points)}, 外道: {len(self.outer_points)}",
                throttle_duration_sec=0.5
            )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = SimplePerceptionNode()
    
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

