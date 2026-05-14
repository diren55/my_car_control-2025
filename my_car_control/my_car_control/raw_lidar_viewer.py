#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原始激光雷达点云可视化节点
- 只进行基本的坐标转换和矩形滤波
- 不进行KMeans、内外道分离等复杂处理
- 用于调试和观察原始数据
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os


class RawLidarViewer(Node):
    """原始激光雷达点云可视化节点"""
    
    def __init__(self):
        super().__init__('raw_lidar_viewer')
        
        # 声明参数（与perception_node保持一致，方便对比）
        self.declare_parameter('min_range', 0.2)          # 最小有效极径
        self.declare_parameter('max_range', 3.5)         # 最大有效极径
        self.declare_parameter('left_width', 2.5)        # 矩形滤波左侧宽度
        self.declare_parameter('right_width', 1.0)       # 矩形滤波右侧宽度
        self.declare_parameter('forward_length', 2.5)     # 矩形滤波前方长度
        self.declare_parameter('back_length', 0.5)        # 矩形滤波后方长度
        
        # 可视化参数
        self.declare_parameter('update_interval', 1)     # 更新间隔（帧数）
        self.declare_parameter('show_all_points', True)  # 是否显示所有原始点（距离滤波前）
        self.declare_parameter('show_filtered_points', True)  # 是否显示距离滤波后的点
        self.declare_parameter('show_rect_points', True)  # 是否显示矩形滤波后的点
        self.declare_parameter('enable_clustering', True)  # 是否启用聚类（将小弧变成点）
        self.declare_parameter('cluster_radius', 0.2)   # 聚类半径（米）
        self.declare_parameter('min_points_per_cluster', 10)  # 每个簇最少点数
        self.declare_parameter('enable_circle_tool', True)  # 是否启用圆圈工具
        
        # 获取参数
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.left_width = self.get_parameter('left_width').value
        self.right_width = self.get_parameter('right_width').value
        self.forward_length = self.get_parameter('forward_length').value
        self.back_length = self.get_parameter('back_length').value
        
        self.update_interval = self.get_parameter('update_interval').value
        self.show_all_points = self.get_parameter('show_all_points').value
        self.show_filtered_points = self.get_parameter('show_filtered_points').value
        self.show_rect_points = self.get_parameter('show_rect_points').value
        self.enable_clustering = self.get_parameter('enable_clustering').value
        self.cluster_radius = self.get_parameter('cluster_radius').value
        self.min_points_per_cluster = self.get_parameter('min_points_per_cluster').value
        self.enable_circle_tool = self.get_parameter('enable_circle_tool').value
        
        # 状态变量
        self.frame_count = 0
        self.rect_x = []  # 保存当前帧的点，用于圆圈工具
        self.rect_y = []
        self.circle_center = None  # 圆圈中心
        self.circle_radius = 0.0   # 圆圈半径
        self.circle_patch = None    # 圆圈图形对象
        self.drawing_circle = False  # 是否正在画圆圈
        
        # 初始化matplotlib
        try:
            import matplotlib
            if 'DISPLAY' not in os.environ or not os.environ.get('DISPLAY'):
                self.get_logger().warn(">>> DISPLAY环境变量未设置，无法显示弹窗。请设置DISPLAY环境变量（如：export DISPLAY=:0）<<<")
                self.show_window = False
            else:
                matplotlib.use('TkAgg')
                plt.ion()
                self.fig, self.ax = plt.subplots(figsize=(12, 12))
                self.fig.canvas.manager.set_window_title('Raw Lidar Point Cloud Viewer')
                
                # 绑定鼠标事件（圆圈工具）
                if self.enable_circle_tool:
                    self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
                    self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
                    self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
                    self.get_logger().info(">>> 圆圈工具已启用：点击并拖动鼠标画圆圈 <<<")
                
                plt.show(block=False)
                plt.pause(0.1)
                self.show_window = True
                self.get_logger().info(">>> Matplotlib弹窗初始化成功 <<<")
        except Exception as e:
            self.get_logger().error(f">>> Matplotlib弹窗初始化失败: {str(e)} <<<")
            self.show_window = False
        
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
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("原始激光雷达点云可视化节点已启动")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"距离滤波: [{self.min_range}, {self.max_range}]m")
        self.get_logger().info(f"矩形滤波: 左{self.left_width}m, 右{self.right_width}m, 前{self.forward_length}m, 后{self.back_length}m")
        self.get_logger().info(f"更新间隔: 每{self.update_interval}帧更新一次")
        if self.enable_clustering:
            self.get_logger().info(f"聚类: 启用, 半径={self.cluster_radius}m, 最少点数={self.min_points_per_cluster}")
        if self.enable_circle_tool:
            self.get_logger().info("圆圈工具: 启用（点击并拖动鼠标画圆圈，查看半径和点数）")
        self.get_logger().info("=" * 60)
    
    def scan_callback(self, msg: LaserScan):
        """激光雷达回调函数"""
        self.frame_count += 1
        
        # 只在指定间隔更新显示
        if self.frame_count % self.update_interval != 0:
            return
        
        if not self.show_window:
            return
        
        ranges = np.array(msg.ranges)
        num_ranges = len(ranges)
        
        if num_ranges == 0:
            self.get_logger().warn("激光雷达数据为空")
            return
        
        # === 步骤1: 极坐标转笛卡尔坐标（所有点）===
        decimal = 2  # 保留小数位
        i = np.arange(len(ranges))
        angles = msg.angle_min + i * msg.angle_increment
        
        # 所有原始点（包括无效点）
        all_x = np.round(np.round(ranges, decimal) * np.cos(angles), decimal)
        all_y = np.round(np.round(ranges, decimal) * np.sin(angles), decimal)
        
        # 有效点（距离范围内）
        valid_i = (ranges > self.min_range) & (ranges < self.max_range)
        valid_angles = angles[valid_i]
        valid_ranges = ranges[valid_i]
        
        filtered_x = np.round(np.round(valid_ranges, decimal) * np.cos(valid_angles), decimal)
        filtered_y = np.round(np.round(valid_ranges, decimal) * np.sin(valid_angles), decimal)
        
        # === 步骤2: 矩形滤波 ===
        rect_i = (filtered_x > -self.back_length) & (filtered_x < self.forward_length) & \
                 (filtered_y < self.left_width) & (filtered_y > -self.right_width)
        rect_x = filtered_x[rect_i]
        rect_y = filtered_y[rect_i]
        
        # 统计信息
        total_points = len(ranges)
        valid_points = len(filtered_x)
        rect_points = len(rect_x)
        
        # 保存当前点用于圆圈工具
        self.rect_x = rect_x.tolist() if isinstance(rect_x, np.ndarray) else rect_x
        self.rect_y = rect_y.tolist() if isinstance(rect_y, np.ndarray) else rect_y
        
        # === 步骤3: 聚类（将小弧变成点）===
        cluster_centers = []
        if self.enable_clustering and len(rect_x) > 0:
            cluster_centers = self.simple_cluster(rect_x, rect_y)
        
        # 更新显示
        self.update_plot(all_x, all_y, filtered_x, filtered_y, rect_x, rect_y, 
                        total_points, valid_points, rect_points, cluster_centers)
    
    def euclidean_dist(self, p1, p2):
        """计算两点间欧氏距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def simple_cluster(self, x_list, y_list):
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
    
    def count_points_in_circle(self, center_x, center_y, radius):
        """统计圆圈内的点数"""
        if len(self.rect_x) == 0:
            return 0
        
        count = 0
        for x, y in zip(self.rect_x, self.rect_y):
            dist = math.sqrt((x - center_x)**2 + (y - center_y)**2)
            if dist <= radius:
                count += 1
        return count
    
    def on_mouse_press(self, event):
        """鼠标按下事件"""
        if event.inaxes != self.ax or event.button != 1:  # 只响应左键
            return
        
        if self.enable_circle_tool:
            self.circle_center = (event.xdata, event.ydata)
            self.drawing_circle = True
            self.circle_radius = 0.0
    
    def on_mouse_move(self, event):
        """鼠标移动事件"""
        if not self.drawing_circle or event.inaxes != self.ax:
            return
        
        if self.circle_center is not None:
            # 计算当前半径
            dx = event.xdata - self.circle_center[0]
            dy = event.ydata - self.circle_center[1]
            self.circle_radius = math.sqrt(dx**2 + dy**2)
            
            # 更新圆圈显示
            if self.circle_patch is not None:
                self.circle_patch.remove()
            
            circle = plt.Circle(self.circle_center, self.circle_radius, 
                             fill=False, edgecolor='orange', linewidth=2, linestyle='--')
            self.circle_patch = self.ax.add_patch(circle)
            
            # 统计圆圈内的点数
            point_count = self.count_points_in_circle(self.circle_center[0], self.circle_center[1], self.circle_radius)
            
            # 更新标题显示信息
            title_text = (
                f'Raw Lidar Point Cloud | Frame: {self.frame_count} | '
                f'Total: {len(self.rect_x)}, Valid: {len(self.rect_x)}, Rect: {len(self.rect_x)} | '
                f'Circle: R={self.circle_radius:.3f}m, Points={point_count}'
            )
            self.ax.set_title(title_text, fontsize=11, pad=10)
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
    
    def on_mouse_release(self, event):
        """鼠标释放事件"""
        if event.button != 1:
            return
        
        if self.drawing_circle and self.circle_center is not None:
            point_count = self.count_points_in_circle(self.circle_center[0], self.circle_center[1], self.circle_radius)
            self.get_logger().info(
                f"圆圈: 中心=({self.circle_center[0]:.3f}, {self.circle_center[1]:.3f}), "
                f"半径={self.circle_radius:.3f}m, 包含点数={point_count}"
            )
            self.drawing_circle = False
    
    def update_plot(self, all_x, all_y, filtered_x, filtered_y, rect_x, rect_y,
                   total_points, valid_points, rect_points, cluster_centers):
        """更新绘图"""
        try:
            # 保存圆圈工具的状态
            saved_circle = None
            if self.circle_patch is not None and not self.drawing_circle:
                saved_circle = (self.circle_center, self.circle_radius)
            
            self.ax.clear()
            self.circle_patch = None  # 清除旧的圆圈
            
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)
            self.ax.set_aspect('equal')
            
            # 恢复圆圈（如果存在且不在绘制中）
            if saved_circle is not None:
                center, radius = saved_circle
                circle = plt.Circle(center, radius, 
                                 fill=False, edgecolor='orange', linewidth=2, linestyle='--')
                self.circle_patch = self.ax.add_patch(circle)
                point_count = self.count_points_in_circle(center[0], center[1], radius)
                # 在标题中显示圆圈信息
                title_suffix = f' | Circle: R={radius:.3f}m, Points={point_count}'
            else:
                title_suffix = ''
            
            # 绘制车辆位置
            self.ax.plot([0], [0], "p", color="green", markersize=15, label='Vehicle', zorder=10)
            
            # 绘制矩形滤波区域
            rect = Rectangle(
                (-self.back_length, -self.right_width),
                self.forward_length + self.back_length,
                self.left_width + self.right_width,
                linewidth=2, edgecolor='cyan', facecolor='none', linestyle='--',
                label=f'Filter Region (X:[-{self.back_length:.1f}, {self.forward_length:.1f}], Y:[-{self.right_width:.1f}, {self.left_width:.1f}])'
            )
            self.ax.add_patch(rect)
            
            # 绘制距离滤波范围（圆形）
            circle_all = plt.Circle((0, 0), self.max_range, fill=False, 
                                   edgecolor='gray', linestyle=':', linewidth=1, 
                                   label=f'Max Range ({self.max_range}m)')
            circle_min = plt.Circle((0, 0), self.min_range, fill=False, 
                                   edgecolor='gray', linestyle=':', linewidth=1,
                                   label=f'Min Range ({self.min_range}m)')
            self.ax.add_patch(circle_all)
            self.ax.add_patch(circle_min)
            
            # 绘制所有原始点（灰色，小点）
            if self.show_all_points:
                # 只显示有效距离的点（避免显示无穷大或NaN）
                valid_all = np.isfinite(all_x) & np.isfinite(all_y) & \
                           (np.sqrt(all_x**2 + all_y**2) > 0.1) & \
                           (np.sqrt(all_x**2 + all_y**2) < 10.0)
                self.ax.scatter(all_x[valid_all], all_y[valid_all], 
                              s=3, alpha=0.3, c='gray', label=f'All Points ({total_points})', zorder=1)
            
            # 绘制距离滤波后的点（蓝色）
            if self.show_filtered_points and len(filtered_x) > 0:
                self.ax.scatter(filtered_x, filtered_y, 
                              s=5, alpha=0.6, c='blue', 
                              label=f'Distance Filtered ({valid_points})', zorder=2)
            
            # 绘制矩形滤波后的点（红色，重点显示）
            if self.show_rect_points and len(rect_x) > 0:
                self.ax.scatter(rect_x, rect_y, 
                              s=8, alpha=0.8, c='red', marker='o',
                              label=f'Rectangle Filtered ({rect_points})', zorder=3)
            
            # 绘制聚类中心（如果启用聚类）
            if self.enable_clustering and len(cluster_centers) > 0:
                cluster_x = [c[0] for c in cluster_centers]
                cluster_y = [c[1] for c in cluster_centers]
                self.ax.scatter(cluster_x, cluster_y, 
                              s=150, alpha=0.9, c='blue', marker='*',
                              edgecolors='darkblue', linewidths=2,
                              label=f'Cluster Centers ({len(cluster_centers)})', zorder=4)
                
                # 显示每个簇的点数
                for i, (cx, cy, count) in enumerate(cluster_centers):
                    self.ax.annotate(f'{count}', (cx, cy),
                                   xytext=(5, 5), textcoords='offset points',
                                   fontsize=8, color='blue', weight='bold',
                                   bbox=dict(boxstyle='round,pad=0.3', 
                                           facecolor='white', edgecolor='blue', alpha=0.7))
            
            # 添加坐标轴
            self.ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
            self.ax.axvline(x=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
            
            # 添加方向指示
            self.ax.arrow(0, 0, 0.5, 0, head_width=0.1, head_length=0.1, 
                        fc='green', ec='green', linewidth=2, zorder=5)
            self.ax.text(0.6, 0.1, 'Forward (X+)', fontsize=10, color='green', weight='bold')
            
            # 添加图例和标题
            self.ax.legend(loc='upper right', fontsize=9)
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title(
                f'Raw Lidar Point Cloud | Frame: {self.frame_count} | '
                f'Total: {total_points}, Valid: {valid_points}, Rect: {rect_points}{title_suffix}',
                fontsize=11, pad=10
            )
            self.ax.set_xlabel('X (m) - Forward', fontsize=10)
            self.ax.set_ylabel('Y (m) - Left', fontsize=10)
            
            # 强制刷新显示
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.01)
            
            # 打印统计信息（每10帧打印一次）
            if self.frame_count % (10 * self.update_interval) == 0:
                self.get_logger().info(
                    f"帧{self.frame_count}: 总点数={total_points}, "
                    f"距离滤波后={valid_points}, 矩形滤波后={rect_points}"
                )
        
        except Exception as e:
            self.get_logger().error(f'绘图失败: {str(e)}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = RawLidarViewer()
    
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

