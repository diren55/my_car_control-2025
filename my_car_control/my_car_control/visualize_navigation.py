#!/usr/bin/env python3
"""
导航路径可视化工具
实时显示：
1. Nav2规划的全局路径和局部路径
2. 记录的目标点（从CSV文件）
3. 当前机器人位置
4. 坐标系信息
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import csv
import os
import math

class NavigationVisualizer(Node):
    def __init__(self):
        super().__init__('navigation_visualizer')
        
        # 发布器
        self.recorded_points_pub = self.create_publisher(MarkerArray, '/navigation/recorded_points', 10)
        self.current_goal_pub = self.create_publisher(Marker, '/navigation/current_goal', 10)
        self.coordinate_info_pub = self.create_publisher(Marker, '/navigation/coordinate_info', 10)
        
        # 订阅器（尝试多个可能的话题名称）
        # 全局路径：可能是 /plan 或 /planning/global_path
        self.global_plan_sub = self.create_subscription(
            Path, '/plan', self.global_plan_callback, 10)
        # 局部路径：可能是 /local_plan 或 /controller_server/local_plan
        self.local_plan_sub = self.create_subscription(
            Path, '/local_plan', self.local_plan_callback, 10)
        # 当前位置：AMCL的定位结果
        self.current_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        # 目标点：可能是 /goal_pose 或从navigation_test_node发布
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # 加载记录的点
        self.recorded_points = []
        self.load_recorded_points()
        
        # 当前状态
        self.current_pose = None
        self.current_goal = None
        self.global_plan = None
        self.local_plan = None
        
        # 定时器：定期发布可视化标记
        self.timer = self.create_timer(0.5, self.publish_visualization)
        
        self.get_logger().info("导航可视化节点已启动")
        self.get_logger().info(f"已加载 {len(self.recorded_points)} 个记录的点")
    
    def load_recorded_points(self):
        """从CSV文件加载记录的点"""
        csv_path = os.path.expanduser('~/navigation_points.csv')
        if not os.path.exists(csv_path):
            self.get_logger().warn(f"CSV文件不存在: {csv_path}")
            return
        
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            self.recorded_points.append((x, y))
                        except ValueError:
                            continue
            self.get_logger().info(f"成功加载 {len(self.recorded_points)} 个点")
        except Exception as e:
            self.get_logger().error(f"加载CSV文件失败: {e}")
    
    def global_plan_callback(self, msg):
        """全局路径规划回调"""
        self.global_plan = msg
        if len(msg.poses) > 0:
            self.get_logger().info(
                f"收到全局路径: {len(msg.poses)} 个点, "
                f"起点: ({msg.poses[0].pose.position.x:.2f}, {msg.poses[0].pose.position.y:.2f}), "
                f"终点: ({msg.poses[-1].pose.position.x:.2f}, {msg.poses[-1].pose.position.y:.2f})"
            )
    
    def local_plan_callback(self, msg):
        """局部路径规划回调"""
        self.local_plan = msg
        if len(msg.poses) > 0:
            self.get_logger().info(
                f"收到局部路径: {len(msg.poses)} 个点, "
                f"起点: ({msg.poses[0].pose.position.x:.2f}, {msg.poses[0].pose.position.y:.2f})"
            )
    
    def pose_callback(self, msg):
        """当前位置回调"""
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """目标点回调"""
        self.current_goal = msg
    
    def publish_visualization(self):
        """发布可视化标记"""
        # 1. 发布记录的点
        self.publish_recorded_points()
        
        # 2. 发布当前目标点
        if self.current_goal:
            self.publish_current_goal()
        
        # 3. 发布坐标系信息
        self.publish_coordinate_info()
    
    def publish_recorded_points(self):
        """发布记录的点（绿色球体）"""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.recorded_points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # 绿色
            marker_array.markers.append(marker)
            
            # 添加文本标签
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = marker.header.stamp
            text_marker.id = i + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.15
            text_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            text_marker.text = f"{i+1}"
            marker_array.markers.append(text_marker)
        
        self.recorded_points_pub.publish(marker_array)
    
    def publish_current_goal(self):
        """发布当前目标点（红色箭头）"""
        marker = Marker()
        marker.header.frame_id = self.current_goal.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 9999
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = self.current_goal.pose
        marker.scale.x = 0.3
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 红色
        
        self.current_goal_pub.publish(marker)
    
    def publish_coordinate_info(self):
        """发布坐标系信息（文本）"""
        if not self.current_pose:
            return
        
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 20000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.current_pose.position.x
        marker.pose.position.y = self.current_pose.position.y
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.2
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 黄色
        
        # 计算yaw角度
        q = self.current_pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_deg = math.degrees(yaw)
        
        info_text = (
            f"当前位置 (map):\n"
            f"  x: {self.current_pose.position.x:.3f}\n"
            f"  y: {self.current_pose.position.y:.3f}\n"
            f"  yaw: {yaw_deg:.1f}°\n"
            f"记录点数: {len(self.recorded_points)}"
        )
        
        if self.current_goal:
            dx = self.current_goal.pose.position.x - self.current_pose.position.x
            dy = self.current_goal.pose.position.y - self.current_pose.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            info_text += f"\n目标距离: {dist:.3f}m"
        
        marker.text = info_text
        self.coordinate_info_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

