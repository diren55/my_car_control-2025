#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航测试节点 - 用于单独调试导航功能
功能：
1. 手动启动/停止记录路径点
2. 查看已记录的路径点数量
3. 保存路径点到CSV
4. 从CSV加载路径点
5. 手动发送目标点
6. 自动循环发送目标点（第二圈导航模式）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
import sys
import os
import time
import math
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # 必须导入以支持 PoseStamped 的转换

from my_car_control.navigation_manager import NavigationManager


class NavigationTestNode(Node):
    """导航测试节点"""
    
    def __init__(self):
        super().__init__('navigation_test_node')
        
        # 参数
        self.declare_parameter('csv_path', os.path.expanduser('~/navigation_points.csv'))
        self.declare_parameter('record_interval', 0.5)
        self.declare_parameter('min_distance', 0.8)
        
        csv_path = self.get_parameter('csv_path').value
        record_interval = self.get_parameter('record_interval').value
        min_distance = self.get_parameter('min_distance').value
        
        # 初始化导航管理器
        self.nav_manager = NavigationManager(
            csv_file_path=csv_path,
            logger=self.get_logger(),
            record_interval=record_interval,
            min_distance=min_distance
        )
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_combined',
            self.odom_callback,
            qos_profile=qos_profile
        )
        
        # TF2 缓冲区和监听器（用于坐标转换）
        # 设置缓存时间为 10 秒，避免时间戳问题
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 发布初始位置（用于自动设置初始位置）
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # 发布目标点（保留用于兼容性，但主要使用Action）
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Nav2 Action 客户端
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_goal_handle = None
        self.action_server_ready = False
        
        # 创建定时器检查Action服务器
        self.action_check_timer = self.create_timer(2.0, self.check_action_server)
        
        # 状态发布（用于显示当前状态）
        self.status_pub = self.create_publisher(
            String,
            '/navigation_test_status',
            10
        )
        
        # 订阅命令话题（用于接收命令，替代input）
        self.command_sub = self.create_subscription(
            String,
            '/navigation_test_command',
            self.command_callback,
            10
        )
        
        # 创建服务用于接收命令（更可靠）
        self.command_service = self.create_service(
            Trigger,
            '/navigation_test_execute',
            self.execute_command_service
        )
        
        # 定时器：自动循环发送目标点（第二圈导航模式）
        self.auto_nav_timer = None
        self.auto_nav_enabled = False
        
        # 定时器：第一圈记录路径点
        self.record_timer = None
        
        # 当前里程计数据
        self.current_odom = None
        
        # 初始位置设置标志
        self.initial_pose_set = False
        
        # 打印使用说明
        self.print_usage()
        
        # 等待一段时间后自动设置初始位置（只执行一次）
        self.initial_pose_timer = self.create_timer(2.0, self.auto_set_initial_pose_once)
        
        # 创建定时器用于定期打印状态
        self.status_timer = self.create_timer(2.0, self.print_status)
    
    def print_usage(self):
        """打印使用说明"""
        usage = """
╔══════════════════════════════════════════════════════════════╗
║           导航测试节点 - 使用说明                            ║
╠══════════════════════════════════════════════════════════════╣
║ 命令（输入单个字母或数字即可）：                              ║
║                                                              ║
║  1 或 s  - 开始记录路径点（第一圈）                           ║
║  2 或 t  - 停止记录路径点                                     ║
║  3 或 v  - 保存路径点到CSV文件                               ║
║  4 或 l  - 从CSV文件加载路径点                                ║
║  5 或 g  - 发送当前目标点（手动）                            ║
║  6 或 n  - 移动到下一个目标点                                 ║
║  7 或 a  - 启动自动循环导航（第二圈模式）                    ║
║  8 或 o  - 停止自动循环导航                                  ║
║  9 或 i  - 显示当前状态                                      ║
║  0 或 p  - 显示已记录的路径点数量                             ║
║  c       - 清空已记录的路径点（不保存）                       ║
║  h       - 显示此帮助信息                                    ║
║  q       - 退出程序                                          ║
╚══════════════════════════════════════════════════════════════╝
        """
        print(usage, flush=True)
        self.get_logger().info(usage)
    
    def auto_set_initial_pose_once(self):
        """自动设置初始位置（在节点启动后调用，只执行一次）"""
        if self.initial_pose_set:
            self.initial_pose_timer.cancel()
            return
        
        if self.current_odom is None:
            return
        
        # 执行设置
        self.auto_set_initial_pose()
        
        # 取消定时器（只执行一次）
        if self.initial_pose_set:
            self.initial_pose_timer.cancel()
    
    def auto_set_initial_pose(self):
        """设置初始位置"""
        if self.initial_pose_set:
            return
        
        if self.current_odom is None:
            return
        
        # 检查 map 坐标系是否存在
        map_exists = False
        try:
            # 尝试获取 map -> odom_combined 的变换
            self.tf_buffer.lookup_transform('map', 'odom_combined', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            map_exists = True
        except TransformException:
            map_exists = False
        
        # 创建初始位置消息
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        if map_exists:
            # 如果 map 坐标系已存在，使用当前里程计位置转换为 map 坐标
            try:
                pose_odom = PoseStamped()
                pose_odom.header.frame_id = 'odom_combined'
                pose_odom.header.stamp = rclpy.time.Time().to_msg()  # 使用最新时间
                pose_odom.pose = self.current_odom.pose.pose
                
                pose_map = self.tf_buffer.transform(pose_odom, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
                initial_pose.header.frame_id = 'map'
                initial_pose.pose.pose = pose_map.pose
                self.get_logger().info("map 坐标系已存在，使用当前位置设置初始位置")
            except TransformException as e:
                # 转换失败，使用odom_combined坐标作为初始位置
                self.get_logger().warn(f"无法转换到 map 坐标系: {e}，使用odom_combined坐标作为初始位置")
                initial_pose.header.frame_id = 'map'
                initial_pose.pose.pose.position.x = self.current_odom.pose.pose.position.x
                initial_pose.pose.pose.position.y = self.current_odom.pose.pose.position.y
                initial_pose.pose.pose.position.z = 0.0
                initial_pose.pose.pose.orientation = self.current_odom.pose.pose.orientation
        else:
            # 如果 map 坐标系不存在，设置初始位置为 (0, 0, 0)
            # 这样 gmapping 会以当前位置为原点创建 map 坐标系
            initial_pose.header.frame_id = 'map'
            initial_pose.pose.pose.position.x = 0.0
            initial_pose.pose.pose.position.y = 0.0
            initial_pose.pose.pose.position.z = 0.0
            # 使用当前里程计的姿态
            initial_pose.pose.pose.orientation = self.current_odom.pose.pose.orientation
            self.get_logger().info("map 坐标系不存在，设置初始位置为 (0, 0, 0)，gmapping 将以此创建 map 坐标系")
        
        # 设置协方差（位置不确定性）
        # 对角线元素：x, y, z, roll, pitch, yaw 的方差
        covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
                      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # y
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # z
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # roll
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # pitch
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]  # yaw (约 15 度的方差)
        initial_pose.pose.covariance = covariance
        
        # 发布初始位置
        self.initial_pose_pub.publish(initial_pose)
        self.initial_pose_set = True
        
        x = initial_pose.pose.pose.position.x
        y = initial_pose.pose.pose.position.y
        self.get_logger().info(
            f"✅ 已自动设置初始位置: ({x:.3f}, {y:.3f}), frame_id: {initial_pose.header.frame_id}"
        )
    
    def odom_callback(self, msg: Odometry):
        """里程计回调"""
        self.current_odom = msg
        
        # 更新导航管理器的姿态信息
        orientation = msg.pose.pose.orientation
        self.nav_manager.update_orientation(orientation.z, orientation.w)
        
        # 如果启用自动导航，检查是否到达目标点（在里程计回调中检查，与源程序保持一致）
        if self.auto_nav_enabled and self.nav_manager.is_navigating:
            # 尝试获取map坐标系中的当前位置
            current_x_map = None
            current_y_map = None
            x_odom = msg.pose.pose.position.x
            y_odom = msg.pose.pose.position.y
            
            # 先检查map坐标系是否存在
            map_exists = False
            try:
                # 检查map坐标系是否存在（使用非常短的超时）
                self.tf_buffer.can_transform('map', 'odom_combined', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
                map_exists = True
            except:
                map_exists = False
            
            if map_exists:
                try:
                    # 创建odom_combined坐标系中的pose
                    pose_odom = PoseStamped()
                    pose_odom.header.frame_id = 'odom_combined'
                    pose_odom.header.stamp = rclpy.time.Time().to_msg()  # 使用最新时间
                    pose_odom.pose = msg.pose.pose
                    
                    # 转换到map坐标系（增加超时时间到0.5秒）
                    pose_map = self.tf_buffer.transform(
                        pose_odom, 'map',
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    current_x_map = pose_map.pose.position.x
                    current_y_map = pose_map.pose.position.y
                    # 清除警告标志（如果转换成功）
                    if hasattr(self, '_tf_warned'):
                        delattr(self, '_tf_warned')
                except Exception as e:
                    # 如果转换失败，使用odom_combined坐标，但给出警告（只警告一次）
                    if not hasattr(self, '_tf_warned'):
                        self.get_logger().warn(f"坐标转换失败: {e}，使用odom_combined坐标")
                        self._tf_warned = True
                    current_x_map = x_odom
                    current_y_map = y_odom
            else:
                # map坐标系不存在，使用odom_combined坐标（只警告一次）
                if not hasattr(self, '_tf_warned'):
                    self.get_logger().warn("map坐标系不存在，使用odom_combined坐标。请确保gmapping已启动并已设置初始位置")
                    self._tf_warned = True
                current_x_map = x_odom
                current_y_map = y_odom
            
            # 检查是否到达当前目标点
            # 添加调试日志（每5次打印一次，更频繁以便调试）
            if not hasattr(self, '_goal_check_counter'):
                self._goal_check_counter = 0
            self._goal_check_counter += 1
            
            # 每次打印位置更新信息（用于调试位置是否更新）
            goal_idx = self.nav_manager.current_goal_index - 1 if self.nav_manager.current_goal_index > 0 else 0
            if goal_idx < len(self.nav_manager.loaded_points):
                check_goal = self.nav_manager.loaded_points[goal_idx]
                check_dist = math.sqrt((current_x_map - check_goal[0])**2 + (current_y_map - check_goal[1])**2)
                
                # 每5次打印一次详细日志
                if self._goal_check_counter % 5 == 0:
                    self.get_logger().info(
                        f"【位置检查】目标点 {goal_idx+1}: ({check_goal[0]:.3f}, {check_goal[1]:.3f}), "
                        f"当前位置(map): ({current_x_map:.3f}, {current_y_map:.3f}), "
                        f"距离: {check_dist:.3f} 米, "
                        f"odom: ({x_odom:.3f}, {y_odom:.3f})"
                    )
            
            if self.nav_manager.check_goal_reached(current_x_map, current_y_map):
                # 获取当前检查的目标点索引（goalId-1）
                # 注意：此时current_goal_index已经是下一个要发送的目标点索引
                # 所以检查的是 current_goal_index - 1，也就是刚发送的目标点
                goal_idx = self.nav_manager.current_goal_index - 1 if self.nav_manager.current_goal_index > 0 else 0
                idx = goal_idx + 1  # 显示时加1（从1开始）
                print(f"\n✅ 到达目标点 {idx}，移动到下一个")
                self.get_logger().info(
                    f"到达目标点 {idx}（索引 {goal_idx}），移动到下一个。"
                    f"当前位置(map): ({current_x_map:.3f}, {current_y_map:.3f})"
                )
                # 注意：不需要再调用move_to_next_goal，因为send_current_goal中已经更新了
                # 只需要发送下一个目标点
                self.send_current_goal()
                print("\n导航测试> ", end='', flush=True)
    
    def record_navigation_point(self):
        """定时器回调：记录当前路径点（第一圈）"""
        if not self.nav_manager.is_recording:
            return
        
        if self.current_odom is None:
            return  # 静默返回，避免日志影响实时性
        
        # 获取当前位置（odom_combined 坐标系）
        x_odom = self.current_odom.pose.pose.position.x
        y_odom = self.current_odom.pose.pose.position.y
        
        # 快速检查 map 坐标系是否存在（使用非常短的超时，避免阻塞）
        map_exists = False
        try:
            self.tf_buffer.lookup_transform(
                'map', 'odom_combined', 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.01)  # 非常短的超时
            )
            map_exists = True
        except:
            map_exists = False
        
        # 如果 map 坐标系存在，尝试快速转换（使用短超时避免阻塞）
        if map_exists:
            try:
                pose_odom = PoseStamped()
                pose_odom.header.frame_id = 'odom_combined'
                pose_odom.header.stamp = rclpy.time.Time().to_msg()
                pose_odom.pose.position.x = x_odom
                pose_odom.pose.position.y = y_odom
                pose_odom.pose.position.z = 0.0
                pose_odom.pose.orientation = self.current_odom.pose.pose.orientation
                
                # 使用非常短的超时，避免阻塞主线程
                pose_map = self.tf_buffer.transform(
                    pose_odom, 'map', 
                    timeout=rclpy.duration.Duration(seconds=0.05)  # 50ms超时
                )
                x = pose_map.pose.position.x
                y = pose_map.pose.position.y
                orientation = pose_map.pose.orientation
                
                self.nav_manager.update_orientation(orientation.z, orientation.w)
                self.nav_manager.record_point(x, y)
            except:
                # 转换失败，快速回退到 odom_combined 坐标（不打印日志，避免影响实时性）
                self.nav_manager.record_point(x_odom, y_odom)
        else:
            # map 坐标系不存在，直接使用 odom_combined 坐标
            self.nav_manager.record_point(x_odom, y_odom)
        
        # 每记录20个点打印一次日志（减少日志频率）
        count = self.nav_manager.get_points_count()
        if count % 20 == 0:
            coord_system = "map" if map_exists else "odom_combined"
            self.get_logger().info(
                f"已记录 {count} 个路径点（{coord_system}）"
            )
    
    def send_current_goal(self, skip_count=0):
        """
        发送当前目标点（使用Nav2 Action）
        
        Args:
            skip_count: 已跳过的目标点数量（防止无限递归）
        """
        import sys
        import math
        
        # 防止无限递归（如果连续跳过太多点，停止）
        if skip_count > 10:
            self.get_logger().error("连续跳过太多目标点（>10），停止导航")
            print("⚠️  连续跳过太多目标点，停止导航", flush=True)
            return False
        
        goal_pose = self.nav_manager.get_current_goal(clock=self.get_clock())
        if not goal_pose:
            print("⚠️  没有可用的目标点", flush=True)
            sys.stdout.flush()
            self.get_logger().warn("没有可用的目标点")
            return False
        
        # 检查目标点距离和有效性
        # 注意：goal_pose的frame_id是'map'，需要将当前位置也转换到map坐标系
        if self.current_odom:
            # 尝试获取map坐标系中的当前位置
            current_x_map = None
            current_y_map = None
            # 先检查map坐标系是否存在
            map_exists = False
            try:
                self.tf_buffer.can_transform('map', 'odom_combined', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))
                map_exists = True
            except:
                map_exists = False
            
            if map_exists:
                try:
                    # 创建odom_combined坐标系中的pose
                    pose_odom = PoseStamped()
                    pose_odom.header.frame_id = 'odom_combined'
                    pose_odom.header.stamp = rclpy.time.Time().to_msg()
                    pose_odom.pose = self.current_odom.pose.pose
                    
                    # 转换到map坐标系（增加超时时间到0.5秒）
                    pose_map = self.tf_buffer.transform(
                        pose_odom, 'map',
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    current_x_map = pose_map.pose.position.x
                    current_y_map = pose_map.pose.position.y
                except Exception as e:
                    # 如果转换失败，使用odom_combined坐标（但会警告，只警告一次）
                    if not hasattr(self, '_send_goal_tf_warned'):
                        self.get_logger().warn(f"无法转换当前位置到map坐标系: {e}，使用odom_combined坐标")
                        self._send_goal_tf_warned = True
                    current_x_map = self.current_odom.pose.pose.position.x
                    current_y_map = self.current_odom.pose.pose.position.y
            else:
                # map坐标系不存在，使用odom_combined坐标（只警告一次）
                if not hasattr(self, '_send_goal_tf_warned'):
                    self.get_logger().warn("map坐标系不存在，使用odom_combined坐标。请确保gmapping已启动并已设置初始位置")
                    self._send_goal_tf_warned = True
                current_x_map = self.current_odom.pose.pose.position.x
                current_y_map = self.current_odom.pose.pose.position.y
            
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            distance = math.sqrt((goal_x - current_x_map)**2 + (goal_y - current_y_map)**2)
            
            self.get_logger().info(
                f"发送目标点: ({goal_x:.3f}, {goal_y:.3f}), "
                f"当前位置(map): ({current_x_map:.3f}, {current_y_map:.3f}), "
                f"距离: {distance:.3f} 米"
            )
            
            # 源程序使用 MIN_DISTANCE = 1.5 米来判断到达
            # 如果距离小于 0.3 米，Nav2可能无法规划路径（状态码6），自动跳过
            if distance < 0.3:
                idx = self.nav_manager.get_current_goal_index() + 1
                self.get_logger().warn(
                    f"⚠️  目标点 {idx} 距离当前位置太近 ({distance:.3f} 米 < 0.3 米)，"
                    f"Nav2无法规划路径，自动跳过此点"
                )
                print(f"⚠️  跳过目标点 {idx}（距离太近: {distance:.3f} 米）", flush=True)
                # 自动移动到下一个目标点
                self.nav_manager.move_to_next_goal()
                # 递归调用，发送下一个目标点（增加skip_count）
                return self.send_current_goal(skip_count=skip_count + 1)
        
        # 检查Action服务器是否就绪（使用缓存的状态）
        if not self.action_server_ready:
            if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn("Nav2 Action服务器未就绪，使用话题方式（桥接节点会处理）")
                # 发布到话题，桥接节点会处理
                self.goal_pub.publish(goal_pose)
                idx = self.nav_manager.get_current_goal_index() + 1
                total = self.nav_manager.get_loaded_points_count()
                x = goal_pose.pose.position.x
                y = goal_pose.pose.position.y
                print(f"📤 已通过话题发布目标点 {idx}/{total}: ({x:.3f}, {y:.3f})", flush=True)
                self.get_logger().info(f"已通过话题发布目标点 {idx}/{total}: ({x:.3f}, {y:.3f})")
                return True
            else:
                self.action_server_ready = True
        
        # 取消之前的goal（如果有）
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal()
        
        # 创建Action目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # 记录当前发送的目标点索引（发送前）
        sent_goal_idx = self.nav_manager.get_current_goal_index()
        
        # 发送Action目标
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # 发送目标点后，立即更新索引（与源程序保持一致：发送后goalId加1）
        # 这样检查时才能检查到刚发送的目标点
        self.nav_manager.move_to_next_goal()
        
        idx = sent_goal_idx + 1
        total = self.nav_manager.get_loaded_points_count()
        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y
        print(f"📤 已发送导航目标 {idx}/{total}: ({x:.3f}, {y:.3f})", flush=True)
        sys.stdout.flush()
        self.get_logger().info(f"已发送导航目标 {idx}/{total}: ({x:.3f}, {y:.3f})，current_goal_index已更新为: {self.nav_manager.get_current_goal_index()}")
        
        # 同时发布到话题（用于兼容性）
        self.goal_pub.publish(goal_pose)
        
        return True
    
    def goal_response_callback(self, future):
        """Action目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("导航目标被拒绝")
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info("导航目标已接受，开始导航")
        
        # 获取结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Action目标结果回调"""
        import math
        try:
            response = future.result()
            status = response.status
            result = response.result
            
            # 检查导航状态
            status_names = {
                1: "ACCEPTED",
                2: "CANCELED", 
                3: "ABORTED",
                4: "SUCCEEDED",
                5: "REJECTED",
                6: "UNKNOWN"
            }
            status_name = status_names.get(status, f"UNKNOWN({status})")
            
            # 注意：源程序不依赖Nav2的状态码，而是直接检查距离
            # 状态码6（UNKNOWN）通常表示Nav2无法规划路径，但不一定意味着失败
            # 我们仍然依赖 odom_callback 中的距离检查来判断是否到达目标点
            
            # 只在状态码为4（SUCCEEDED）时记录日志
            if status == 4:
                if self.current_odom and self.nav_manager.is_navigating:
                    # 尝试获取map坐标系中的当前位置
                    current_x_map = None
                    current_y_map = None
                    try:
                        pose_odom = PoseStamped()
                        pose_odom.header.frame_id = 'odom_combined'
                        pose_odom.header.stamp = rclpy.time.Time().to_msg()
                        pose_odom.pose = self.current_odom.pose.pose
                        
                        pose_map = self.tf_buffer.transform(
                            pose_odom, 'map',
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        current_x_map = pose_map.pose.position.x
                        current_y_map = pose_map.pose.position.y
                    except:
                        current_x_map = self.current_odom.pose.pose.position.x
                        current_y_map = self.current_odom.pose.pose.position.y
                    
                    if self.nav_manager.current_goal_index > 0:
                        goal_idx = self.nav_manager.current_goal_index - 1
                    else:
                        goal_idx = 0
                    
                    if goal_idx < len(self.nav_manager.loaded_points):
                        goal_point = self.nav_manager.loaded_points[goal_idx]
                        goal_x = goal_point[0]
                        goal_y = goal_point[1]
                        distance = math.sqrt((goal_x - current_x_map)**2 + (goal_y - current_y_map)**2)
                        self.get_logger().info(
                            f"✅ Nav2报告导航成功（状态: {status_name}），"
                            f"距离目标点: {distance:.3f} 米"
                        )
            elif status == 6:  # UNKNOWN - 通常表示无法规划路径
                # 状态码6表示Nav2无法规划路径，这通常意味着：
                # 1. 目标点在地图外或不可达区域
                # 2. 没有可行的路径（障碍物阻挡）
                # 3. 地图或定位有问题
                # 在这种情况下，小车可能还在执行之前的cmd_vel命令，导致路径不对
                self.get_logger().warn(
                    f"⚠️  Nav2状态码: {status_name}（无法规划路径）！"
                    f"这可能导致小车执行错误的路径。"
                    f"可能的原因："
                    f"  1. 目标点在地图外或不可达区域"
                    f"  2. 没有可行的路径（障碍物阻挡）"
                    f"  3. 地图或定位有问题"
                    f"建议检查："
                    f"  - 目标点是否在地图内"
                    f"  - 地图是否有障碍物"
                    f"  - 定位是否准确"
                )
            else:
                # 其他状态码（CANCELED, ABORTED, REJECTED）才认为是失败
                self.get_logger().warn(
                    f"⚠️  导航状态: {status_name}，但将继续通过距离检查判断是否到达目标点"
                )
        except Exception as e:
            self.get_logger().error(f"获取导航结果时出错: {e}")
    
    def auto_nav_timer_callback(self):
        """自动导航定时器回调"""
        if not self.auto_nav_enabled:
            return
        
        if self.nav_manager.is_navigating:
            self.send_current_goal()
    
    def command_callback(self, msg: String):
        """命令话题回调（用于接收命令）"""
        command = msg.data.strip()
        if command:
            self.get_logger().info(f"【通过话题接收命令】{command}")
            self.handle_command(command)
    
    def execute_command_service(self, request, response):
        """执行命令服务（用于接收命令）"""
        # 这个服务需要一个参数，我们通过request的字段来传递命令
        # 但Trigger服务没有参数，所以我们改用话题方式
        response.success = True
        response.message = "请使用话题 /navigation_test_command 发送命令"
        return response
    
    def check_action_server(self):
        """检查Action服务器是否就绪"""
        if not self.action_server_ready:
            if self.nav_action_client.wait_for_server(timeout_sec=0.5):
                self.action_server_ready = True
                self.action_check_timer.cancel()
                self.get_logger().info("✅ Nav2 Action 服务器已就绪")
    
    def print_status(self):
        """打印当前状态"""
        status_msg = String()
        status = []
        
        if self.nav_manager.is_recording:
            status.append("记录中")
        if self.nav_manager.is_navigating:
            status.append("导航中")
        if self.auto_nav_enabled:
            status.append("自动导航")
        
        if self.current_odom:
            x = self.current_odom.pose.pose.position.x
            y = self.current_odom.pose.pose.position.y
            status.append(f"位置: ({x:.2f}, {y:.2f})")
        
        status.append(f"已记录: {self.nav_manager.get_points_count()} 个点")
        status.append(f"已加载: {self.nav_manager.get_loaded_points_count()} 个点")
        
        if self.nav_manager.is_navigating:
            status.append(f"当前目标: {self.nav_manager.get_current_goal_index() + 1}")
        
        status_msg.data = " | ".join(status)
        self.status_pub.publish(status_msg)
    
    def handle_command(self, command: str):
        """处理命令"""
        import sys
        command = command.strip().lower()
        
        # 调试：确认命令被接收
        self.get_logger().info(f"收到命令: {command}")
        
        # 简化命令：支持单个字符
        if command == 'start_record' or command == '1' or command == 's':
            if not self.nav_manager.is_recording:
                # 在开始记录前，确保已设置初始位置
                if not self.initial_pose_set:
                    self.get_logger().info("自动设置初始位置...")
                    self.auto_set_initial_pose()
                    # 等待一下让初始位置生效
                    time.sleep(0.5)
                
                self.nav_manager.start_recording()
                # 创建定时器，定期记录路径点
                if self.record_timer is None:
                    self.record_timer = self.create_timer(
                        self.nav_manager.record_interval,
                        self.record_navigation_point
                    )
                    self.get_logger().info(f"已创建记录定时器，间隔: {self.nav_manager.record_interval}秒")
                msg = ">>> 开始记录路径点 <<<"
                print(msg, flush=True)
                sys.stdout.flush()
                self.get_logger().info(msg)
                # 确保输出到stderr也能看到
                import sys
                print(msg, file=sys.stderr, flush=True)
            else:
                msg = "⚠️  已经在记录中"
                print(msg, flush=True)
                sys.stdout.flush()
                self.get_logger().warn("已经在记录中")
                print(msg, file=sys.stderr, flush=True)
        
        elif command == 'stop_record' or command == '2' or command == 't':
            import sys
            if self.nav_manager.is_recording:
                count = self.nav_manager.get_points_count()
                self.nav_manager.stop_recording()
                # 停止定时器
                if self.record_timer is not None:
                    self.record_timer.cancel()
                    self.record_timer = None
                    self.get_logger().info("已停止记录定时器")
                print(f">>> 停止记录，共记录 {count} 个点 <<<", flush=True)
                sys.stdout.flush()
                self.get_logger().info(f">>> 停止记录，共记录 {count} 个点 <<<")
            else:
                print("⚠️  当前没有在记录", flush=True)
                sys.stdout.flush()
                self.get_logger().warn("当前没有在记录")
        
        elif command == 'save_points' or command == '3' or command == 'v':
            import sys
            if self.nav_manager.get_points_count() > 0:
                self.nav_manager.save_points_to_csv()
                print(f"✅ 已保存 {self.nav_manager.get_points_count()} 个路径点到: {self.nav_manager.csv_file_path}", flush=True)
                sys.stdout.flush()
            else:
                print("⚠️  没有路径点可保存", flush=True)
                sys.stdout.flush()
                self.get_logger().warn("没有路径点可保存")
        
        elif command == 'load_points' or command == '4' or command == 'l':
            import sys
            if self.nav_manager.load_points_from_csv():
                count = self.nav_manager.get_loaded_points_count()
                print(f"✅ 成功加载 {count} 个路径点", flush=True)
                sys.stdout.flush()
                self.get_logger().info(f">>> 成功加载 {count} 个路径点 <<<")
            else:
                print("❌ 加载路径点失败，请检查CSV文件", flush=True)
                sys.stdout.flush()
                self.get_logger().error("加载路径点失败")
        
        elif command == 'send_goal' or command == '5' or command == 'g':
            import sys
            if self.nav_manager.is_navigating:
                if self.send_current_goal():
                    print("✅ 目标点已发送", flush=True)
                    sys.stdout.flush()
            else:
                print("⚠️  请先加载路径点（使用 load_points）", flush=True)
                sys.stdout.flush()
                self.get_logger().warn("请先加载路径点（使用 load_points）")
        
        elif command == 'next_goal' or command == '6' or command == 'n':
            import sys
            if self.nav_manager.is_navigating:
                self.nav_manager.move_to_next_goal()
                if self.send_current_goal():
                    print("✅ 已切换到下一个目标点并发送", flush=True)
                    sys.stdout.flush()
                else:
                    print("❌ 切换到下一个目标点失败：无法获取目标点", flush=True)
                    sys.stdout.flush()
                    self.get_logger().error("切换到下一个目标点失败：无法获取目标点")
            else:
                print("⚠️  请先加载路径点（使用 load_points）", flush=True)
                sys.stdout.flush()
                self.get_logger().warn("请先加载路径点（使用 load_points）")
        
        elif command == 'auto_nav_start' or command == '7' or command == 'a':
            if not self.nav_manager.is_navigating:
                if not self.nav_manager.load_points_from_csv():
                    print("❌ 无法启动自动导航：路径点加载失败")
                    self.get_logger().error("无法启动自动导航：路径点加载失败")
                    return
            
            if not self.auto_nav_enabled:
                self.auto_nav_enabled = True
                # 创建定时器，每1秒发送一次目标点
                self.auto_nav_timer = self.create_timer(1.0, self.auto_nav_timer_callback)
                # 立即发送第一个目标点
                if self.send_current_goal():
                    print("✅ 启动自动循环导航")
                self.get_logger().info(">>> 启动自动循环导航 <<<")
            else:
                print("⚠️  自动导航已经在运行")
                self.get_logger().warn("自动导航已经在运行")
        
        elif command == 'auto_nav_stop' or command == '8' or command == 'o':
            if self.auto_nav_enabled:
                self.auto_nav_enabled = False
                if self.auto_nav_timer:
                    self.auto_nav_timer.cancel()
                    self.auto_nav_timer = None
                print("✅ 停止自动循环导航")
                self.get_logger().info(">>> 停止自动循环导航 <<<")
            else:
                print("⚠️  自动导航未运行")
                self.get_logger().warn("自动导航未运行")
        
        elif command == 'show_status' or command == '9' or command == 'i':
            import sys
            print("\n" + "=" * 60, flush=True)
            print(f"记录状态: {'✅ 记录中' if self.nav_manager.is_recording else '❌ 未记录'}", flush=True)
            print(f"导航状态: {'✅ 导航中' if self.nav_manager.is_navigating else '❌ 未导航'}", flush=True)
            print(f"自动导航: {'✅ 运行中' if self.auto_nav_enabled else '❌ 未运行'}", flush=True)
            print(f"已记录路径点: {self.nav_manager.get_points_count()} 个", flush=True)
            print(f"已加载路径点: {self.nav_manager.get_loaded_points_count()} 个", flush=True)
            if self.nav_manager.is_navigating:
                print(f"当前目标点索引: {self.nav_manager.get_current_goal_index() + 1}/{self.nav_manager.get_loaded_points_count()}", flush=True)
            if self.current_odom:
                x = self.current_odom.pose.pose.position.x
                y = self.current_odom.pose.pose.position.y
                print(f"当前位置: ({x:.3f}, {y:.3f})", flush=True)
            print("=" * 60, flush=True)
            sys.stdout.flush()
            self.print_status()
        
        elif command == 'show_points' or command == '10' or command == '0' or command == 'p':
            import sys
            count = self.nav_manager.get_points_count()
            print(f"已记录 {count} 个路径点", flush=True)
            if count > 0:
                print(f"CSV文件路径: {self.nav_manager.csv_file_path}", flush=True)
            sys.stdout.flush()
            self.get_logger().info(f"已记录 {count} 个路径点")
        
        elif command == 'clear_points' or command == '11' or command == 'c':
            if self.nav_manager.is_recording:
                self.nav_manager.stop_recording()
            self.nav_manager.navigation_points = []
            print("✅ 已清空路径点（未保存）")
            self.get_logger().info("已清空路径点（未保存）")
        
        elif command == 'help' or command == 'h':
            self.print_usage()
        
        elif command == 'quit' or command == 'q' or command == 'exit':
            print("\n退出程序...")
            self.get_logger().info("退出程序...")
            rclpy.shutdown()
            return False
        
        else:
            import sys
            print(f"⚠️  未知命令: {command}，输入 'help' 查看帮助", flush=True)
            sys.stdout.flush()
            self.get_logger().warn(f"未知命令: {command}，输入 'help' 查看帮助")
        
        return True
    
    def run_interactive(self):
        """运行交互式命令行界面"""
        import threading
        import queue
        
        # 命令队列
        self.command_queue = queue.Queue()
        
        def input_thread():
            """输入线程"""
            import sys
            print("\n" + "="*60, flush=True)
            print("导航测试节点已启动", flush=True)
            print("="*60, flush=True)
            print("\n输入命令（输入 'help' 查看帮助，'quit' 退出）\n", flush=True)
            sys.stdout.flush()
            
            while rclpy.ok():
                try:
                    # 使用标准input，更可靠
                    sys.stdout.flush()
                    sys.stderr.flush()
                    command = input("导航测试> ").strip()
                    if command:
                        # 立即反馈输入被接收
                        self.get_logger().info(f"【输入接收】{command}")
                        self.command_queue.put(command)
                        print(f"[已接收: {command}]", flush=True)  # 立即反馈
                except (EOFError, KeyboardInterrupt):
                    print("\n收到中断信号，退出...", flush=True)
                    self.command_queue.put('quit')
                    break
                except Exception as e:
                    error_msg = f"输入线程错误: {e}"
                    print(error_msg, file=sys.stderr, flush=True)
                    self.get_logger().error(error_msg)
                    break
        
        # 启动输入线程
        input_thread_obj = threading.Thread(target=input_thread, daemon=True)
        input_thread_obj.start()
        
        # 主线程运行ROS2并处理命令
        import time
        import sys
        try:
            while rclpy.ok():
                # 处理ROS2消息
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # 处理命令队列
                try:
                    command = self.command_queue.get_nowait()
                    if command:
                        # 确保输出立即显示
                        sys.stdout.flush()
                        sys.stderr.flush()
                        # 在日志中明确显示收到命令
                        self.get_logger().info(f"【处理命令】{command}")
                        result = self.handle_command(command)
                        sys.stdout.flush()
                        sys.stderr.flush()
                        if not result:
                            break
                except queue.Empty:
                    pass
                
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = NavigationTestNode()
    
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

