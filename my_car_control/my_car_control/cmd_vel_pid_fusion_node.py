#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav2 速度命令PID融合节点（参考23张原的nav.py）
融合PID控制和TEB输出，提供更稳定的控制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
import math
import time


class CmdVelPidFusionNode(Node):
    """PID融合控制节点"""
    
    def __init__(self):
        super().__init__('cmd_vel_pid_fusion_node')
        
        # PID参数（修改为纯P控制，降低增益以消除抽搐）
        # 修改原因：禁用积分和微分项，消除滞后和高频震荡；降低P增益，降低响应强度
        self.declare_parameter('gain_p', 5.0)  # 比例增益（设置为5.0，平衡响应强度和平滑性）
        self.declare_parameter('gain_i', 0.0)   # 积分增益（强制设为0.0，禁用积分项）
        self.declare_parameter('gain_d', 0.0)  # 微分增益（强制设为0.0，禁用微分项，消除高频震荡）
        self.declare_parameter('k_smooth', 0.50)  # 平滑系数（增加到0.50，使输出更平滑，大幅削弱剧烈转向）
        self.declare_parameter('max_pid_output_deg', 45.0)  # 最大PID输出（度），降低到45度，限制最大转向角度，使角度范围更小（45-135度）
        
        self.gain_p = self.get_parameter('gain_p').get_parameter_value().double_value
        self.gain_i = self.get_parameter('gain_i').get_parameter_value().double_value
        self.gain_d = self.get_parameter('gain_d').get_parameter_value().double_value
        self.k_smooth = self.get_parameter('k_smooth').get_parameter_value().double_value
        max_pid_output_deg = self.get_parameter('max_pid_output_deg').get_parameter_value().double_value
        self.max_pid_output_rad = math.radians(max_pid_output_deg)  # 转换为弧度
        
        # TEB混合系数
        self.declare_parameter('teb_mix_k', 1.5)  # TEB混合系数（进一步降低到1.5，大幅削弱TEB对角度的影响，减少剧烈转向）
        
        # 角度变化率限制（防止突然跳变）
        self.declare_parameter('max_angular_change_per_step_deg', 10.0)  # 每步最大角度变化（度），防止突然跳变
        self.max_angular_change_per_step_deg = self.get_parameter('max_angular_change_per_step_deg').get_parameter_value().double_value
        self.last_angular_deg = 90.0  # 上次的角度值，用于限制变化率
        self.teb_mix_k = self.get_parameter('teb_mix_k').get_parameter_value().double_value
        
        # 速度参数
        self.declare_parameter('linear_scale', 100.0)  # m/s 转 PWM 的系数
        self.declare_parameter('linear_offset', 1500.0)  # PWM 中值（静止值，1500=停止）
        self.declare_parameter('linear_default', 1520.0)  # 默认速度（PWM值，有PID目标但无TEB输出时使用）
        self.declare_parameter('linear_stop', 1500.0)  # 停止速度（PWM值，1500=停止）
        self.declare_parameter('linear_max', 1530.0)  # 最大速度（PWM值，1530=最大速度）
        self.declare_parameter('linear_min', 1500.0)  # 最小速度（PWM值，1500=停止）
        
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.linear_offset = self.get_parameter('linear_offset').get_parameter_value().double_value
        self.linear_default = self.get_parameter('linear_default').get_parameter_value().double_value
        self.linear_stop = self.get_parameter('linear_stop').get_parameter_value().double_value
        self.linear_max = self.get_parameter('linear_max').get_parameter_value().double_value
        self.linear_min = self.get_parameter('linear_min').get_parameter_value().double_value
        
        # 角度限制（根据max_pid_output_deg自动计算，确保角度范围合理）
        self.declare_parameter('angular_center_deg', 90.0)  # 直行角度（度）
        # 注意：angular_min_deg 和 angular_max_deg 会根据 max_pid_output_deg 自动调整
        # 实际角度范围 = angular_center_deg ± max_pid_output_deg
        self.declare_parameter('angular_min_deg', 30.0)   # 最小角度（度，保留原值作为下限）
        self.declare_parameter('angular_max_deg', 150.0)   # 最大角度（度，保留原值作为上限）
        
        self.angular_min_deg = self.get_parameter('angular_min_deg').get_parameter_value().double_value
        self.angular_max_deg = self.get_parameter('angular_max_deg').get_parameter_value().double_value
        self.angular_center_deg = self.get_parameter('angular_center_deg').get_parameter_value().double_value
        
        # 控制频率
        self.declare_parameter('control_frequency', 200.0)  # 控制频率（Hz，23张原使用200Hz）
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # PID状态变量
        self.last_error = 0.0
        self.last_out = 0.0
        self.last_pid_out = 0.0  # 上次PID输出（用于限制变化率，在平滑处理之前的值）
        self.last_current = 0.0
        self.integral = 0.0
        
        # 当前状态
        self.current_omega = 0.0  # 当前角速度（rad/s）
        self.current_pose = None  # 当前位置
        self.target_theta = 0.0   # 目标角度（弧度）
        self.smoothed_target_theta = 0.0  # 平滑后的目标角度（弧度）
        self.has_target = False    # 是否有目标
        self.global_path = None    # 全局路径
        self.last_target_idx = None  # 上次选择的目标点索引，用于保持连续性
        
        # 目标角度平滑参数
        self.declare_parameter('target_angle_smooth_factor', 0.3)  # 目标角度平滑系数（0-1，越小越平滑）
        self.declare_parameter('target_angle_max_change_rad', 0.2)  # 目标角度最大变化率（弧度/步，约11.5度/步）
        self.target_angle_smooth_factor = self.get_parameter('target_angle_smooth_factor').get_parameter_value().double_value
        self.target_angle_max_change_rad = self.get_parameter('target_angle_max_change_rad').get_parameter_value().double_value
        
        # 前瞻点选择参数
        # 修改原因：增大最小前瞻距离，防止机器人离终点很近时选取的点过于贴近当前位置，避免45°-135°的角度跳变
        self.declare_parameter('lookahead_dist_min', 0.8)  # 最小前瞻距离（米，从0.5增大到0.8）
        self.declare_parameter('lookahead_dist_max', 1.5)  # 最大前瞻距离（米）
        self.declare_parameter('lookahead_dist_factor', 0.5)  # 前瞻距离因子（根据速度调整）
        self.lookahead_dist_min = self.get_parameter('lookahead_dist_min').get_parameter_value().double_value
        self.lookahead_dist_max = self.get_parameter('lookahead_dist_max').get_parameter_value().double_value
        self.lookahead_dist_factor = self.get_parameter('lookahead_dist_factor').get_parameter_value().double_value
        
        # TEB速度命令
        self.teb_vel = Twist()
        self.teb_vel.linear.x = 0.0
        self.teb_vel.angular.z = 0.0
        
        # TEB输出超时检测（如果TEB输出为0持续超过此时间，清除目标状态）
        self.declare_parameter('teb_timeout', 2.0)  # TEB输出超时时间（秒）
        self.teb_timeout = self.get_parameter('teb_timeout').get_parameter_value().double_value
        self.last_teb_time = None  # 上次收到TEB输出的时间
        
        # 订阅话题
        # 1. TEB输出（Nav2的速度命令）
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.teb_callback,
            10
        )
        
        # 2. 里程计（获取当前角速度）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_combined',
            self.odom_callback,
            10
        )
        
        # 3. 全局路径（用于计算目标角度，可选）
        self.path_sub = self.create_subscription(
            Path,
            '/plan',  # Nav2的全局路径
            self.path_callback,
            10
        )
        
        # 4. 目标点（用于计算目标角度，可选）
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 发布话题
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/teleop_cmd_vel',
            10
        )
        
        # 定时器（PID控制循环）
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.pid_controller_callback
        )
        
        self.get_logger().info('PID融合控制节点已启动')
        self.get_logger().info(f'PID参数: P={self.gain_p}, I={self.gain_i}, D={self.gain_d}')
        self.get_logger().info(f'PID输出限制: ±{max_pid_output_deg}度 (±{self.max_pid_output_rad:.3f}弧度)')
        self.get_logger().info(f'平滑系数: {self.k_smooth}')
        self.get_logger().info(f'角度变化率限制: ±{self.max_angular_change_per_step_deg}度/步')
        self.get_logger().info(f'TEB混合系数: {self.teb_mix_k}')
        self.get_logger().info(f'控制频率: {self.control_frequency} Hz')
        self.get_logger().info(f'速度限制: min={self.linear_min}, max={self.linear_max}, scale={self.linear_scale}, offset={self.linear_offset}')
        self.get_logger().info(f'角度范围: {self.angular_min_deg}° - {self.angular_max_deg}° (中心: {self.angular_center_deg}°)')
        self.get_logger().info(f'目标角度平滑: 平滑系数={self.target_angle_smooth_factor}, 最大变化率=±{math.degrees(self.target_angle_max_change_rad):.1f}度/步')
        self.get_logger().info(f'前瞻距离: {self.lookahead_dist_min}-{self.lookahead_dist_max}米')
    
    def quaternion_to_yaw(self, quaternion: Quaternion) -> float:
        """从四元数提取yaw角（弧度）"""
        # 四元数: [x, y, z, w]
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # 计算yaw角（绕z轴旋转）
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def teb_callback(self, msg: Twist):
        """TEB速度命令回调"""
        self.teb_vel = msg
        # 更新TEB输出时间戳
        self.last_teb_time = time.time()
    
    def odom_callback(self, msg: Odometry):
        """里程计回调，获取当前角速度和位置"""
        self.current_omega = msg.twist.twist.angular.z
        self.current_pose = msg.pose.pose
        
        # 修复问题1：从里程计提取yaw角，而不是积分角速度
        # 这样可以避免累积误差，并确保角度始终准确
        if self.current_pose is not None:
            self.last_current = self.quaternion_to_yaw(self.current_pose.orientation)
    
    def smooth_target_angle(self, new_target_angle):
        """
        平滑目标角度，防止突然变化
        使用低通滤波 + 变化率限制
        """
        # 初始化smoothed_target_theta（如果还没有）
        if not hasattr(self, 'smoothed_target_theta'):
            self.smoothed_target_theta = new_target_angle
            return new_target_angle
        
        # 如果之前没有目标（has_target=False），使用当前朝向作为起点，而不是直接使用新目标
        # 这样可以避免从0度突然跳到新目标角度
        if not self.has_target and hasattr(self, 'last_current') and self.last_current is not None:
            # 从当前朝向平滑过渡到新目标
            # 关键修复：如果smoothed_target_theta已经存在，不要完全重置，而是平滑过渡
            if hasattr(self, 'smoothed_target_theta'):
                # 计算从smoothed_target_theta到last_current的角度差
                diff = self.last_current - self.smoothed_target_theta
                # 归一化到[-π, π]
                while diff > math.pi:
                    diff -= 2 * math.pi
                while diff < -math.pi:
                    diff += 2 * math.pi
                # 平滑过渡到last_current（使用50%的平滑系数）
                self.smoothed_target_theta = self.smoothed_target_theta + diff * 0.5
            else:
                self.smoothed_target_theta = self.last_current
        
        # 计算角度差（考虑角度环绕）
        angle_diff = new_target_angle - self.smoothed_target_theta
        # 归一化到[-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 限制变化率（更严格的限制）
        if abs(angle_diff) > self.target_angle_max_change_rad:
            if angle_diff > 0:
                angle_diff = self.target_angle_max_change_rad
            else:
                angle_diff = -self.target_angle_max_change_rad
        
        # 低通滤波（使用更强的平滑，确保变化更平滑）
        # 如果角度差很大，使用更强的平滑系数
        if abs(angle_diff) > self.target_angle_max_change_rad * 0.5:
            # 角度差较大时，使用更强的平滑（平滑系数增加）
            smooth_factor = min(self.target_angle_smooth_factor * 1.5, 0.8)  # 最多0.8
        else:
            smooth_factor = self.target_angle_smooth_factor
        
        smoothed_diff = angle_diff * (1.0 - smooth_factor)
        self.smoothed_target_theta = self.smoothed_target_theta + smoothed_diff
        
        # 归一化到[-π, π]
        while self.smoothed_target_theta > math.pi:
            self.smoothed_target_theta -= 2 * math.pi
        while self.smoothed_target_theta < -math.pi:
            self.smoothed_target_theta += 2 * math.pi
        
        return self.smoothed_target_theta
    
    def find_lookahead_point(self, path_poses, current_x, current_y):
        """
        改进的前瞻点选择逻辑
        使用动态前瞻距离，并选择更稳定的点
        """
        if len(path_poses) == 0:
            return None
        
        # 关键修复：如果之前有目标点，优先选择接近上次目标点的点，避免突然跳跃
        if hasattr(self, 'last_target_idx') and self.last_target_idx is not None:
            last_idx = self.last_target_idx
            # 检查上次的目标点是否仍然有效
            if last_idx < len(path_poses):
                last_pose = path_poses[last_idx].pose.position
                dist_to_last = math.sqrt((last_pose.x - current_x)**2 + (last_pose.y - current_y)**2)
                # 如果上次的目标点仍然在合理范围内（0.2-2.0米），优先使用它
                if 0.2 <= dist_to_last <= 2.0:
                    return last_idx
                # 如果上次的目标点太远，尝试选择路径上最接近上次目标点的点（向前搜索）
                # 这样可以确保选择的点不会突然跳到路径前端
                if last_idx < len(path_poses):
                    # 从上次目标点开始，向前搜索最接近前瞻距离的点
                    min_diff = float('inf')
                    best_idx = last_idx
                    for i in range(last_idx, min(last_idx + 10, len(path_poses))):  # 最多向前搜索10个点
                        pose = path_poses[i].pose.position
                        dist = math.sqrt((pose.x - current_x)**2 + (pose.y - current_y)**2)
                        # 计算动态前瞻距离
                        current_speed = abs(self.teb_vel.linear.x) if hasattr(self, 'teb_vel') else 0.0
                        lookahead_dist = self.lookahead_dist_min + (self.lookahead_dist_max - self.lookahead_dist_min) * min(current_speed * self.lookahead_dist_factor, 1.0)
                        diff = abs(dist - lookahead_dist)
                        if diff < min_diff:
                            min_diff = diff
                            best_idx = i
                    return best_idx
        
        # 计算动态前瞻距离（根据速度调整）
        current_speed = abs(self.teb_vel.linear.x) if hasattr(self, 'teb_vel') else 0.0
        lookahead_dist = self.lookahead_dist_min + (self.lookahead_dist_max - self.lookahead_dist_min) * min(current_speed * self.lookahead_dist_factor, 1.0)
        
        # 找到路径上距离当前位置最近的点
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose_stamped in enumerate(path_poses):
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 从最近点开始，向前查找满足前瞻距离的点
        # 修改原因：确保即使机器人离终点很近，选取的点也不要过于贴近机器人当前位置（防止45°-135°的角度跳变）
        target_idx = len(path_poses) - 1  # 默认选择最后一个点
        
        for i in range(closest_idx, len(path_poses)):
            pose_stamped = path_poses[i]
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            # 确保距离至少为lookahead_dist_min（0.8米），防止选取的点过于贴近当前位置
            if dist >= max(lookahead_dist, self.lookahead_dist_min):
                target_idx = i
                break
        
        # 如果找到的点距离仍然小于最小前瞻距离，强制选择至少满足最小前瞻距离的点
        final_pose = path_poses[target_idx].pose.position
        final_dist = math.sqrt((final_pose.x - current_x)**2 + (final_pose.y - current_y)**2)
        if final_dist < self.lookahead_dist_min and target_idx < len(path_poses) - 1:
            # 如果当前点距离太近，尝试选择更远的点
            for i in range(target_idx + 1, len(path_poses)):
                pose = path_poses[i].pose.position
                dist = math.sqrt((pose.x - current_x)**2 + (pose.y - current_y)**2)
                if dist >= self.lookahead_dist_min:
                    target_idx = i
                    break
        
        return target_idx
    
    def path_callback(self, msg: Path):
        """路径回调，从路径中计算目标角度（改进版：使用平滑处理）"""
        self.global_path = msg
        
        # 关键修复：只有当TEB有输出时，才更新目标角度
        # 如果TEB输出为0，说明Nav2没有在导航，不应该设置目标
        if abs(self.teb_vel.linear.x) < 1e-5 and abs(self.teb_vel.angular.z) < 1e-5:
            # TEB没有输出，不更新目标（但保持has_target状态，让超时机制处理）
            return
        
        # 修复：检查路径是否有效（至少2个点，且不是空路径）
        if len(msg.poses) < 2:
            self.has_target = False
            return
        
        # 修复：检查路径点是否在原点（可能是初始化的空路径）
        if len(msg.poses) > 0:
            first_pose = msg.poses[0].pose.position
            if abs(first_pose.x) < 0.01 and abs(first_pose.y) < 0.01:
                # 路径起点在原点，可能是初始化的空路径，不设置目标
                if len(msg.poses) == 1:
                    self.has_target = False
                    return
        
        # 如果有当前位置，计算到路径上前瞻点的角度
        if self.current_pose is not None:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            
            # 使用改进的前瞻点选择逻辑
            target_idx = self.find_lookahead_point(msg.poses, current_x, current_y)
            
            if target_idx is not None and target_idx < len(msg.poses):
                target_pose = msg.poses[target_idx].pose.position
                dx = target_pose.x - current_x
                dy = target_pose.y - current_y
                
                if abs(dx) > 0.01 or abs(dy) > 0.01:
                    # 计算目标点在全局坐标系中的角度
                    target_angle_global = math.atan2(dy, dx)
                    
                    # 修复：检查目标点是否在机器人前方
                    # 计算目标点相对于机器人当前朝向的相对角度
                    if hasattr(self, 'last_current') and self.last_current is not None:
                        relative_angle = target_angle_global - self.last_current
                        
                        # 归一化相对角度到[-π, π]
                        while relative_angle > math.pi:
                            relative_angle -= 2 * math.pi
                        while relative_angle < -math.pi:
                            relative_angle += 2 * math.pi
                        
                        # 如果目标点在机器人后方（相对角度>90度或<-90度），选择更近的点
                        if abs(relative_angle) > math.pi / 2:
                            # 选择更近的点（至少0.3米，避免选择太近的点）
                            min_dist = 0.3
                            for i, pose_stamped in enumerate(msg.poses):
                                dx_near = pose_stamped.pose.position.x - current_x
                                dy_near = pose_stamped.pose.position.y - current_y
                                dist_near = math.sqrt(dx_near*dx_near + dy_near*dy_near)
                                
                                if dist_near >= min_dist:
                                    target_pose = pose_stamped.pose.position
                                    dx = target_pose.x - current_x
                                    dy = target_pose.y - current_y
                                    target_angle_global = math.atan2(dy, dx)
                                    break
                    
                    # 使用平滑处理设置目标角度
                    smoothed_angle = self.smooth_target_angle(target_angle_global)
                    self.target_theta = smoothed_angle
                    self.has_target = True
                    # 保存目标点索引，用于下次选择时保持连续性
                    self.last_target_idx = target_idx
                    return
        
        # 如果没有当前位置，使用路径的前几个点计算
        lookahead_idx = min(5, len(msg.poses) - 1)
        if lookahead_idx < len(msg.poses):
            target_pose = msg.poses[lookahead_idx].pose.position
            dx = target_pose.x
            dy = target_pose.y
            
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                new_target = math.atan2(dy, dx)
                smoothed_angle = self.smooth_target_angle(new_target)
                self.target_theta = smoothed_angle
                self.has_target = True
            else:
                self.has_target = False
    
    def goal_callback(self, msg: PoseStamped):
        """目标点回调，计算目标角度（改进版：使用平滑处理）"""
        if self.current_pose is not None:
            # 从当前位置到目标点的角度
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            
            dx = goal_x - current_x
            dy = goal_y - current_y
            
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                new_target = math.atan2(dy, dx)
                smoothed_angle = self.smooth_target_angle(new_target)
                self.target_theta = smoothed_angle
                self.has_target = True
            else:
                self.has_target = False
        else:
            # 如果没有当前位置，使用目标点相对于原点的角度
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            
            if abs(goal_x) > 0.01 or abs(goal_y) > 0.01:
                new_target = math.atan2(goal_y, goal_x)
                smoothed_angle = self.smooth_target_angle(new_target)
                self.target_theta = smoothed_angle
                self.has_target = True
            else:
                self.has_target = False
    
    def compute_theta_from_teb(self):
        """从TEB速度命令计算目标角度（简化方法，改进版：使用平滑处理）"""
        # 如果TEB有角速度输出，可以基于此计算目标角度
        # 这是一个简化的方法，实际应该基于路径
        if abs(self.teb_vel.angular.z) > 1e-5:
            # 基于角速度和线速度计算转弯半径，然后计算目标角度
            if abs(self.teb_vel.linear.x) > 0.01:
                # 转弯半径 = v / omega
                radius = self.teb_vel.linear.x / self.teb_vel.angular.z
                # 目标角度 = atan(wheelbase / radius)
                wheelbase = 0.335  # 轴距
                new_target = math.atan(wheelbase / radius)
                # 使用平滑处理
                smoothed_angle = self.smooth_target_angle(new_target)
                self.target_theta = smoothed_angle
                self.has_target = True
            else:
                # TEB没有线速度输出，不设置目标
                self.has_target = False
        else:
            # 修复：TEB没有角速度输出，说明没有导航任务
            # 不应该设置目标角度，保持has_target=False，让PID保持当前状态
            self.has_target = False
    
    def pid_controller_callback(self):
        """PID控制循环（参考23张原的nav.py）"""
        # ========== 修改4：强化"停车即切断"逻辑 ==========
        # 修改原因：如果检测到TEB输出极小，立即切断PID计算，直接发送零速度命令并return
        # 不要等待超时，立即切断以防止到达终点后的震荡
        if abs(self.teb_vel.linear.x) < 0.01:
            # TEB输出极小，立即切断
            self.has_target = False
            self.integral = 0.0  # 清空PID积分
            self.last_out = 0.0  # 重置PID输出
            self.last_pid_out = 0.0  # 重置PID输出（用于斜率限制）
            self.last_error = 0.0
            self.last_target_idx = None  # 清除目标点索引
            
            # 直接发送零速度命令并return，不再进行后续PID计算
            cmd_vel = Twist()
            cmd_vel.linear.x = float(self.linear_stop)  # 1500 = 停止
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = float(self.angular_center_deg)  # 90度 = 直行
            self.cmd_vel_pub.publish(cmd_vel)
            return  # 立即返回，不进行后续计算
        
        # TEB有输出，更新超时计时
        current_time = time.time()
        self.last_teb_time = current_time
        
        # 如果没有目标，尝试从TEB计算
        if not self.has_target:
            self.compute_theta_from_teb()
        
        # 修复问题1：不再积分角速度，而是直接从里程计提取yaw角
        # last_current已经在odom_callback中从里程计更新，这里只需要获取dt用于积分项
        dt = 1.0 / self.control_frequency
        
        # 如果里程计还没有更新，使用积分作为后备（但应该尽快从里程计获取）
        if self.current_pose is None:
            # 后备方案：如果里程计未初始化，使用积分（但会记录警告）
            if not hasattr(self, '_warned_no_pose'):
                self.get_logger().warn('里程计pose未初始化，使用积分角速度作为后备（不推荐）')
                self._warned_no_pose = True
            self.last_current = self.last_current + dt * self.current_omega
        
        # 计算PID输出
        if self.has_target:
            target = self.target_theta
            current = self.last_current
            error = target - current
            
            # 修复：角度误差归一化到[-π, π]范围
            # 这样可以正确处理角度跨越±π边界的情况
            while error > math.pi:
                error -= 2 * math.pi
            while error < -math.pi:
                error += 2 * math.pi
            
            # PID计算
            p_term = self.gain_p * error
            i_term = self.gain_i * self.integral
            d_term = self.gain_d * (error - self.last_error)
            
            out = p_term + i_term + d_term
            
            # 修复：限制PID输出范围，防止输出过大导致控制不稳定
            # 限制在±max_pid_output_rad范围内（例如：±60度 = ±1.05弧度）
            out = max(-self.max_pid_output_rad, min(self.max_pid_output_rad, out))
            
            # ========== 修改1：实施严格的PID输出斜率限制 (Slew Rate Limiter) ==========
            # 修改原因：这是解决抽搐的最关键步骤，必须限制每次循环PID输出变化幅度不能超过0.05弧度
            # 计算PID输出相对于上次的变化
            if not hasattr(self, 'last_pid_out'):
                self.last_pid_out = 0.0
            
            # 计算变化量 delta = out - self.last_out
            delta = out - self.last_pid_out
            
            # 限制delta的最大值为0.05弧度（每次循环PID输出变化幅度不能超过0.05弧度）
            max_delta = 0.05  # 严格的斜率限制：0.05弧度/步
            if abs(delta) > max_delta:
                if delta > 0:
                    out = self.last_pid_out + max_delta
                else:
                    out = self.last_pid_out - max_delta
                # 再次限制在范围内
                out = max(-self.max_pid_output_rad, min(self.max_pid_output_rad, out))
            
            # 只有经过斜率限制后的out才能用于后续计算和更新self.last_pid_out
            # 注意：这里不再使用平滑处理，因为斜率限制已经足够严格
            # 平滑处理（保留原有逻辑，但斜率限制已经足够）
            out = (1.0 - self.k_smooth) * out + self.k_smooth * self.last_out
            
            # 更新状态
            # 注意：last_pid_out保存的是经过斜率限制后的值（在平滑处理之前）
            # 这样下次计算delta时，使用的是经过斜率限制的值
            self.last_pid_out = out  # 保存本次PID输出（经过斜率限制后，平滑处理之前的值）
            self.last_out = out
            self.last_error = error
            self.integral += error * dt
            
            # 限制积分项（防止积分饱和）
            if abs(self.integral) > 1.0:
                self.integral = 1.0 if self.integral > 0 else -1.0
        else:
            # 关键修复3：没有目标时，平滑过渡到0，而不是突然跳到0
            # 这样可以避免角度在45度和135度之间突然跳变
            target_out = 0.0  # 目标输出是0
            
            # 限制PID输出变化率（即使没有目标，也要平滑过渡）
            if not hasattr(self, 'last_pid_out'):
                self.last_pid_out = 0.0
            
            # 计算到0的变化量
            # 修改原因：使用与有目标时相同的严格斜率限制（0.05弧度/步）
            pid_output_change = target_out - self.last_pid_out
            max_pid_change_rad = 0.05  # 使用严格的斜率限制：0.05弧度/步
            if abs(pid_output_change) > max_pid_change_rad:
                if pid_output_change > 0:
                    target_out = self.last_pid_out + max_pid_change_rad
                else:
                    target_out = self.last_pid_out - max_pid_change_rad
            
            # 使用平滑处理，逐渐过渡到0
            out = (1.0 - self.k_smooth) * target_out + self.k_smooth * self.last_out
            
            # 如果已经非常接近0（小于0.01），直接设为0并重置状态
            if abs(out) < 0.01:
                out = 0.0
                self.last_out = 0.0
                self.last_pid_out = 0.0
                self.integral = 0.0  # 清除积分项
                self.last_error = 0.0
                self.last_angular_deg = self.angular_center_deg
                # 重置平滑后的目标角度为当前朝向
                if hasattr(self, 'smoothed_target_theta'):
                    self.smoothed_target_theta = self.last_current if hasattr(self, 'last_current') else 0.0
            else:
                # 还在过渡中，更新last_out和last_pid_out
                self.last_out = out
                self.last_pid_out = out
        
        # 计算基础角度（90度是直行）
        # 修复：PID输出out是弧度，需要转换为度
        angular_deg = self.angular_center_deg + math.degrees(out)
        
        # 限制角度范围
        angular_deg = max(self.angular_min_deg, min(self.angular_max_deg, angular_deg))
        
        # 计算线速度（PWM值）
        # 逻辑：有TEB输出 → 使用TEB速度；无TEB输出 → 停止（不管是否有PID目标）
        # 关键修复：TEB输出为0说明Nav2没有在导航，应该停止，而不是使用默认速度
        if abs(self.teb_vel.linear.x) > 1e-5:
            # 有TEB输出：使用TEB的线速度（Nav2正在导航）
            # 计算速度：PWM = m/s * scale + offset
            linear_pwm = self.teb_vel.linear.x * self.linear_scale + self.linear_offset
            # 立即限制在1500-1530范围内（关键修复：确保限制生效）
            if linear_pwm > self.linear_max:
                linear_pwm = self.linear_max
            if linear_pwm < self.linear_min:
                linear_pwm = self.linear_min
        else:
            # 无TEB输出：停止（Nav2没有在导航，不管是否有PID目标）
            linear_pwm = self.linear_stop  # 1500 = 停止
        
        # 最终限制线速度范围（双重保险：确保不超过1530）
        linear_pwm = max(self.linear_min, min(self.linear_max, linear_pwm))
        
        # 调试：如果超过限制，记录警告
        if linear_pwm > self.linear_max or linear_pwm < self.linear_min:
            self.get_logger().warn(
                f'速度限制异常！PWM={linear_pwm:.2f}, max={self.linear_max}, min={self.linear_min}, '
                f'TEB输出={self.teb_vel.linear.x:.3f} m/s'
            )
        
        # 混合TEB的角速度（参考23张原第112行）
        # 关键修复：TEB混合应该在PID输出限制之后进行，并且要限制混合后的角度变化率
        if abs(self.teb_vel.angular.z) < 1e-5 and abs(self.teb_vel.linear.x) < 1e-5:
            teb_mix_k = 0.0  # TEB没有输出，不混合
        else:
            teb_mix_k = self.teb_mix_k
        
        # 将TEB的rad/s转换为角度增量并混合
        # 注意：23张原直接将rad/s加到角度值上，这里做类似处理
        # 但需要适当的缩放
        max_angular_velocity = 1.5  # 最大角速度（rad/s），参考TEB配置
        teb_angular_deg = (self.teb_vel.angular.z / max_angular_velocity) * 90.0  # 转换为角度增量
        
        # 先限制TEB角度增量的变化率（防止TEB导致突然跳变）
        # 计算TEB角度增量相对于上次的变化
        if not hasattr(self, 'last_teb_angular_deg'):
            self.last_teb_angular_deg = 0.0
        
        teb_angular_change = teb_angular_deg - self.last_teb_angular_deg
        # 关键修复：限制TEB角度增量的变化率，使用更严格的限制
        max_teb_change = self.max_angular_change_per_step_deg * 0.5  # 限制为角度变化率限制的一半
        if abs(teb_angular_change) > max_teb_change:
            if teb_angular_change > 0:
                teb_angular_deg = self.last_teb_angular_deg + max_teb_change
            else:
                teb_angular_deg = self.last_teb_angular_deg - max_teb_change
        self.last_teb_angular_deg = teb_angular_deg
        
        # 关键修复：在混合TEB之前，保存当前角度，以便限制混合后的变化率
        angular_deg_before_teb = angular_deg
        angular_deg = angular_deg + teb_mix_k * teb_angular_deg
        
        # 关键修复：限制TEB混合后的角度变化率，确保不会导致突然跳变
        teb_mixed_change = angular_deg - angular_deg_before_teb
        max_teb_mixed_change = self.max_angular_change_per_step_deg * 0.3  # 限制TEB混合的最大变化
        if abs(teb_mixed_change) > max_teb_mixed_change:
            if teb_mixed_change > 0:
                angular_deg = angular_deg_before_teb + max_teb_mixed_change
            else:
                angular_deg = angular_deg_before_teb - max_teb_mixed_change
        
        # 再次限制角度范围
        angular_deg = max(self.angular_min_deg, min(self.angular_max_deg, angular_deg))
        
        # 限制角度变化率（防止突然跳变）- 这是最终限制
        angular_change = angular_deg - self.last_angular_deg
        if abs(angular_change) > self.max_angular_change_per_step_deg:
            # 如果变化超过限制，限制变化量
            if angular_change > 0:
                angular_deg = self.last_angular_deg + self.max_angular_change_per_step_deg
            else:
                angular_deg = self.last_angular_deg - self.max_angular_change_per_step_deg
            # 再次确保在范围内
            angular_deg = max(self.angular_min_deg, min(self.angular_max_deg, angular_deg))
            # 记录警告，帮助调试
            if not hasattr(self, '_warned_angular_change'):
                self._warned_angular_change = False
            if not self._warned_angular_change:
                self.get_logger().warn(
                    f'角度变化率超过限制！变化: {angular_change:.1f}度, 限制: ±{self.max_angular_change_per_step_deg}度, '
                    f'从 {self.last_angular_deg:.1f}度 限制到 {angular_deg:.1f}度'
                )
                self._warned_angular_change = True
        
        # 更新上次角度值
        self.last_angular_deg = angular_deg
        
        # 创建并发布速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = float(linear_pwm)
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = float(angular_deg)  # 角度值（0-180度）
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 调试日志（降低频率）
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % int(self.control_frequency / 10) == 0:  # 每0.1秒打印一次
            self.get_logger().info(
                f'PID输出: {out:.3f}, 角度: {angular_deg:.1f}度, '
                f'TEB: {self.teb_vel.angular.z:.3f} rad/s, '
                f'速度: {linear_pwm:.1f} PWM'
            )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = CmdVelPidFusionNode()
    
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

