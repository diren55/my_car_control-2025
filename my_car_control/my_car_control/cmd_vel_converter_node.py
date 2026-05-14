#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav2 速度命令转换节点
将 Nav2 发布的标准 ROS 速度（m/s, rad/s）转换为小车需要的 PWM 值
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelConverterNode(Node):
    """速度命令转换节点"""
    
    def __init__(self):
        super().__init__('cmd_vel_converter_node')
        
        # 参数：速度转换系数
        # 参考 racecar_driver_node.cpp 的 teleop_TwistCallback:
        #   linear_x = twist->linear.x;  (直接使用 PWM 值)
        #   angle = 2500.0 - twist->angular.z * 2000.0 / 180.0;  (angular.z 是角度值 0-180度)
        self.declare_parameter('linear_scale', 100.0)  # m/s 转 PWM 的系数
        self.declare_parameter('linear_offset', 1500.0)  # PWM 中值（静止值）
        
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.linear_offset = self.get_parameter('linear_offset').get_parameter_value().double_value
        
        # 速度限制（PWM 值）
        self.declare_parameter('linear_min', 500.0)   # 最小速度（反向）
        self.declare_parameter('linear_max', 2500.0)  # 最大速度（正向）
        
        self.linear_min = self.get_parameter('linear_min').get_parameter_value().double_value
        self.linear_max = self.get_parameter('linear_max').get_parameter_value().double_value
        
        # 角度限制（0-180度）
        self.declare_parameter('angular_min_deg', 0.0)   # 最小角度（度）
        self.declare_parameter('angular_max_deg', 180.0)  # 最大角度（度）
        
        self.angular_min_deg = self.get_parameter('angular_min_deg').get_parameter_value().double_value
        self.angular_max_deg = self.get_parameter('angular_max_deg').get_parameter_value().double_value
        
        # 订阅 Nav2 的速度命令
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10
        )
        
        # 发布转换后的速度命令到小车底盘
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/teleop_cmd_vel',
            10
        )
        
        self.get_logger().info('速度命令转换节点已启动')
        self.get_logger().info(f'订阅: /cmd_vel_nav -> 发布: /teleop_cmd_vel')
        self.get_logger().info(f'速度转换: linear = input * {self.linear_scale} + {self.linear_offset} (PWM值)')
        self.get_logger().info(f'角度转换: angular = rad/s -> 角度值(0-180度)')
        self.get_logger().info(f'注意: racecar_driver 的 teleop_TwistCallback 期望 angular.z 是角度值(0-180度)')
    
    def cmd_vel_callback(self, msg: Twist):
        """转换并转发速度命令"""
        # 创建新的 Twist 消息
        converted_msg = Twist()
        
        # 转换线速度：m/s -> PWM
        # 公式：PWM = m/s * scale + offset
        # 参考 racecar_driver_node.cpp: linear_x = twist->linear.x * 100 + 1500
        linear_pwm = msg.linear.x * self.linear_scale + self.linear_offset
        
        # 限制线速度范围
        linear_pwm = max(self.linear_min, min(self.linear_max, linear_pwm))
        
        # 转换角速度：rad/s -> 角度值（0-180度）
        # 参考 racecar_driver_node.cpp 的 teleop_TwistCallback:
        #   angle = 2500.0 - twist->angular.z * 2000.0 / 180.0;
        #   其中 twist->angular.z 是角度值（0-180度），不是 PWM 值！
        # 
        # 从公式看：angle = 2500.0 - angular.z * 2000.0 / 180.0
        #   angular.z = 0 -> angle = 2500（最大左转）
        #   angular.z = 90 -> angle = 1500（直行）
        #   angular.z = 180 -> angle = 500（最大右转）
        # 
        # Nav2 输出的是 rad/s：
        # - angular.z > 0 = 左转（逆时针）
        # - angular.z = 0 = 直行
        # - angular.z < 0 = 右转（顺时针）
        # 
        # 转换映射：
        # - Nav2 angular.z = 0 (rad/s) -> 角度 = 90度（直行）
        # - Nav2 angular.z > 0 (rad/s, 左转) -> 角度 < 90度（左转，0度=最大左转）
        # - Nav2 angular.z < 0 (rad/s, 右转) -> 角度 > 90度（右转，180度=最大右转）
        # 
        # 公式：角度 = 90 - (rad/s / max_angular_velocity) * 90
        # 其中 max_angular_velocity 是最大角速度（rad/s）
        import math
        max_angular_velocity = 1.0  # rad/s，最大角速度（根据Nav2配置，实际可能更大）
        # 角度值 = 90度（直行） - rad/s * (90度 / 最大角速度)
        # 这样：rad/s = 0 -> 90度，rad/s = 1 -> 0度，rad/s = -1 -> 180度
        angular_deg = 90.0 - (msg.angular.z / max_angular_velocity) * 90.0
        
        # 限制角度范围（0-180度）
        angular_deg = max(self.angular_min_deg, min(self.angular_max_deg, angular_deg))
        
        # 设置转换后的值
        converted_msg.linear.x = float(linear_pwm)
        converted_msg.linear.y = 0.0
        converted_msg.linear.z = 0.0
        converted_msg.angular.x = 0.0
        converted_msg.angular.y = 0.0
        converted_msg.angular.z = float(angular_deg)  # 角度值（0-180度）
        
        # 发布转换后的速度命令
        self.cmd_vel_pub.publish(converted_msg)
        
        # 打印日志（用于调试，每10次打印一次）
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'转换速度: {msg.linear.x:.3f} m/s -> {linear_pwm:.1f} PWM, '
                f'{msg.angular.z:.3f} rad/s -> {angular_deg:.1f} 度'
            )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = CmdVelConverterNode()
    
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

