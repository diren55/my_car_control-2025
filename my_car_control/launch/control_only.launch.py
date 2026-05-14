#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仅启动控制节点
- 用于测试控制功能
- 基于NCSC2024源程序racecar_teleop_test.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('my_car_control')
    
    # 声明启动参数
    control_config = LaunchConfiguration('control_config')
    
    control_config_arg = DeclareLaunchArgument(
        'control_config',
        default_value=os.path.join(pkg_share, 'config', 'control_params.yaml'),
        description='控制节点配置文件路径'
    )
    
    # 控制节点
    control_node = Node(
        package='my_car_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config],
        remappings=[
            ('/target', '/target'),                    # 目标点话题
            ('/encoder_imu_odom', '/encoder_imu_odom'), # 里程计话题
            ('/car_cmd_vel', '/car_cmd_vel'),          # 车辆控制话题
            ('/navi', '/navi'),                        # 导航话题
            ('/start', '/start'),                      # 开始话题
        ]
    )
    
    return LaunchDescription([
        control_config_arg,
        control_node,
    ])

