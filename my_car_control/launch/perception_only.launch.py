#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仅启动感知节点
- 用于测试感知功能
- 基于NCSC2024源程序main.py
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
    perception_config = LaunchConfiguration('perception_config')
    
    perception_config_arg = DeclareLaunchArgument(
        'perception_config',
        default_value=os.path.join(pkg_share, 'config', 'perception_params.yaml'),
        description='感知节点配置文件路径'
    )
    
    # 感知节点
    perception_node = Node(
        package='my_car_control',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[perception_config],
        remappings=[
            ('/scan', '/scan'),           # 激光雷达话题
            ('/target', '/target'),       # 目标点话题
        ]
    )
    
    return LaunchDescription([
        perception_config_arg,
        perception_node,
    ])

