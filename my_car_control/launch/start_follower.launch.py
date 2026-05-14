#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
启动激光雷达循迹节点的Launch文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成Launch描述"""
    
    # 获取功能包路径
    package_dir = get_package_share_directory('my_car_control')
    
    # 配置文件路径
    config_file = os.path.join(package_dir, 'config', 'follower_params.yaml')
    
    # 声明Launch参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 创建节点
    lidar_follower_node = Node(
        package='my_car_control',
        executable='lidar_follower_node',
        name='lidar_follower',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        lidar_follower_node,
    ])

