#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
感知+控制节点启动文件
功能: 同时启动感知节点（简单聚类）和控制节点
使用方法: ros2 launch my_car_control perception_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========== 感知节点参数 ==========
    cluster_radius_arg = DeclareLaunchArgument(
        'cluster_radius',
        default_value='0.25',
        description='聚类半径（米）'
    )
    
    min_points_per_cluster_arg = DeclareLaunchArgument(
        'min_points_per_cluster',
        default_value='5',
        description='每个簇最少点数'
    )
    
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping',
        default_value='false',
        description='是否启用可视化'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='是否显示弹窗'
    )
    
    # ========== 节点定义 ==========
    perception_node = Node(
        package='my_car_control',
        executable='perception_node_simple_cluster',
        name='perception_node',
        parameters=[{
            'cluster_radius': LaunchConfiguration('cluster_radius'),
            'min_points_per_cluster': LaunchConfiguration('min_points_per_cluster'),
            'enable_mapping': LaunchConfiguration('enable_mapping'),
            'show_window': LaunchConfiguration('show_window'),
        }],
        output='screen'
    )
    
    control_node = Node(
        package='my_car_control',
        executable='control_node',
        name='control_node',
        output='screen'
    )
    
    # ========== 返回LaunchDescription ==========
    return LaunchDescription([
        cluster_radius_arg,
        min_points_per_cluster_arg,
        enable_mapping_arg,
        show_window_arg,
        perception_node,
        control_node,
    ])

