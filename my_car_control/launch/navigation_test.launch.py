#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航测试节点启动文件
用于单独调试导航功能，不需要启动整个控制系统
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 声明参数
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value=os.path.expanduser('~/navigation_points.csv'),
        description='路径点CSV文件路径'
    )
    
    record_interval_arg = DeclareLaunchArgument(
        'record_interval',
        default_value='0.5',
        description='第一圈记录路径点的时间间隔（秒）'
    )
    
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.8',
        description='第二圈到达目标点的最小距离阈值（米）'
    )
    
    # 导航测试节点
    navigation_test_node = Node(
        package='my_car_control',
        executable='navigation_test_node',
        name='navigation_test_node',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'record_interval': LaunchConfiguration('record_interval'),
            'min_distance': LaunchConfiguration('min_distance'),
        }]
    )
    
    # 目标点话题到Action桥接节点
    goal_bridge_node = Node(
        package='my_car_control',
        executable='goal_pose_to_action_bridge',
        name='goal_pose_to_action_bridge',
        output='screen',
    )
    
    return LaunchDescription([
        csv_path_arg,
        record_interval_arg,
        min_distance_arg,
        navigation_test_node,
        goal_bridge_node,
    ])




