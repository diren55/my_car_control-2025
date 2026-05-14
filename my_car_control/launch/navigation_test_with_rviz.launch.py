#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航测试节点启动文件（带 RViz2 可视化）
同时启动导航测试节点和 RViz2，方便查看地图和导航信息
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
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
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动 RViz2'
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
    
    # RViz2 节点
    rviz_config_file = os.path.join(
        os.path.expanduser('~'),
        '.rviz2',
        'navigation_test.rviz'
    )
    
    # 如果配置文件不存在，使用默认配置
    rviz_args = []
    if os.path.exists(rviz_config_file):
        rviz_args = ['-d', rviz_config_file]
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )
    
    return LaunchDescription([
        csv_path_arg,
        record_interval_arg,
        min_distance_arg,
        use_rviz_arg,
        navigation_test_node,
        goal_bridge_node,
        rviz_node,
    ])

