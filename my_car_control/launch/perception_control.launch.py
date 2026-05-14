#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
感知+控制节点启动文件
功能: 同时启动感知节点（简单聚类）和控制节点

使用方法:
    # 用 YAML 默认值
    ros2 launch my_car_control perception_control.launch.py

    # 命令行覆盖某个 YAML 参数
    ros2 launch my_car_control perception_control.launch.py \\
        cluster_radius:=0.3 enable_mapping:=false

参数优先级（后者覆盖前者）：
    1. perception_node_simple_cluster.py 里 declare_parameter 的默认值
    2. config/perception_params.yaml（本 launch 加载）
    3. 命令行 launch 参数（仅显式列出的 4 个）
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========== YAML 配置文件路径 ==========
    perception_config = os.path.join(
        get_package_share_directory('my_car_control'),
        'config',
        'perception_params.yaml'
    )

    # ========== 常用快捷参数（命令行可覆盖 YAML 值）==========
    # 这些是经常调的几个，写成 launch args 方便快速 A/B 测。
    # 其它 23 个参数请改 perception_params.yaml 后重启节点。
    cluster_radius_arg = DeclareLaunchArgument(
        'cluster_radius', default_value='0.25',
        description='聚类半径（米），覆盖 YAML 中同名参数'
    )
    min_points_per_cluster_arg = DeclareLaunchArgument(
        'min_points_per_cluster', default_value='5',
        description='每个簇最少点数，覆盖 YAML 中同名参数'
    )
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping', default_value='false',
        description='是否启用 matplotlib 可视化（比赛建议 false）'
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window', default_value='true',
        description='是否弹窗，无图形环境必须 false'
    )

    # ========== 节点定义 ==========
    # 参数加载顺序：先 YAML 文件，再命令行覆盖（dict 在 list 末尾覆盖前者）
    perception_node = Node(
        package='my_car_control',
        executable='perception_node_simple_cluster',
        name='perception_node',
        parameters=[
            perception_config,                           # 1) 加载 YAML 27 个参数
            {                                            # 2) 命令行覆盖
                'cluster_radius': LaunchConfiguration('cluster_radius'),
                'min_points_per_cluster': LaunchConfiguration('min_points_per_cluster'),
                'enable_mapping': LaunchConfiguration('enable_mapping'),
                'show_window': LaunchConfiguration('show_window'),
            },
        ],
        output='screen'
    )

    control_node = Node(
        package='my_car_control',
        executable='control_node',
        name='control_node',
        output='screen'
    )

    # ========== 返回 LaunchDescription ==========
    return LaunchDescription([
        cluster_radius_arg,
        min_points_per_cluster_arg,
        enable_mapping_arg,
        show_window_arg,
        perception_node,
        control_node,
    ])
