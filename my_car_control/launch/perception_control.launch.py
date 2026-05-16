#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
感知 + 视觉 + 控制 主 launch（默认入口，不带 A/B 投票）
================================================================================
启动节点：
  - perception_node_simple_cluster   ← 加载 perception_params.yaml
  - image_detector                    ← 当前硬编码 HSV 阈值（未来可改）
  - control_node                      ← 加载 control_params.yaml【本次新增】

如要启用 A/B 投票后处理：用 perception_control_with_ab_vote.launch.py 替代。

================================================================================
2025 改造说明：
  - 原版（ba5a143）不加载 control_params.yaml，导致 control_node 完全用代码
    declare_parameter 默认值，YAML 调参看似可调实际无效。
  - 本次修复：control_node 加上 parameters=[control_config]，让 YAML 生效。
  - 行为不变：control_params.yaml 默认值与 control_node.py declare_parameter
    默认值已对齐。
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_car_control')
    perception_config = os.path.join(pkg_dir, 'config', 'perception_params.yaml')
    control_config = os.path.join(pkg_dir, 'config', 'control_params.yaml')

    # --- launch 参数（与原版一致）---
    cluster_radius_arg = DeclareLaunchArgument(
        'cluster_radius', default_value='0.25',
        description='聚类半径阈值（米）')
    min_points_arg = DeclareLaunchArgument(
        'min_points', default_value='1',
        description='聚类最小点数')
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping', default_value='False',
        description='是否启用建图模式')
    show_window_arg = DeclareLaunchArgument(
        'show_window', default_value='True',
        description='是否显示 matplotlib 可视化窗口')

    # ====================================================================
    # 1. 感知节点（简单聚类版，加载 YAML）
    # ====================================================================
    perception_node = Node(
        package='my_car_control',
        executable='perception_node_simple_cluster',
        name='perception_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            perception_config,
            {
                'cluster_radius': LaunchConfiguration('cluster_radius'),
                'min_points': LaunchConfiguration('min_points'),
                'enable_mapping': LaunchConfiguration('enable_mapping'),
                'show_window': LaunchConfiguration('show_window'),
            }
        ],
    )

    # ====================================================================
    # 2. 视觉识别节点
    # ====================================================================
    image_detector_node = Node(
        package='my_car_control',
        executable='image_detector',
        name='image_detector',
        output='screen',
        emulate_tty=True,
    )

    # ====================================================================
    # 3. 控制节点（加载 control_params.yaml）【2025 改造修复点】
    # ====================================================================
    control_node = Node(
        package='my_car_control',
        executable='control_node',
        name='control_node',
        output='screen',
        emulate_tty=True,
        parameters=[control_config],
    )

    return LaunchDescription([
        cluster_radius_arg,
        min_points_arg,
        enable_mapping_arg,
        show_window_arg,
        perception_node,
        image_detector_node,
        control_node,
    ])
