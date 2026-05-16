#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启用 A/B 投票 + combo 锁定的 launch 文件
================================================================================
与 perception_control.launch.py 的区别：
  1. image_detector 的输出 topic 重映射为 /image_detection_raw
  2. 新增 ab_vote_node 在中间做投票后处理，重新发布 /image_detection
  3. 其它节点（perception, control）一行不改

数据流：
  原版:
    image_detector → /image_detection → control_node

  本 launch:
    image_detector → /image_detection_raw      ┐
                                                ├─→ ab_vote_node → /image_detection → control_node
        (data[1] 原始 A/B)                      ┘  (data[1] 经投票后)
        (其它字段透传)

如果想关闭投票，直接用回原版 perception_control.launch.py 即可。

================================================================================
参数调整：
  在文件顶部 AB_VOTE_PARAMS 字典里改：
    combo_max:   看到一次锁定多少帧（值越大越粘）
    require_min_votes: 报告前要求最少投票数（0=立刻报告）
    enable_logging:    切换时是否打日志（调参期间建议 True）
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# A/B 投票参数（启用本 launch 时的默认值）
AB_VOTE_PARAMS = {
    'combo_max': 10,          # 22 年原值
    'require_min_votes': 0,   # 0 = 立刻报告（与 22 年原版一致）
    'enable_logging': True,   # 调参期建议开
}


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_car_control')
    perception_config = os.path.join(pkg_dir, 'config', 'perception_params.yaml')
    control_config = os.path.join(pkg_dir, 'config', 'control_params.yaml')

    # --- launch 参数（与 perception_control.launch.py 一致）---
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
        description='是否显示matplotlib可视化窗口')

    # ====================================================================
    # 1. 感知节点（简单聚类版，加载 YAML，与 perception_control 一致）
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
    # 2. 视觉识别节点（与 perception_control 一致，但 topic remap）
    #    关键：把 /image_detection 重映射为 /image_detection_raw
    #    这样 ab_vote_node 能接到原始数据，control_node 看不到原始数据
    # ====================================================================
    image_detector_node = Node(
        package='my_car_control',
        executable='image_detector',
        name='image_detector',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/image_detection', '/image_detection_raw'),
        ],
    )

    # ====================================================================
    # 3. A/B 投票后处理节点（新加！）
    #    订阅 /image_detection_raw，发布 /image_detection
    # ====================================================================
    ab_vote_node = Node(
        package='my_car_control',
        executable='ab_vote_node',
        name='ab_vote_node',
        output='screen',
        emulate_tty=True,
        parameters=[AB_VOTE_PARAMS],
    )

    # ====================================================================
    # 4. 控制节点（与 perception_control 一致，不知道有投票节点存在）
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
        ab_vote_node,
        control_node,
    ])
