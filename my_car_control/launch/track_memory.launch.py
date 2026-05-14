# -*- coding: utf-8 -*-
"""
track_memory.launch.py

启动 TrackMemoryRecorder 这一个节点。它只订阅、不发布、不控制车。
设计上可以与任意主 launch（full_system / modular_system / perception_control）
并行启动：

    # 终端 1：原车主流程
    ros2 launch my_car_control full_system.launch.py

    # 终端 2：旁路记录
    ros2 launch my_car_control track_memory.launch.py

或者直接 ros2 run：

    ros2 run my_car_control track_memory_recorder

输出文件路径可以通过参数修改：

    ros2 launch my_car_control track_memory.launch.py \
        output_dir:=/home/davinci-mini/track_memory_demo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir = LaunchConfiguration('output_dir')
    file_prefix = LaunchConfiguration('file_prefix')
    record_scan_stats = LaunchConfiguration('record_scan_stats')

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value='~/track_memory',
            description='CSV 文件输出目录'
        ),
        DeclareLaunchArgument(
            'file_prefix',
            default_value='track_memory',
            description='CSV 文件名前缀'
        ),
        DeclareLaunchArgument(
            'record_scan_stats',
            default_value='true',
            description='是否记录每帧 /scan 的有效点数'
        ),
        Node(
            package='my_car_control',
            executable='track_memory_recorder',
            name='track_memory_recorder',
            output='screen',
            parameters=[{
                'output_dir': output_dir,
                'file_prefix': file_prefix,
                'record_scan_stats': record_scan_stats,
            }],
        ),
    ])
