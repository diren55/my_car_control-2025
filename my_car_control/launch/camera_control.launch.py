#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头参数控制启动文件
功能: 启动摄像头参数实时调整节点
使用方法: ros2 launch my_car_control camera_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========== 参数声明 ==========
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='摄像头设备路径'
    )
    
    publish_image_arg = DeclareLaunchArgument(
        'publish_image',
        default_value='true',
        description='是否发布图像话题'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='图像发布话题名称'
    )
    
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='640',
        description='图像宽度'
    )
    
    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='480',
        description='图像高度'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='帧率'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='是否显示画面窗口'
    )
    
    show_trackbars_arg = DeclareLaunchArgument(
        'show_trackbars',
        default_value='true',
        description='是否显示滑动条'
    )
    
    # ========== 节点定义 ==========
    camera_control_node = Node(
        package='my_car_control',
        executable='camera_control_node',
        name='camera_control_node',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'publish_image': LaunchConfiguration('publish_image'),
            'image_topic': LaunchConfiguration('image_topic'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'fps': LaunchConfiguration('fps'),
            'show_window': LaunchConfiguration('show_window'),
            'show_trackbars': LaunchConfiguration('show_trackbars'),
            # 摄像头硬件参数（-1表示使用默认值）
            'brightness': -1.0,
            'contrast': -1.0,
            'saturation': -1.0,
            'hue': -1.0,
            'gain': -1.0,
            'exposure': -1.0,
            'auto_exposure': -1.0,
            'white_balance': -1.0,
            'auto_white_balance': -1.0,
            'sharpness': -1.0,
            'gamma': -1.0,
            'backlight_compensation': -1.0,
            'power_line_frequency': -1.0,
        }],
        output='screen'
    )
    
    # ========== 返回LaunchDescription ==========
    return LaunchDescription([
        video_device_arg,
        publish_image_arg,
        image_topic_arg,
        frame_width_arg,
        frame_height_arg,
        fps_arg,
        show_window_arg,
        show_trackbars_arg,
        camera_control_node,
    ])

