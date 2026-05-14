#!/usr/bin/env python3
"""
ROS2 Launch文件 - 统一启动图像检测和控制节点
功能: 图像检测（红灯、A/B标识、黄线）+ 控制执行（停车逻辑）
使用方法: ros2 launch my_car_control detection_control.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # ========== 参数声明 ==========
    
    # 摄像头参数
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='摄像头设备路径'
    )
    
    brightness_arg = DeclareLaunchArgument(
        'brightness',
        default_value='30',
        description='摄像头亮度（-1=自动，0-100，值越小越暗）'
    )
    
    # 图像检测参数
    enable_red_arg = DeclareLaunchArgument(
        'enable_red_detect', 
        default_value='true',
        description='是否启用红灯检测'
    )
    
    enable_ab_arg = DeclareLaunchArgument(
        'enable_ab_detect', 
        default_value='true',
        description='是否启用A/B标识牌检测'
    )
    
    enable_yellow_arg = DeclareLaunchArgument(
        'enable_yellow_detect', 
        default_value='false',
        description='是否启用黄线检测'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug', 
        default_value='false',
        description='是否显示调试窗口'
    )
    
    yellow_stop_height_arg = DeclareLaunchArgument(
        'yellow_stop_height',
        default_value='0.5',
        description='黄线停车高度阈值'
    )
    
    red_filter_x_arg = DeclareLaunchArgument(
        'red_filter_x',
        default_value='320',
        description='红灯x坐标过滤阈值'
    )
    
    # ========== 节点定义 ==========
    
    # 0. USB摄像头节点（可选）
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'camera_frame_id': 'camera',
            'framerate': 30.0,
            'brightness': LaunchConfiguration('brightness')  # 亮度参数（-1=自动，0-100）
        }],
        output='screen'
    )
    
    # 1. 图像检测节点
    image_detector_node = Node(
        package='my_car_control',
        executable='image_detector',
        name='image_detector',
        parameters=[{
            'enable_red_detect': LaunchConfiguration('enable_red_detect'),
            'enable_ab_detect': LaunchConfiguration('enable_ab_detect'),
            'enable_yellow_detect': LaunchConfiguration('enable_yellow_detect'),
            'show_debug': LaunchConfiguration('show_debug'),
            'yellow_stop_height': LaunchConfiguration('yellow_stop_height'),
            'red_filter_x': LaunchConfiguration('red_filter_x')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 2. 控制节点（订阅图像检测结果，执行停车逻辑）
    control_node = Node(
        package='my_car_control',
        executable='control_node',
        name='control_node',
        output='screen',
        emulate_tty=True
    )
    
    # ========== 返回LaunchDescription ==========
    
    return LaunchDescription([
        # 参数声明
        video_device_arg,
        brightness_arg,
        enable_red_arg,
        enable_ab_arg,
        enable_yellow_arg,
        show_debug_arg,
        yellow_stop_height_arg,
        red_filter_x_arg,
        
        # 节点
        usb_cam_node,
        image_detector_node,
        control_node,
    ])

