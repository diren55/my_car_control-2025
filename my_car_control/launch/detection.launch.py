#!/usr/bin/env python3
"""
ROS2 Launch文件 - 启动摄像头和图像检测
使用方法: ros2 launch racecar detection.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # ========== 参数声明 ==========
    
    # 摄像头设备参数
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
    
    # 检测开关参数
    enable_red_arg = DeclareLaunchArgument(
        'enable_red', 
        default_value='true',
        description='是否启用红灯检测'
    )
    
    enable_ab_arg = DeclareLaunchArgument(
        'enable_ab', 
        default_value='true',
        description='是否启用A/B标识牌检测'
    )
    
    enable_yellow_arg = DeclareLaunchArgument(
        'enable_yellow', 
        default_value='true',
        description='是否启用黄线检测'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug', 
        default_value='true',
        description='是否显示调试窗口'
    )
    
    # ========== 节点定义 ==========
    
    # 1. USB摄像头节点 (使用您车上已有的usb_cam)
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',  # 与您的camer.launch.py一致
            'camera_frame_id': 'camera',
            'framerate': 30.0,
            'brightness': LaunchConfiguration('brightness')  # 亮度参数（-1=自动，0-100）
        }],
        output='screen'
    )
    
    # 2. 图像检测节点 (新增)
    image_detector_node = Node(
        package='my_car_control',  # 使用my_car_control包
        executable='image_detector',  # 已在setup.py中配置
        name='image_detector',
        parameters=[{
            'enable_red_detect': LaunchConfiguration('enable_red'),
            'enable_ab_detect': LaunchConfiguration('enable_ab'),
            'enable_yellow_detect': LaunchConfiguration('enable_yellow'),
            'show_debug': LaunchConfiguration('show_debug'),
            'yellow_stop_height': 0.5,  # 50%
            'red_filter_x': 500
        }],
        output='screen',
        emulate_tty=True  # 启用终端颜色输出
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
        
        # 节点
        usb_cam_node,
        image_detector_node,
    ])
