#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整系统启动文件
- 启动所有必要节点：perception_node + control_node + image_detector + usb_cam
- 一键启动完整比赛流程
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('my_car_control')
    
    # ========== 参数声明 ==========
    
    # 配置文件
    perception_config = LaunchConfiguration('perception_config')
    control_config = LaunchConfiguration('control_config')
    
    perception_config_arg = DeclareLaunchArgument(
        'perception_config',
        default_value=os.path.join(pkg_share, 'config', 'perception_params.yaml'),
        description='感知节点配置文件路径'
    )
    
    control_config_arg = DeclareLaunchArgument(
        'control_config',
        default_value=os.path.join(pkg_share, 'config', 'control_params.yaml'),
        description='控制节点配置文件路径'
    )
    
    # 摄像头参数
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='摄像头设备路径'
    )
    
    brightness_arg = DeclareLaunchArgument(
        'brightness',
        default_value='30',
        description='摄像头亮度'
    )
    
    # 图像检测参数
    enable_yellow_arg = DeclareLaunchArgument(
        'enable_yellow_detect', 
        default_value='true',
        description='是否启用黄线检测（停车任务必须启用）'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug', 
        default_value='false',
        description='是否显示调试窗口'
    )
    
    # ========== 节点定义 ==========
    
    # 1. 感知节点（雷达循迹 + 队友的红灯检测）
    perception_node = Node(
        package='my_car_control',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[perception_config],
        remappings=[
            ('/scan', '/scan'),
            ('/target', '/target'),
        ]
    )
    
    # 2. 控制节点（车辆控制 + 停车逻辑）
    control_node = Node(
        package='my_car_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config],
        remappings=[
            ('/target', '/target'),
            ('/odom_combined', '/odom_combined'),
            ('/teleop_cmd_vel', '/teleop_cmd_vel'),
        ]
    )
    
    # 3. USB摄像头节点
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
            'brightness': LaunchConfiguration('brightness')
        }],
        output='screen'
    )
    
    # 4. 图像检测节点（黄线检测）
    image_detector_node = Node(
        package='my_car_control',
        executable='image_detector',
        name='image_detector',
        parameters=[{
            'enable_red_detect': False,  # 红灯由perception_node检测
            'enable_ab_detect': False,   # 不需要AB识别
            'enable_yellow_detect': LaunchConfiguration('enable_yellow_detect'),
            'show_debug': LaunchConfiguration('show_debug'),
        }],
        output='screen',
        emulate_tty=True
    )
    
    # ========== 返回LaunchDescription ==========
    
    return LaunchDescription([
        # 参数声明
        perception_config_arg,
        control_config_arg,
        video_device_arg,
        brightness_arg,
        enable_yellow_arg,
        show_debug_arg,
        
        # 节点
        perception_node,      # 雷达循迹 + 红灯检测（队友）
        control_node,         # 车辆控制 + 停车逻辑
        usb_cam_node,         # 摄像头
        image_detector_node,  # 黄线检测
    ])

