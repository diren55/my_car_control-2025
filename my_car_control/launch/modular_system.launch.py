#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块化系统启动文件
- 启动感知节点、控制节点和图像检测节点
- 基于NCSC2024源程序的模块化设计
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
    
    # 声明启动参数
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
    
    # 图像检测参数
    enable_red_detect_arg = DeclareLaunchArgument(
        'enable_red_detect',
        default_value='true',
        description='是否启用红灯检测'
    )
    
    enable_ab_detect_arg = DeclareLaunchArgument(
        'enable_ab_detect',
        default_value='true',
        description='是否启用A/B标识牌检测'
    )
    
    enable_yellow_detect_arg = DeclareLaunchArgument(
        'enable_yellow_detect',
        default_value='false',
        description='是否启用黄线检测'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='是否显示调试窗口'
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
        description='摄像头亮度（-1=自动，0-100）'
    )
    
    # USB摄像头节点（为图像检测提供图像数据）
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
    
    # 感知节点
    perception_node = Node(
        package='my_car_control',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[perception_config],
        remappings=[
            ('/scan', '/scan'),           # 激光雷达话题
            ('/target', '/target'),       # 目标点话题
        ]
    )
    
    # 图像检测节点（红灯、A/B、黄线检测）
    image_detector_node = Node(
        package='my_car_control',
        executable='image_detector',
        name='image_detector',
        output='screen',
        parameters=[{
            'enable_red_detect': LaunchConfiguration('enable_red_detect'),
            'enable_ab_detect': LaunchConfiguration('enable_ab_detect'),
            'enable_yellow_detect': LaunchConfiguration('enable_yellow_detect'),
            'show_debug': LaunchConfiguration('show_debug'),
        }],
        emulate_tty=True
    )
    
    # 控制节点
    control_node = Node(
        package='my_car_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config],
        remappings=[
            ('/target', '/target'),                    # 目标点话题
            ('/encoder_imu_odom', '/odom_combined'),   # 里程计话题（映射到实际话题）
            ('/car_cmd_vel', '/car_cmd_vel'),          # 车辆控制话题
            ('/navi', '/navi'),                        # 导航话题
            ('/start', '/start'),                      # 开始话题
        ]
    )
    
    return LaunchDescription([
        perception_config_arg,
        control_config_arg,
        enable_red_detect_arg,
        enable_ab_detect_arg,
        enable_yellow_detect_arg,
        show_debug_arg,
        video_device_arg,
        brightness_arg,
        usb_cam_node,          # USB摄像头节点（必须先启动，为image_detector提供图像）
        perception_node,
        image_detector_node,   # 图像检测节点（需要摄像头数据）
        control_node,
    ])

