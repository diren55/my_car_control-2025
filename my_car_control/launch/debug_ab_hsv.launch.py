#!/usr/bin/env python3
"""
AB标识检测HSV调试工具启动文件
功能：启动摄像头和AB标识检测HSV调试工具，方便调节蓝色HSV参数
显示窗口：
1. AB标识-原始画面：裁剪后的检测区域
2. AB标识-HSV掩码：蓝色掩码（黑白显示）
3. AB标识-检测结果：检测结果和参数信息
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('brightness', default_value='50'),
        
        # HSV阈值参数（可调）
        DeclareLaunchArgument('blue_h_min', default_value='105'),
        DeclareLaunchArgument('blue_h_max', default_value='140'),
        DeclareLaunchArgument('blue_s_min', default_value='59'),
        DeclareLaunchArgument('blue_s_max', default_value='193'),
        DeclareLaunchArgument('blue_v_min', default_value='20'),
        DeclareLaunchArgument('blue_v_max', default_value='255'),
        
        # 圆环检测参数
        DeclareLaunchArgument('use_ring_detection', default_value='true'),
        DeclareLaunchArgument('ring_min_radius', default_value='30'),
        DeclareLaunchArgument('ring_max_radius', default_value='200'),
        
        # 摄像头节点
        Node(
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
        ),
        
        # 图像检测节点（只启用AB检测，显示HSV调试窗口）
        Node(
            package='my_car_control',
            executable='image_detector',
            name='image_detector',
            parameters=[{
                'enable_red_detect': False,
                'enable_ab_detect': True,
                'enable_yellow_detect': False,
                'enable_ab_parking_detect': False,
                'show_debug': True,
                'show_ab_only': True,  # 只显示AB检测窗口
                'show_ab_parking_only': False,
                'show_red_light_only': False,
                'save_debug_image': False,
                # HSV阈值参数（可调）
                'blue_h_min': LaunchConfiguration('blue_h_min'),
                'blue_h_max': LaunchConfiguration('blue_h_max'),
                'blue_s_min': LaunchConfiguration('blue_s_min'),
                'blue_s_max': LaunchConfiguration('blue_s_max'),
                'blue_v_min': LaunchConfiguration('blue_v_min'),
                'blue_v_max': LaunchConfiguration('blue_v_max'),
                # 圆环检测参数
                'use_ring_detection': LaunchConfiguration('use_ring_detection'),
                'ring_min_radius': LaunchConfiguration('ring_min_radius'),
                'ring_max_radius': LaunchConfiguration('ring_max_radius'),
                # 其他参数
                'judge_threshold': 0.667,
            }],
            output='screen',
            emulate_tty=True
        ),
    ])

