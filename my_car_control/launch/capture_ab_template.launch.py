#!/usr/bin/env python3
"""
AB标志模板拍照工具启动文件
功能：启动摄像头和拍照工具，用于拍摄AB标志模板
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
        
        # 裁剪参数
        DeclareLaunchArgument('crop_bottom', default_value='true'),
        DeclareLaunchArgument('crop_right_ratio', default_value='0.2'),
        
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
                'brightness': LaunchConfiguration('brightness')
            }],
            output='screen'
        ),
        
        # AB模板拍照工具节点
        Node(
            package='my_car_control',
            executable='capture_ab_template',
            name='capture_ab_template',
            parameters=[{
                'blue_h_min': LaunchConfiguration('blue_h_min'),
                'blue_h_max': LaunchConfiguration('blue_h_max'),
                'blue_s_min': LaunchConfiguration('blue_s_min'),
                'blue_s_max': LaunchConfiguration('blue_s_max'),
                'blue_v_min': LaunchConfiguration('blue_v_min'),
                'blue_v_max': LaunchConfiguration('blue_v_max'),
                'crop_bottom': LaunchConfiguration('crop_bottom'),
                'crop_right_ratio': LaunchConfiguration('crop_right_ratio'),
            }],
            output='screen',
            emulate_tty=True
        ),
    ])

