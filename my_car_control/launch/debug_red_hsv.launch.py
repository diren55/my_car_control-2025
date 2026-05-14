#!/usr/bin/env python3
"""
红灯检测HSV调试工具启动文件
功能：启动摄像头和HSV调试工具，方便调节红灯检测HSV参数
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
        
        # HSV调试工具节点
        Node(
            package='my_car_control',
            executable='debug_red_hsv',
            name='debug_red_hsv',
            output='screen',
            emulate_tty=True
        ),
    ])

