#!/usr/bin/env python3
"""
AB车库停车测试
功能：车辆正常行驶 → 识别到AB车库区域（倒T字型） → 根据参数指定方向 → 快速停车

注意：AB方向由参数 ab_target 指定（A/B），而不是通过视觉识别
视觉只用于检测是否到达车库区域（倒T字型）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('brightness', default_value='50'),
        DeclareLaunchArgument('show_debug', default_value='true'),  # 默认开启调试窗口
        
        # AB车库停车参数
        DeclareLaunchArgument('ab_target', default_value='A'),  # A/B/AUTO - 目标停车位置（A=左转，B=右转，AUTO=自动）
        DeclareLaunchArgument('ab_parking_turn_angle', default_value='45'),  # 转向角度
        DeclareLaunchArgument('ab_parking_forward_time', default_value='0.8'),  # 前进时间（秒）- 减少延迟，避免超过停车区域
        DeclareLaunchArgument('ab_parking_turn_time', default_value='0.8'),  # 转向时间（秒）
        DeclareLaunchArgument('ab_parking_min_pixels', default_value='6000'),  # 黄色区域最小白色像素数（只有≥此值才判定为进入停车区）
        DeclareLaunchArgument('base_speed', default_value='25'),  # 基础行驶速度
        
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
        
        # 图像检测节点（启用AB车库检测）
        Node(
            package='my_car_control',
            executable='image_detector',
            name='image_detector',
            parameters=[{
                'enable_red_detect': False,
                'enable_ab_detect': False,
                'enable_yellow_detect': False,
                'enable_ab_parking_detect': True,  # 启用AB车库检测
                'show_debug': LaunchConfiguration('show_debug'),
                'show_ab_parking_only': True,  # 只显示AB车库检测窗口
                'ab_parking_min_pixels': LaunchConfiguration('ab_parking_min_pixels'),  # 竖线最小像素数
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # 视觉控制测试节点（AB车库停车模式）
        Node(
            package='my_car_control',
            executable='vision_control_test',
            name='vision_control_test',
            parameters=[{
                'test_mode': 'ab_parking',  # AB车库停车模式
                'ab_target': LaunchConfiguration('ab_target'),  # AB目标位置（A/B/AUTO）
                'base_speed': LaunchConfiguration('base_speed'),
                'ab_parking_turn_angle': LaunchConfiguration('ab_parking_turn_angle'),
                'ab_parking_forward_time': LaunchConfiguration('ab_parking_forward_time'),
                'ab_parking_turn_time': LaunchConfiguration('ab_parking_turn_time'),
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # 视觉调试查看器（实时显示画面）
        Node(
            package='my_car_control',
            executable='vision_debug_viewer',
            name='vision_debug_viewer',
            parameters=[{
                'show_detection': True,
                'window_name': 'AB车库停车测试 - Vision Debug'
            }],
            output='screen',
            emulate_tty=True
        ),
    ])

