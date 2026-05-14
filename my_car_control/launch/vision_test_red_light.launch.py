#!/usr/bin/env python3
"""
红绿灯测试 - 车辆行驶看到红灯停车
功能：车辆正常行驶 → 识别到红灯 → 停车3秒 → 继续前进
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
        DeclareLaunchArgument('red_light_stop_time', default_value='3.0'),
        DeclareLaunchArgument('enable_red_detect', default_value='true'),
        DeclareLaunchArgument('red_filter_x', default_value='320'),
        DeclareLaunchArgument('min_red_diameter', default_value='20'),  # 最小红灯直径（像素），直径>=此值才停车
        DeclareLaunchArgument('show_debug', default_value='false'),
        # 速度参数
        DeclareLaunchArgument('base_speed', default_value='25'),      # 基础行驶速度
        DeclareLaunchArgument('stop_speed', default_value='-500'),    # 停车速度
        DeclareLaunchArgument('start_speed', default_value='1500'),   # 速度基准值
        DeclareLaunchArgument('start_theta', default_value='75'),    # 角度基准值
        DeclareLaunchArgument('wheel_align_time', default_value='2.0'),  # 轮子回正时间（秒）
        
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
        
        # 图像检测节点
        Node(
            package='my_car_control',
            executable='image_detector',
            name='image_detector',
            parameters=[{
                'enable_red_detect': LaunchConfiguration('enable_red_detect'),
                'enable_ab_detect': False,
                'enable_yellow_detect': False,
                'show_debug': LaunchConfiguration('show_debug'),
                'show_red_light_only': True,  # 只显示红灯检测窗口
                'red_filter_x': LaunchConfiguration('red_filter_x'),
                'min_red_diameter': LaunchConfiguration('min_red_diameter'),  # 最小红灯直径阈值
                'yellow_stop_height': 0.5
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # 视觉控制测试节点（红绿灯模式）
        Node(
            package='my_car_control',
            executable='vision_control_test',
            name='vision_control_test',
            parameters=[{
                'test_mode': 'red_light',
                'red_light_stop_time': LaunchConfiguration('red_light_stop_time'),
                'ab_target': 'AUTO',
                'ab_forward_time': 2.0,
                'ab_turn_angle': 45,
                'yellow_stop_time': 5.0,
                # 速度参数
                'base_speed': LaunchConfiguration('base_speed'),
                'stop_speed': LaunchConfiguration('stop_speed'),
                'start_speed': LaunchConfiguration('start_speed'),
                'start_theta': LaunchConfiguration('start_theta'),
                'wheel_align_time': LaunchConfiguration('wheel_align_time')  # 轮子回正时间
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
                'window_name': '红绿灯测试 - Vision Debug'
            }],
            output='screen',
            emulate_tty=True
        ),
    ])

