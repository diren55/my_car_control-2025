#!/usr/bin/env python3
"""
A/B标识牌测试 - 车辆行驶看到A/B标识停到对应位置
功能：车辆正常行驶 → 识别到A/B标识 → 转向进入对应区域 → 停车
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
        DeclareLaunchArgument('ab_target', default_value='AUTO'),
        DeclareLaunchArgument('ab_forward_time', default_value='2.0'),
        DeclareLaunchArgument('ab_turn_angle', default_value='45'),
        DeclareLaunchArgument('enable_ab_detect', default_value='true'),
        DeclareLaunchArgument('show_debug', default_value='false'),
        DeclareLaunchArgument('save_debug_image', default_value='false'),
        # HSV阈值参数（用于调整蓝色检测）
        DeclareLaunchArgument('blue_h_min', default_value='100'),
        DeclareLaunchArgument('blue_h_max', default_value='130'),
        DeclareLaunchArgument('blue_s_min', default_value='90'),
        DeclareLaunchArgument('blue_s_max', default_value='255'),
        DeclareLaunchArgument('blue_v_min', default_value='50'),
        DeclareLaunchArgument('blue_v_max', default_value='255'),
        DeclareLaunchArgument('judge_threshold', default_value='0.556'),  # A/B判断阈值（5/9≈0.556，越小越严格）
        DeclareLaunchArgument('first_kernel_size', default_value='4'),  # 初始kernel_size
        
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
                'enable_red_detect': False,
                'enable_ab_detect': LaunchConfiguration('enable_ab_detect'),
                'enable_yellow_detect': False,
                'show_debug': LaunchConfiguration('show_debug'),
                'save_debug_image': LaunchConfiguration('save_debug_image'),
                'red_filter_x': 500,
                'yellow_stop_height': 0.5,
                # HSV阈值参数（可调）
                'blue_h_min': LaunchConfiguration('blue_h_min'),
                'blue_h_max': LaunchConfiguration('blue_h_max'),
                'blue_s_min': LaunchConfiguration('blue_s_min'),
                'blue_s_max': LaunchConfiguration('blue_s_max'),
                'blue_v_min': LaunchConfiguration('blue_v_min'),
                'blue_v_max': LaunchConfiguration('blue_v_max'),
                'judge_threshold': LaunchConfiguration('judge_threshold'),
                'first_kernel_size': LaunchConfiguration('first_kernel_size'),
            }],
            output='screen',
            emulate_tty=True
        ),
        
        # 视觉控制测试节点（A/B标识模式）
        Node(
            package='my_car_control',
            executable='vision_control_test',
            name='vision_control_test',
            parameters=[{
                'test_mode': 'ab_sign',
                'ab_target': LaunchConfiguration('ab_target'),
                'ab_forward_time': LaunchConfiguration('ab_forward_time'),
                'ab_turn_angle': LaunchConfiguration('ab_turn_angle'),
                'red_light_stop_time': 3.0,
                'yellow_stop_time': 5.0
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
                'window_name': 'A/B标识测试 - Vision Debug'
            }],
            output='screen',
            emulate_tty=True
        ),
    ])

