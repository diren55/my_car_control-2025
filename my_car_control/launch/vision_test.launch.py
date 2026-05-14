#!/usr/bin/env python3
"""
视觉控制测试Launch文件 - 独立调试各项视觉功能
功能：启动图像检测节点 + 视觉控制测试节点
使用方法：
  # 测试红绿灯停车
  ros2 launch my_car_control vision_test.launch.py test_mode:=red_light

  # 测试A/B标识识别停车
  ros2 launch my_car_control vision_test.launch.py test_mode:=ab_sign ab_target:=A

  # 测试黄线停车
  ros2 launch my_car_control vision_test.launch.py test_mode:=yellow_line

  # 测试所有功能
  ros2 launch my_car_control vision_test.launch.py test_mode:=all
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
        default_value='50',
        description='摄像头亮度（-1=自动，0-100，值越小越暗）'
    )
    
    # 测试模式参数
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='all',
        description='测试模式：all/red_light/ab_sign/yellow_line'
    )
    
    ab_target_arg = DeclareLaunchArgument(
        'ab_target',
        default_value='AUTO',
        description='A/B标识测试目标位置：A/B/AUTO（AUTO=自动匹配识别结果）'
    )
    
    ab_turn_angle_arg = DeclareLaunchArgument(
        'ab_turn_angle',
        default_value='45',
        description='A/B识别后转向角度（A区左转45度，B区右转-45度）'
    )
    
    red_light_stop_time_arg = DeclareLaunchArgument(
        'red_light_stop_time',
        default_value='3.0',
        description='红灯停车时间（秒）'
    )
    
    ab_forward_time_arg = DeclareLaunchArgument(
        'ab_forward_time',
        default_value='2.0',
        description='A/B识别后前进时间（秒）'
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
        default_value='true',
        description='是否启用黄线检测'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='是否显示调试窗口（简单文本显示）'
    )
    
    enable_debug_viewer_arg = DeclareLaunchArgument(
        'enable_debug_viewer',
        default_value='true',
        description='是否启动调试查看器（实时显示画面+调阈值）'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='是否启动摄像头节点（如果已有摄像头节点，设为false）'
    )
    
    # ========== 节点定义 ==========
    
    # 0. USB摄像头节点（可选，如果已有摄像头节点可设为false）
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
            'yellow_stop_height': 0.5,
            'red_filter_x': 500
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 2. 视觉控制测试节点
    vision_control_test_node = Node(
        package='my_car_control',
        executable='vision_control_test',
        name='vision_control_test',
        parameters=[{
            'test_mode': LaunchConfiguration('test_mode'),
            'ab_target': LaunchConfiguration('ab_target'),
            'red_light_stop_time': LaunchConfiguration('red_light_stop_time'),
            'ab_forward_time': LaunchConfiguration('ab_forward_time'),
            'ab_turn_angle': LaunchConfiguration('ab_turn_angle'),
            'yellow_stop_time': 5.0
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 3. 视觉调试查看器节点（实时显示画面+调阈值）
    vision_debug_viewer_node = Node(
        package='my_car_control',
        executable='vision_debug_viewer',
        name='vision_debug_viewer',
        parameters=[{
            'red_filter_x': 500,
            'yellow_stop_height': 0.5,
            'show_detection': True,
            'window_name': 'Vision Debug',
            'red_h_min': 0,
            'red_h_max': 255,
            'red_s_min': 100,
            'red_s_max': 255,
            'red_v_min': 200,
            'red_v_max': 255
        }],
        output='screen',
        emulate_tty=True
    )
    
    # ========== 返回LaunchDescription ==========
    
    return LaunchDescription([
        # 参数声明
        video_device_arg,
        brightness_arg,
        enable_camera_arg,
        test_mode_arg,
        ab_target_arg,
        ab_turn_angle_arg,
        red_light_stop_time_arg,
        ab_forward_time_arg,
        enable_red_arg,
        enable_ab_arg,
        enable_yellow_arg,
        show_debug_arg,
        enable_debug_viewer_arg,
        
        # 节点
        usb_cam_node,
        image_detector_node,
        vision_control_test_node,
        vision_debug_viewer_node,  # 调试查看器（实时显示画面）
    ])

