#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NPU AB标志识别 + AB车库停车
功能：
1. 使用NPU识别AB标志（best.om）
2. 识别到AB标志后向前走到黄线识别
3. 然后停到AB库（检测竖线，根据A/B决定转向方向）
4. 识别到红绿灯则停下来

使用方法：
ros2 launch my_car_control npu_ab_parking.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 设置昇腾NPU库路径（在函数内部设置，影响后续节点）
    ascend_lib_paths = [
        '/usr/local/Ascend/ascend-toolkit/7.0.RC1/aarch64-linux/devlib',
        '/usr/local/Ascend/ascend-toolkit/7.0.RC1/atc/lib64',
        '/usr/local/Ascend/driver/lib64',
        '/usr/local/Ascend/add-ons',
    ]
    
    # 检查并构建LD_LIBRARY_PATH
    existing_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_paths = [p for p in ascend_lib_paths if os.path.exists(p)]
    if new_paths:
        new_ld_path = ':'.join(new_paths)
        if existing_ld_path:
            final_ld_path = f"{new_ld_path}:{existing_ld_path}"
        else:
            final_ld_path = new_ld_path
        # 直接设置环境变量（会传递给所有子进程）
        os.environ['LD_LIBRARY_PATH'] = final_ld_path
        print(f"设置LD_LIBRARY_PATH: {final_ld_path}")
    
    return LaunchDescription([
        
        # 参数声明
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('brightness', default_value='50'),
        DeclareLaunchArgument('show_debug', default_value='true'),
        
        # NPU模型参数
        DeclareLaunchArgument('ab_model', default_value='/home/davinci-mini/best.om'),
        DeclareLaunchArgument('redlight_model', default_value='/home/davinci-mini/redlight.om'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('enable_redlight', default_value='true'),  # 是否检测红绿灯
        
        # AB车库停车参数
        DeclareLaunchArgument('ab_target', default_value='AUTO'),  # A/B/AUTO - 目标停车位置（AUTO=自动匹配识别结果）
        DeclareLaunchArgument('ab_parking_turn_angle', default_value='45'),
        DeclareLaunchArgument('ab_parking_forward_time', default_value='0.8'),
        DeclareLaunchArgument('ab_parking_turn_time', default_value='0.8'),
        DeclareLaunchArgument('ab_parking_min_pixels', default_value='6000'),
        DeclareLaunchArgument('yellow_stop_height', default_value='0.65'),
        DeclareLaunchArgument('base_speed', default_value='25'),
        
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
        
        # NPU AB标志检测节点（替代image_detector）
        # 注意：LD_LIBRARY_PATH已在函数开始处通过os.environ设置，会传递给所有子进程
        # 使用启动器自动选择正确的Python版本
        Node(
            package='my_car_control',
            executable='npu_ab_detector_node_launcher',  # 使用启动器，自动选择Python 3.9
            name='npu_ab_detector_node',
            parameters=[{
                'ab_model': LaunchConfiguration('ab_model'),
                'device_id': LaunchConfiguration('device_id'),
                'input_shape': [640, 640],
                'conf_thres': 0.4,
                'iou_thres': 0.5,
                'ab_classes': 'A,B',
                'enable_redlight': LaunchConfiguration('enable_redlight'),
                'redlight_model': LaunchConfiguration('redlight_model'),
                'redlight_classes': 'red,green',
                'camera_topic': '/image_raw',
                'enable_yellow_detect': True,  # 启用黄线检测
                'enable_ab_parking_detect': True,  # 启用AB车库方向检测
                'yellow_stop_height': LaunchConfiguration('yellow_stop_height'),
                'ab_parking_min_pixels': LaunchConfiguration('ab_parking_min_pixels'),
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
                'window_name': 'NPU AB车库停车测试 - Vision Debug'
            }],
            output='screen',
            emulate_tty=True
        ),
    ])

