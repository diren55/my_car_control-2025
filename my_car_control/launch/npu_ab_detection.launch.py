#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    ab_model_arg = DeclareLaunchArgument(
        'ab_model',
        default_value='/home/davinci-mini/best.om',
        description='AB标志检测模型路径'
    )
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='昇腾NPU设备ID'
    )
    
    enable_redlight_arg = DeclareLaunchArgument(
        'enable_redlight',
        default_value='false',
        description='是否同时检测红绿灯'
    )
    
    redlight_model_arg = DeclareLaunchArgument(
        'redlight_model',
        default_value='/home/davinci-mini/redlight.om',
        description='红绿灯检测模型路径'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw',
        description='摄像头图像话题名称（与image_detector.py保持一致）'
    )
    
    # NPU AB检测节点
    npu_ab_detector_node = Node(
        package='my_car_control',
        executable='npu_ab_detector_node',
        name='npu_ab_detector_node',
        output='screen',
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
            'camera_topic': LaunchConfiguration('camera_topic'),
        }]
    )
    
    return LaunchDescription([
        ab_model_arg,
        device_id_arg,
        enable_redlight_arg,
        redlight_model_arg,
        camera_topic_arg,
        npu_ab_detector_node,
    ])

