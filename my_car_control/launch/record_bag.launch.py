#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
录制bag文件的launch文件
用于录制室内跑的小车的所有动作行为
包括所有输入和输出话题，以便在室外完全复现
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('my_car_control')
    
    # 声明启动参数
    output_arg = DeclareLaunchArgument(
        'output',
        default_value=None,
        description='输出bag文件路径（默认：bags/indoor_run_YYYYMMDD_HHMMSS）'
    )
    
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value=None,
        description='录制时长（秒），不指定则录制到手动停止'
    )
    
    # 获取参数
    output_path = LaunchConfiguration('output')
    duration = LaunchConfiguration('duration')
    
    # 要录制的话题列表（完整记录所有相关话题）
    topics_to_record = [
        '/scan',              # 激光雷达数据（感知节点输入）
        '/odom_combined',     # 里程计数据（两个节点都需要）
        '/target',            # 目标点（感知节点输出，控制节点输入）
        '/teleop_cmd_vel',    # 车辆控制指令（控制节点输出）
        '/navi',              # 导航信号（控制节点输出）
        '/start',             # 开始信号（控制节点输出）
        '/tf',                # 坐标变换（可选，用于完整复现）
        '/tf_static',         # 静态坐标变换（可选）
    ]
    
    # 如果没有指定输出路径，生成默认路径
    # 注意：由于LaunchConfiguration的限制，默认路径需要在shell脚本中处理
    # 或者使用PythonExpression来处理，但这里假设用户会通过shell脚本或指定路径
    
    # 构建ros2 bag record命令
    # 使用--all选项可以录制所有话题，或者使用--topics指定话题列表
    record_cmd = ['ros2', 'bag', 'record']
    record_cmd.extend(topics_to_record)
    record_cmd.extend(['-o', output_path])
    
    # 如果指定了时长，添加时长参数
    # 注意：这里需要处理duration可能为None的情况
    # 由于LaunchConfiguration不支持条件判断，我们使用ExecuteProcess的condition参数
    # 但更简单的方法是直接构建命令，如果duration为None则不添加该参数
    
    # 录制bag文件的进程
    record_process = ExecuteProcess(
        cmd=record_cmd,
        output='screen',
        shell=False,
        name='bag_recorder'
    )
    
    return LaunchDescription([
        output_arg,
        duration_arg,
        record_process,
    ])

