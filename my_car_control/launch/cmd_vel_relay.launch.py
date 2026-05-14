#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav2 速度命令转发节点启动文件
将 Nav2 发布的 /cmd_vel_nav 转发到 /teleop_cmd_vel
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""
    
    # 速度命令转发节点
    cmd_vel_relay_node = Node(
        package='my_car_control',
        executable='cmd_vel_relay_node',
        name='cmd_vel_relay_node',
        output='screen',
        parameters=[]
    )
    
    return LaunchDescription([
        cmd_vel_relay_node,
    ])



