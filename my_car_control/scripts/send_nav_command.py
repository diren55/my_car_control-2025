#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
发送导航测试命令的交互式脚本
在MobaXterm中，可以在另一个终端运行此脚本来发送命令
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time

class CommandSender(Node):
    def __init__(self):
        super().__init__('nav_command_sender')
        self.pub = self.create_publisher(String, '/navigation_test_command', 10)
        time.sleep(0.5)  # 等待发布者就绪
    
    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        self.get_logger().info(f"已发送命令: {cmd}")

def main():
    rclpy.init()
    sender = CommandSender()
    
    print("\n" + "="*60)
    print("导航命令发送器")
    print("="*60)
    print("\n输入命令（单个字母或数字），'q' 退出\n")
    
    try:
        while True:
            cmd = input("命令> ").strip()
            if not cmd:
                continue
            if cmd.lower() == 'q':
                break
            sender.send_command(cmd)
            rclpy.spin_once(sender, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

