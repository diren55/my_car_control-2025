#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
录制bag文件脚本
用于录制室内跑的小车的所有动作行为
"""

import sys
import os
import subprocess
import argparse
from datetime import datetime

def main():
    parser = argparse.ArgumentParser(description='录制小车运行数据到bag文件')
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help='输出bag文件路径（默认：bags/indoor_run_YYYYMMDD_HHMMSS）'
    )
    parser.add_argument(
        '--topics',
        nargs='+',
        default=[
            '/scan',              # 激光雷达数据（感知节点输入）
            '/odom_combined',     # 里程计数据（两个节点都需要）
            '/target',            # 目标点（感知节点输出，控制节点输入）
            '/teleop_cmd_vel',    # 车辆控制指令（控制节点输出）
            '/navi',              # 导航信号（控制节点输出）
            '/start',             # 开始信号（控制节点输出）
        ],
        help='要录制的话题列表（默认：所有相关话题）'
    )
    parser.add_argument(
        '--duration',
        type=int,
        default=None,
        help='录制时长（秒），不指定则录制到手动停止（Ctrl+C）'
    )
    
    args = parser.parse_args()
    
    # 生成默认输出文件名
    if args.output is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = 'bags'
        os.makedirs(output_dir, exist_ok=True)
        args.output = os.path.join(output_dir, f'indoor_run_{timestamp}')
    
    # 构建ros2 bag record命令
    cmd = ['ros2', 'bag', 'record']
    cmd.extend(args.topics)
    cmd.extend(['-o', args.output])
    
    if args.duration:
        cmd.extend(['--duration', str(args.duration)])
    
    print('=' * 80)
    print('开始录制bag文件')
    print('=' * 80)
    print(f'输出路径: {args.output}')
    print(f'录制话题: {", ".join(args.topics)}')
    if args.duration:
        print(f'录制时长: {args.duration}秒')
    else:
        print('录制时长: 手动停止（按Ctrl+C停止）')
    print('=' * 80)
    print('\n按 Ctrl+C 停止录制...\n')
    
    try:
        # 执行录制命令
        subprocess.run(cmd, check=True)
        print(f'\n录制完成！bag文件保存在: {args.output}')
    except KeyboardInterrupt:
        print('\n\n录制已停止')
        print(f'bag文件保存在: {args.output}')
    except subprocess.CalledProcessError as e:
        print(f'\n录制失败: {e}')
        sys.exit(1)

if __name__ == '__main__':
    main()

