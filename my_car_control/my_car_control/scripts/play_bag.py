#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
回放bag文件脚本
用于在室外复现室内录制的小车动作行为
"""

import sys
import os
import subprocess
import argparse

def main():
    parser = argparse.ArgumentParser(description='回放bag文件，复现小车运行数据')
    parser.add_argument(
        'bag_file',
        type=str,
        help='要回放的bag文件路径'
    )
    parser.add_argument(
        '--topics',
        nargs='+',
        default=None,
        help='要回放的话题列表（默认：回放所有话题）'
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=1.0,
        help='回放速率（默认：1.0，即正常速度）'
    )
    parser.add_argument(
        '--loop',
        action='store_true',
        help='循环回放'
    )
    parser.add_argument(
        '--start-offset',
        type=float,
        default=0.0,
        help='从bag文件的第几秒开始回放（默认：0.0）'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=None,
        help='回放时长（秒），不指定则回放整个bag文件'
    )
    parser.add_argument(
        '--remap',
        nargs='+',
        default=[],
        help='话题重映射，格式：old_topic:=new_topic（例如：/scan:=/scan_playback）'
    )
    
    args = parser.parse_args()
    
    # 检查bag文件是否存在
    if not os.path.exists(args.bag_file):
        print(f'错误: bag文件不存在: {args.bag_file}')
        sys.exit(1)
    
    # 构建ros2 bag play命令
    cmd = ['ros2', 'bag', 'play']
    cmd.append(args.bag_file)
    
    if args.topics:
        cmd.extend(['--topics'] + args.topics)
    
    if args.rate != 1.0:
        cmd.extend(['--rate', str(args.rate)])
    
    if args.loop:
        cmd.append('--loop')
    
    if args.start_offset > 0:
        cmd.extend(['--start-offset', str(args.start_offset)])
    
    if args.duration:
        cmd.extend(['--duration', str(args.duration)])
    
    if args.remap:
        cmd.extend(['--remap'] + args.remap)
    
    print('=' * 80)
    print('开始回放bag文件')
    print('=' * 80)
    print(f'bag文件: {args.bag_file}')
    if args.topics:
        print(f'回放话题: {", ".join(args.topics)}')
    else:
        print('回放话题: 所有话题')
    print(f'回放速率: {args.rate}x')
    if args.loop:
        print('循环回放: 是')
    if args.start_offset > 0:
        print(f'起始偏移: {args.start_offset}秒')
    if args.duration:
        print(f'回放时长: {args.duration}秒')
    if args.remap:
        print(f'话题重映射: {", ".join(args.remap)}')
    print('=' * 80)
    print('\n按 Ctrl+C 停止回放...\n')
    
    try:
        # 执行回放命令
        subprocess.run(cmd, check=True)
        print('\n回放完成！')
    except KeyboardInterrupt:
        print('\n\n回放已停止')
    except subprocess.CalledProcessError as e:
        print(f'\n回放失败: {e}')
        sys.exit(1)

if __name__ == '__main__':
    main()

