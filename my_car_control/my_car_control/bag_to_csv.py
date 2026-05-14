#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_to_csv - 离线 rosbag2 → TrackMemory CSV 转换器

目的：
    把 ros2 bag record 录的 bag 文件转成跟 track_memory_recorder 输出
    一模一样格式的 CSV，这样：
      1. 你录一次 bag，可以反复离线分析；
      2. 改了感知/控制算法后，喂同一段 bag，对比新旧 /target 输出差异；
      3. 不用每次都跑到车上才能调代码。

使用：
    # 录的时候（在车上）
    ros2 bag record /scan /image_raw /image_detection /odom_combined \\
        /target /teleop_cmd_vel /tf /tf_static -o my_run_bag

    # 离线转 CSV（任何装了 ROS2 的环境都行，不需要车）
    ros2 run my_car_control bag_to_csv my_run_bag/
    # 或直接 python：
    python3 bag_to_csv.py my_run_bag/

    # 默认输出到 my_run_bag.csv，可以用 -o 指定
    python3 bag_to_csv.py my_run_bag/ -o /tmp/foo.csv

    # 用 track_memory_viewer 看
    python3 track_memory_viewer.py my_run_bag.csv

为什么要复刻 recorder 的逻辑而不是直接用 ros2 bag play + recorder?
    bag play 是按 wall clock 回放的，1 分钟 bag 要 1 分钟。这个脚本一秒
    就转完，迭代更快。两者输出的 CSV 列一致，viewer 通用。

依赖：
    需要 ROS2 Python 接口能 import:
      - rosbag2_py
      - rclpy.serialization
      - nav_msgs, sensor_msgs, std_msgs, geometry_msgs
    Ubuntu 22.04 + ROS Humble 默认安装都有。
    Windows 上跑不了 —— bag 要先传到 Linux/WSL/车上再转。
"""

import argparse
import csv
import math
import os
import sys
from datetime import datetime


# -----------------------------------------------------------------------------
# 与 track_memory_recorder.py 完全一致的模式映射（修改时务必两边同步）
# -----------------------------------------------------------------------------
MODE_MAP = {
    -10000.0: 'YELLOW_CURVE',
    -20000.0: 'RIGHT_ANGLE',
    -30000.0: 'RIGHT_CORRECTION',
    -40000.0: 'KEEP_LAST',
    -99999.0: 'TRAFFIC_LIGHT',
    -88888.0: 'PARK_ZONE',
}


def classify_target(target_x: float) -> str:
    """根据 /target 的第一个字段判断当前是什么模式。"""
    if target_x in MODE_MAP:
        return MODE_MAP[target_x]
    for magic, name in MODE_MAP.items():
        if abs(target_x - magic) < 0.5:
            return name
    return 'NORMAL'


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元数 → yaw（绕 Z 轴），单位弧度。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# -----------------------------------------------------------------------------
# Bag 读取主逻辑
# -----------------------------------------------------------------------------
TOPICS_OF_INTEREST = [
    '/target',           # 主时钟
    '/odom_combined',    # 位姿和速度
    '/teleop_cmd_vel',   # 控制指令
    '/image_detection',  # 视觉检测
    '/scan',             # 激光点数（仅统计）
]


def detect_storage_id(bag_path: str) -> str:
    """根据 bag 目录里的文件自动识别 storage 类型。"""
    if not os.path.isdir(bag_path):
        raise FileNotFoundError(f'bag 路径不存在或不是目录: {bag_path}')
    files = os.listdir(bag_path)
    if any(f.endswith('.db3') for f in files):
        return 'sqlite3'
    if any(f.endswith('.mcap') for f in files):
        return 'mcap'
    # 也支持直接传 .db3/.mcap 文件路径，但通常 bag 是目录
    raise RuntimeError(
        f'在 {bag_path} 里找不到 .db3 或 .mcap 文件，'
        f'确认这是 ros2 bag record 的输出目录'
    )


def open_reader(bag_path: str):
    """打开 bag reader，返回 (reader, type_map)。
    依赖检查已在 convert() 顶层做过，这里直接 import。"""
    import rosbag2_py

    storage_id = detect_storage_id(bag_path)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    return reader, type_map


def get_message_class(type_name: str):
    """通过 rosidl 名字（如 'std_msgs/msg/Float32MultiArray'）拿消息类。
    依赖检查已在 convert() 顶层做过。"""
    from rosidl_runtime_py.utilities import get_message
    return get_message(type_name)


def convert(bag_path: str, output_csv: str, verbose: bool = True):
    """主转换流程。"""
    # 统一的 ROS 依赖检查（移到函数开头，比中途崩了报错友好）
    try:
        import rosbag2_py  # noqa: F401
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message  # noqa: F401
    except ImportError as e:
        sys.stderr.write(
            '\nERROR: 缺少 ROS2 Python 依赖 (%s)。\n' % e.name +
            '这个脚本必须在 ROS2 环境下运行（车上 / Linux + ROS Humble / WSL）。\n'
            '请先执行: source /opt/ros/humble/setup.bash\n'
            '在 Windows 上跑不了——bag 要先传到有 ROS 的机器上再转。\n\n'
        )
        sys.exit(2)

    reader, type_map = open_reader(bag_path)

    # 报告 bag 里有哪些话题
    if verbose:
        print(f'bag 路径: {bag_path}')
        print(f'话题列表:')
        for name, type_name in sorted(type_map.items()):
            mark = '✓' if name in TOPICS_OF_INTEREST else ' '
            print(f'  {mark} {name:30s}  {type_name}')

    missing = [t for t in TOPICS_OF_INTEREST if t not in type_map]
    if missing:
        sys.stderr.write(
            f'\n警告: bag 里缺以下话题，对应字段在 CSV 里会是 NaN/-1:\n'
            f'  {missing}\n\n'
        )

    # 预先缓存最近一次值（recorder 是同样逻辑）
    latest = {
        '/odom_combined': None,
        '/teleop_cmd_vel': None,
        '/image_detection': None,
        '/scan_pt_count': -1,
    }
    cumulative_s = 0.0
    last_xy = None
    t0_ns = None
    frame = 0

    out_dir = os.path.dirname(output_csv)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    with open(output_csv, 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow([
            'frame', 't', 's',
            'x', 'y', 'yaw', 'v',
            'target_x', 'target_y', 'target_mode', 'target_raw_x',
            'cmd_speed', 'cmd_steer',
            'red_light', 'ab_sign', 'yellow_line', 'ab_direction', 'blue_cones',
            'scan_point_count',
        ])

        msg_counter = {t: 0 for t in TOPICS_OF_INTEREST}

        while reader.has_next():
            topic, raw, t_ns = reader.read_next()
            if topic not in TOPICS_OF_INTEREST:
                continue
            msg_counter[topic] += 1
            msg_class = get_message_class(type_map[topic])
            msg = deserialize_message(raw, msg_class)

            if topic == '/odom_combined':
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                v = math.hypot(vx, vy)
                if last_xy is not None:
                    dx = x - last_xy[0]
                    dy = y - last_xy[1]
                    ds = math.hypot(dx, dy)
                    if ds < 0.5:
                        cumulative_s += ds
                last_xy = (x, y)
                latest['/odom_combined'] = {'x': x, 'y': y, 'yaw': yaw, 'v': v}

            elif topic == '/teleop_cmd_vel':
                latest['/teleop_cmd_vel'] = {
                    'speed': msg.linear.x,
                    'steer': msg.angular.z,
                }

            elif topic == '/image_detection':
                data = list(msg.data) if msg.data else []
                while len(data) < 5:
                    data.append(-1)
                latest['/image_detection'] = data[:5]

            elif topic == '/scan':
                rng_min = msg.range_min
                rng_max = msg.range_max
                # 兼容老 bag 里可能没设 range_min/max 的情况
                if rng_min <= 0:
                    rng_min = 0.01
                cnt = sum(1 for r in msg.ranges if rng_min <= r <= rng_max)
                latest['/scan_pt_count'] = cnt

            elif topic == '/target':
                if not msg.data or len(msg.data) < 2:
                    continue
                if t0_ns is None:
                    t0_ns = t_ns
                t_sec = (t_ns - t0_ns) * 1e-9

                tx_raw = float(msg.data[0])
                ty_raw = float(msg.data[1])
                mode = classify_target(tx_raw)

                od = latest['/odom_combined']
                if od is None:
                    ox = oy = oyaw = ov = float('nan')
                else:
                    ox, oy, oyaw, ov = od['x'], od['y'], od['yaw'], od['v']

                cm = latest['/teleop_cmd_vel']
                if cm is None:
                    cs = cst = float('nan')
                else:
                    cs, cst = cm['speed'], cm['steer']

                im = latest['/image_detection']
                if im is None:
                    rl = ab = yl = abd = bc = -1
                else:
                    rl, ab, yl, abd, bc = im

                if mode == 'NORMAL':
                    tx, ty = tx_raw, ty_raw
                else:
                    tx, ty = float('nan'), float('nan')

                w.writerow([
                    frame,
                    round(t_sec, 4),
                    round(cumulative_s, 4),
                    round(ox, 4) if not math.isnan(ox) else 'nan',
                    round(oy, 4) if not math.isnan(oy) else 'nan',
                    round(oyaw, 4) if not math.isnan(oyaw) else 'nan',
                    round(ov, 4) if not math.isnan(ov) else 'nan',
                    round(tx, 4) if not math.isnan(tx) else 'nan',
                    round(ty, 4) if not math.isnan(ty) else 'nan',
                    mode,
                    round(tx_raw, 4),
                    round(cs, 4) if not math.isnan(cs) else 'nan',
                    round(cst, 4) if not math.isnan(cst) else 'nan',
                    rl, ab, yl, abd, bc,
                    latest['/scan_pt_count'],
                ])
                frame += 1
                if verbose and frame % 200 == 0:
                    print(f'  ... 已处理 {frame} 帧 /target')

    if verbose:
        print('-' * 60)
        print(f'输出: {output_csv}')
        print(f'/target 总帧数: {frame}')
        print(f'各话题消息数:')
        for t in TOPICS_OF_INTEREST:
            print(f'  {t:25s}  {msg_counter[t]}')
        print('-' * 60)
        print('用 track_memory_viewer 可视化：')
        print(f'  python3 track_memory_viewer.py {output_csv}')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='rosbag2 → TrackMemory CSV 离线转换器'
    )
    parser.add_argument('bag', help='ros2 bag record 输出的目录路径')
    parser.add_argument('-o', '--output',
                        help='CSV 输出路径，默认 <bag>.csv')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='不打印进度')
    parsed = parser.parse_args(args=args)

    bag = os.path.abspath(parsed.bag.rstrip('/').rstrip('\\'))
    if parsed.output:
        out = parsed.output
    else:
        out = bag + '.csv'

    convert(bag, out, verbose=not parsed.quiet)


if __name__ == '__main__':
    main()
