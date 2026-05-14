#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TrackMemoryRecorder - 被动观察者节点

设计目标：
    迁移 2022 蓝谷耐撞王代码的核心调试方法论 ——
    "行驶过程记录各个时刻小车周围关键信息并保存，
     随后在 python 程序中调用并画出指定时刻的环境图。"

行为：
    纯订阅者，不发布任何控制话题。订阅 /odom_combined, /target,
    /teleop_cmd_vel, /image_detection, /scan 五个话题，按 /target
    的触发频率（约 10Hz）汇总一帧快照，写入 CSV。

    本节点对原车控制链路零侵入，可以与 full_system.launch.py 或
    perception_control.launch.py 并行启动，关掉本节点不影响任何控制。

用途：
    1. 第一圈 TrackMemory：未来 steer_memory(s) replay 的数据源。
    2. 调试：用配套的离线 viewer 重画任意时刻的车体状态。
    3. 基线对照：改感知/控制前后，跑同一段 bag，对比 CSV 差异。

输出文件：
    ~/track_memory/track_memory_<YYYYMMDD_HHMMSS>.csv

字段（含义见文件头注释）：
    t, s, x, y, yaw, v,
    target_x, target_y, target_mode, target_reason,
    cmd_speed, cmd_steer,
    red_light, ab_sign, yellow_line, ab_direction,
    scan_point_count

魔法数字到模式名的映射（与 control_node.py 第 644-650 行注释一致）：
    -10000  → YELLOW_CURVE      （黄线弯/扇形无锥桶）
    -20000  → RIGHT_ANGLE       （直角弯/感知失败）
    -30000  → RIGHT_CORRECTION  （三等分点为空，向右修正）
    -40000  → KEEP_LAST         （保持上一状态）
    -99999  → TRAFFIC_LIGHT     （红灯识别区）
    -88888  → PARK_ZONE         （精确停车区）
    其他    → NORMAL            （正常循迹）
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int16MultiArray


# -----------------------------------------------------------------------------
# 魔法数字 → 模式名映射（保持与 control_node.py 一致）
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
    # 对浮点的近似匹配（防止精度问题导致漏判）
    for magic, name in MODE_MAP.items():
        if abs(target_x - magic) < 0.5:
            return name
    return 'NORMAL'


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元数 → yaw（绕 Z 轴），单位弧度。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class TrackMemoryRecorder(Node):
    """被动观察者：把每一帧的"现场状态"汇总成一行 CSV。"""

    def __init__(self):
        super().__init__('track_memory_recorder')

        # ---------- 参数 ----------
        # 输出目录，默认 ~/track_memory
        self.declare_parameter('output_dir', os.path.expanduser('~/track_memory'))
        # CSV 文件名前缀
        self.declare_parameter('file_prefix', 'track_memory')
        # 是否记录 /scan 的点数（轻量，不记录全部点）
        self.declare_parameter('record_scan_stats', True)
        # 心跳日志的输出间隔（秒）
        self.declare_parameter('heartbeat_period', 5.0)

        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.record_scan_stats = self.get_parameter('record_scan_stats').value
        self.heartbeat_period = self.get_parameter('heartbeat_period').value

        # ---------- 内部状态 ----------
        self.t0 = None                # 第一个 /target 到达的时间，作为 t=0
        self.last_odom = None         # {x, y, yaw, v}
        self.last_cmd = None          # {speed, steer}
        self.last_image_det = None    # [red_light, ab_sign, yellow_line, ab_direction]
        self.last_scan_pt_count = -1  # 上一帧 /scan 的有效点数
        self.cumulative_s = 0.0       # 累计弧长（沿 odom 轨迹）
        self.last_xy = None           # 上一帧的 (x, y)，用于增量积累 s
        self.frame_idx = 0
        self.last_heartbeat_t = time.monotonic()

        # ---------- 准备输出文件 ----------
        os.makedirs(self.output_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir, f'{self.file_prefix}_{ts}.csv'
        )
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'frame', 't', 's',
            'x', 'y', 'yaw', 'v',
            'target_x', 'target_y', 'target_mode', 'target_raw_x',
            'cmd_speed', 'cmd_steer',
            'red_light', 'ab_sign', 'yellow_line', 'ab_direction',
            'scan_point_count',
        ])
        self.csv_file.flush()

        self.get_logger().info(f'[TRACK_MEM] 记录到: {self.csv_path}')

        # ---------- QoS 与订阅 ----------
        # 感知/控制都用默认 QoS，这里也用默认即可
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(Odometry, '/odom_combined',
                                 self.cb_odom, qos)
        self.create_subscription(Twist, '/teleop_cmd_vel',
                                 self.cb_cmd_vel, qos)
        self.create_subscription(Int16MultiArray, '/image_detection',
                                 self.cb_image_det, qos)
        if self.record_scan_stats:
            self.create_subscription(LaserScan, '/scan',
                                     self.cb_scan, scan_qos)
        # /target 是触发记录的主时钟
        self.create_subscription(Float32MultiArray, '/target',
                                 self.cb_target, qos)

        # 心跳定时器：即使没有 /target 也能告诉用户"我还活着但没数据"
        self.create_timer(self.heartbeat_period, self.heartbeat)

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        # 线速度模长（前向为主，但保留全速度）
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.hypot(vx, vy)
        # 累计弧长
        if self.last_xy is not None:
            dx = x - self.last_xy[0]
            dy = y - self.last_xy[1]
            ds = math.hypot(dx, dy)
            # 忽略明显跳变（>0.5m 一帧），通常是 odom 跳点
            if ds < 0.5:
                self.cumulative_s += ds
        self.last_xy = (x, y)
        self.last_odom = {'x': x, 'y': y, 'yaw': yaw, 'v': v}

    def cb_cmd_vel(self, msg: Twist):
        # 注意：linear.x 和 angular.z 在这套代码里语义不是标准 m/s + rad/s，
        # 而是控制层自定义的速度档位/舵机角度。记录原值即可，含义留给离线分析。
        self.last_cmd = {'speed': msg.linear.x, 'steer': msg.angular.z}

    def cb_image_det(self, msg: Int16MultiArray):
        # /image_detection 字段顺序（来自 image_detector.py 第 161 行注释）：
        #   [红灯, A/B, 黄线, AB车库方向]
        data = list(msg.data) if msg.data else []
        # 用 -1 填充，保证 CSV 列数稳定
        while len(data) < 4:
            data.append(-1)
        self.last_image_det = data[:4]

    def cb_scan(self, msg: LaserScan):
        # 只统计点数，不存全部点。如果想存全部点请用 ros2 bag record /scan。
        # 这里的"有效点"定义：在 [range_min, range_max] 之间的点。
        rng_min = msg.range_min
        rng_max = msg.range_max
        count = 0
        for r in msg.ranges:
            if rng_min <= r <= rng_max:
                count += 1
        self.last_scan_pt_count = count

    def cb_target(self, msg: Float32MultiArray):
        """/target 是触发记录的主时钟。每来一帧就 dump 一行 CSV。"""
        if not msg.data or len(msg.data) < 2:
            return

        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        target_x_raw = float(msg.data[0])
        target_y_raw = float(msg.data[1])
        mode = classify_target(target_x_raw)

        # 写入一行
        if self.last_odom is None:
            # 还没收到过 odom，写 NaN 占位但不丢这帧
            ox = oy = oyaw = ov = float('nan')
        else:
            ox = self.last_odom['x']
            oy = self.last_odom['y']
            oyaw = self.last_odom['yaw']
            ov = self.last_odom['v']

        if self.last_cmd is None:
            cs = cst = float('nan')
        else:
            cs = self.last_cmd['speed']
            cst = self.last_cmd['steer']

        if self.last_image_det is None:
            rl = ab = yl = abd = -1
        else:
            rl, ab, yl, abd = self.last_image_det

        # 当处于特殊模式时，target_x_raw 是魔法数字而不是真实坐标。
        # CSV 里同时保留"展示用"的 target_x/y（NaN 替代魔法数字）
        # 和"原始" target_raw_x（用于离线分析）。
        if mode == 'NORMAL':
            tx, ty = target_x_raw, target_y_raw
        else:
            tx, ty = float('nan'), float('nan')

        row = [
            self.frame_idx,
            round(t, 4),
            round(self.cumulative_s, 4),
            round(ox, 4) if not math.isnan(ox) else 'nan',
            round(oy, 4) if not math.isnan(oy) else 'nan',
            round(oyaw, 4) if not math.isnan(oyaw) else 'nan',
            round(ov, 4) if not math.isnan(ov) else 'nan',
            round(tx, 4) if not math.isnan(tx) else 'nan',
            round(ty, 4) if not math.isnan(ty) else 'nan',
            mode,
            round(target_x_raw, 4),
            round(cs, 4) if not math.isnan(cs) else 'nan',
            round(cst, 4) if not math.isnan(cst) else 'nan',
            rl, ab, yl, abd,
            self.last_scan_pt_count,
        ]
        self.csv_writer.writerow(row)
        # 每 50 帧 flush 一次，掉电也能保住大部分数据
        if self.frame_idx % 50 == 0:
            self.csv_file.flush()
        self.frame_idx += 1

    def heartbeat(self):
        now = time.monotonic()
        if self.frame_idx == 0:
            self.get_logger().warn(
                '[TRACK_MEM] 还没收到任何 /target 帧。'
                '检查 perception_node 是否启动。'
            )
        else:
            rate = self.frame_idx / max(now - (self.t0 or now), 1e-3)
            self.get_logger().info(
                f'[TRACK_MEM] frames={self.frame_idx}, '
                f'rate≈{rate:.1f}Hz, s={self.cumulative_s:.2f}m, '
                f'file={os.path.basename(self.csv_path)}'
            )

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(
                f'[TRACK_MEM] CSV 关闭，共 {self.frame_idx} 帧。'
            )
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackMemoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
