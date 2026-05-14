#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
track_memory_viewer - 离线 TrackMemory 重画工具

目的：
    迁移 2022 蓝谷耐撞王代码的核心调试方法 ——
    读取 track_memory_recorder 录的 CSV，把任意时刻的车体状态、
    目标点、特殊模式触发位置在 matplotlib 上重画出来。

    2022 技术手册原话：
    "行驶过程记录各个时刻小车周围锥桶的坐标并保存。
     随后在 python 程序中调用并画出指定时刻的环境图，
     实现错误可视化，提高排错效率。"
     "通过 debug 工具，我们排错效率得到了大大的提高。"

显示：
    上图：车体轨迹（按模式着色），叠加目标点散点和特殊事件标记
    下图：时间序列（speed_cmd, steer_cmd, mode）
    交互：底部 slider 可拖动时间轴，左图会高亮当前位置

用法：
    python3 track_memory_viewer.py <csv_file>
    python3 track_memory_viewer.py ~/track_memory/track_memory_20241225_143012.csv

依赖：
    pip3 install matplotlib pandas
    （都是 ubuntu 22.04 自带或 ROS humble 默认环境就有的）
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.collections import LineCollection
import matplotlib.patches as mpatches

# 模式名 → 颜色，与 control_node 的语义对齐
MODE_COLORS = {
    'NORMAL':           '#2ca02c',  # 绿
    'YELLOW_CURVE':     '#ffbf00',  # 黄
    'RIGHT_ANGLE':      '#d62728',  # 红
    'RIGHT_CORRECTION': '#ff7f0e',  # 橙
    'KEEP_LAST':        '#9467bd',  # 紫
    'TRAFFIC_LIGHT':    '#e377c2',  # 粉
    'PARK_ZONE':        '#1f77b4',  # 蓝
}


def read_csv(path):
    """简易 CSV 读取，避免强依赖 pandas。"""
    import csv
    rows = []
    with open(path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    if not rows:
        raise RuntimeError(f'CSV 是空的: {path}')
    return rows


def to_float(s, default=float('nan')):
    try:
        return float(s)
    except (ValueError, TypeError):
        return default


def to_int(s, default=-1):
    try:
        return int(s)
    except (ValueError, TypeError):
        return default


def main():
    parser = argparse.ArgumentParser(description='离线重画 TrackMemory CSV')
    parser.add_argument('csv', help='track_memory_recorder 输出的 CSV 路径')
    parser.add_argument('--no-targets', action='store_true',
                        help='不画 NORMAL 模式下的目标点散点（CSV 大时加速）')
    args = parser.parse_args()

    if not os.path.isfile(args.csv):
        print(f'文件不存在: {args.csv}', file=sys.stderr)
        sys.exit(1)

    rows = read_csv(args.csv)
    print(f'读取 {len(rows)} 行: {args.csv}')

    # ---------- 解析 ----------
    t       = np.array([to_float(r['t'])         for r in rows])
    s       = np.array([to_float(r['s'])         for r in rows])
    x       = np.array([to_float(r['x'])         for r in rows])
    y       = np.array([to_float(r['y'])         for r in rows])
    yaw     = np.array([to_float(r['yaw'])       for r in rows])
    v       = np.array([to_float(r['v'])         for r in rows])
    tx      = np.array([to_float(r['target_x'])  for r in rows])
    ty      = np.array([to_float(r['target_y'])  for r in rows])
    mode    = [r['target_mode'] for r in rows]
    raw_x   = np.array([to_float(r['target_raw_x']) for r in rows])
    speed   = np.array([to_float(r['cmd_speed']) for r in rows])
    steer   = np.array([to_float(r['cmd_steer']) for r in rows])
    redl    = np.array([to_int(r['red_light'])   for r in rows])
    abs_    = np.array([to_int(r['ab_sign'])     for r in rows])
    yel     = np.array([to_int(r['yellow_line']) for r in rows])
    scan_n  = np.array([to_int(r['scan_point_count']) for r in rows])

    # 把 odom 坐标转到车体坐标系（以起点为原点，初始航向为 X 正方向）
    # 这样不同次启动的 CSV 都能对齐到同一个视图
    if not np.isnan(x[0]) and not np.isnan(y[0]):
        x0, y0, yaw0 = x[0], y[0], yaw[0]
        c, sn = np.cos(-yaw0), np.sin(-yaw0)
        x_local = c * (x - x0) - sn * (y - y0)
        y_local = sn * (x - x0) + c * (y - y0)
    else:
        x_local, y_local = x, y

    # ---------- 画布 ----------
    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(3, 2, width_ratios=[3, 2], height_ratios=[5, 1, 1],
                          hspace=0.35, wspace=0.25, bottom=0.10)

    ax_map  = fig.add_subplot(gs[0, 0])
    ax_legend = fig.add_subplot(gs[0, 1])
    ax_v    = fig.add_subplot(gs[1, :])
    ax_st   = fig.add_subplot(gs[2, :], sharex=ax_v)

    # ---------- 上图：轨迹 + 模式着色 ----------
    # 用 LineCollection 给每段轨迹按当前模式上色
    points = np.array([x_local, y_local]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    seg_colors = [MODE_COLORS.get(mode[i+1], '#888888')
                  for i in range(len(segments))]
    lc = LineCollection(segments, colors=seg_colors, linewidths=2.0)
    ax_map.add_collection(lc)

    # NORMAL 模式下的目标点散点
    if not args.no_targets:
        ok = np.array([m == 'NORMAL' for m in mode])
        ok &= ~np.isnan(tx) & ~np.isnan(ty)
        if ok.any():
            # 目标点也要转到 local 坐标系。注意 target 是车体坐标系下的相对量，
            # 这里需要叠加当时的车体位置和航向。
            tx_w = x + np.cos(yaw) * tx - np.sin(yaw) * ty
            ty_w = y + np.sin(yaw) * tx + np.cos(yaw) * ty
            tx_l = c * (tx_w - x0) - sn * (ty_w - y0)
            ty_l = sn * (tx_w - x0) + c * (ty_w - y0)
            ax_map.scatter(tx_l[ok], ty_l[ok], s=4, c='#aaaaaa', alpha=0.3,
                           label='target (NORMAL)')

    # 特殊事件标记
    for m, color in MODE_COLORS.items():
        if m == 'NORMAL':
            continue
        idx = [i for i, mm in enumerate(mode) if mm == m]
        if idx:
            ax_map.scatter(x_local[idx], y_local[idx], s=30, c=color,
                           marker='X', edgecolors='black', linewidths=0.5,
                           label=f'{m}', zorder=5)

    # 起点终点
    if len(x_local):
        ax_map.scatter([x_local[0]], [y_local[0]], marker='o', s=80,
                       facecolors='white', edgecolors='black', zorder=10)
        ax_map.text(x_local[0], y_local[0], '  START', fontsize=9, zorder=10)
        ax_map.scatter([x_local[-1]], [y_local[-1]], marker='s', s=80,
                       facecolors='white', edgecolors='black', zorder=10)
        ax_map.text(x_local[-1], y_local[-1], '  END', fontsize=9, zorder=10)

    # 当前帧高亮（slider 拖动时更新）
    current_marker, = ax_map.plot([x_local[0]], [y_local[0]],
                                  marker='*', markersize=18,
                                  color='red', zorder=20)

    ax_map.set_aspect('equal', adjustable='datalim')
    ax_map.set_xlabel('x_local [m]')
    ax_map.set_ylabel('y_local [m]')
    ax_map.set_title(f'轨迹（按模式着色） — {os.path.basename(args.csv)}',
                     fontsize=10)
    ax_map.grid(alpha=0.3)
    ax_map.relim()
    ax_map.autoscale_view()

    # ---------- 右上：图例 + 当前帧统计 ----------
    ax_legend.axis('off')
    legend_patches = [mpatches.Patch(color=c, label=m)
                      for m, c in MODE_COLORS.items()]
    ax_legend.legend(handles=legend_patches, loc='upper left',
                     fontsize=9, title='模式', title_fontsize=10)
    info_text = ax_legend.text(0.05, 0.35, '', fontsize=9,
                               family='monospace',
                               verticalalignment='top',
                               transform=ax_legend.transAxes)

    # ---------- 中下：speed/steer ----------
    ax_v.plot(t, speed, color='#1f77b4', linewidth=1.0, label='cmd_speed')
    ax_v.plot(t, v,     color='#aaaaaa', linewidth=0.8, label='odom_v')
    ax_v.set_ylabel('speed')
    ax_v.legend(loc='upper right', fontsize=8)
    ax_v.grid(alpha=0.3)

    ax_st.plot(t, steer, color='#d62728', linewidth=1.0, label='cmd_steer')
    ax_st.set_ylabel('steer')
    ax_st.set_xlabel('t [s]')
    ax_st.legend(loc='upper right', fontsize=8)
    ax_st.grid(alpha=0.3)

    # 在时间序列上叠加模式的彩色背景
    for i in range(1, len(t)):
        m = mode[i]
        if m != 'NORMAL':
            ax_v.axvspan(t[i-1], t[i], color=MODE_COLORS.get(m, '#888888'),
                         alpha=0.15)
            ax_st.axvspan(t[i-1], t[i], color=MODE_COLORS.get(m, '#888888'),
                          alpha=0.15)

    # 当前时间的垂直线
    vline_v  = ax_v.axvline(t[0], color='red', linewidth=0.8)
    vline_st = ax_st.axvline(t[0], color='red', linewidth=0.8)

    # ---------- 底部 slider ----------
    ax_slider = fig.add_axes([0.15, 0.03, 0.70, 0.025])
    slider = Slider(ax_slider, 'frame', 0, len(t) - 1, valinit=0, valstep=1)

    def update(_val):
        i = int(slider.val)
        current_marker.set_data([x_local[i]], [y_local[i]])
        vline_v.set_xdata([t[i], t[i]])
        vline_st.set_xdata([t[i], t[i]])
        info_text.set_text(
            f't       = {t[i]:.2f} s\n'
            f's       = {s[i]:.2f} m\n'
            f'mode    = {mode[i]}\n'
            f'raw_x   = {raw_x[i]:.1f}\n'
            f'v       = {v[i]:.2f}\n'
            f'cmd_spd = {speed[i]:.1f}\n'
            f'cmd_str = {steer[i]:.2f}\n'
            f'red     = {redl[i]}\n'
            f'ab      = {abs_[i]}\n'
            f'yellow  = {yel[i]}\n'
            f'scan_n  = {scan_n[i]}'
        )
        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)

    # ---------- 摘要打印 ----------
    print('-' * 60)
    print('模式频次统计：')
    from collections import Counter
    cnt = Counter(mode)
    for m, n in sorted(cnt.items(), key=lambda kv: -kv[1]):
        pct = 100.0 * n / len(mode)
        print(f'  {m:18s}  {n:6d} 帧  ({pct:5.1f}%)')
    print('-' * 60)
    print(f'总时长: {t[-1]:.2f} s')
    print(f'总路程: {s[-1]:.2f} m')
    print(f'平均频率: {len(t)/max(t[-1], 1e-3):.1f} Hz')
    print('-' * 60)
    print('图例颜色含义：')
    for m, c in MODE_COLORS.items():
        print(f'  {c}  {m}')
    print('-' * 60)
    print('拖动底部 slider 查看任意时刻的车体状态。')

    plt.show()


if __name__ == '__main__':
    main()
