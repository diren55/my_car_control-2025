#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线 CSV A/B 对比工具（TrackMemory 格式）
================================================================================
用途：把同一段 bag 用两个不同算法/参数版本跑出的 CSV 拿来对比。
      回答"我这个改动到底改变了什么"，避免"凭感觉"判断。

典型工作流：
  1. 在车上录一段 bag：
        ros2 bag record /scan /image_raw /image_detection /odom_combined \
                        /target /teleop_cmd_vel /tf /tf_static -o run_001
  2. 离线在 bag 上跑两个版本：
        # 旧版（git checkout v0-baseline）
        ros2 bag play run_001 & ros2 run my_car_control track_memory_recorder
        mv ~/track_memory/track_memory_*.csv old.csv

        # 新版（git checkout master）
        ros2 bag play run_001 & ros2 run my_car_control track_memory_recorder
        mv ~/track_memory/track_memory_*.csv new.csv
  3. 离线对比：
        python3 csv_diff.py old.csv new.csv

输出：
  - 控制台总结：圈时、模式触发次数对比、平均速度、累计 RMSE
  - matplotlib 4 个 panel 对比图：
      左大图：(x,y) 轨迹叠加，模式触发点标记
      右上：cmd_speed vs s
      右中：cmd_steer vs s
      右下：mode 时间线对比

================================================================================
对齐方式：按 s（累计弧长）重采样，而不是按时间。
        理由：两次跑的节奏不同（速度差异），按时间对齐会错位；
              按位置对齐才能公平比较"在同一个赛道位置做了什么不同的事"。

容错：CSV 字段缺失时降级显示，不崩。
================================================================================
"""

import sys
import argparse
from pathlib import Path

try:
    import numpy as np
    import pandas as pd
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError as e:
    print(f'[CSV_DIFF] 缺少依赖: {e}', file=sys.stderr)
    print('[CSV_DIFF] 安装: pip3 install pandas matplotlib numpy --break-system-packages',
          file=sys.stderr)
    sys.exit(2)


# 模式编码（与 track_memory_recorder.py MODE_MAP 对齐）
MODE_COLORS = {
    'NORMAL': '#4A90E2',
    'YELLOW_CURVE': '#F5A623',
    'RIGHT_ANGLE': '#D0021B',
    'RIGHT_CORRECTION': '#9013FE',
    'KEEP_LAST': '#7B7B7B',
    'TRAFFIC_LIGHT': '#FF6B9D',
    'PARK_ZONE': '#00BFA5',
    'UNKNOWN': '#888888',
}


def load_csv(path):
    """读 CSV，做基础校验。失败返回 None 并打错误。"""
    path = Path(path)
    if not path.exists():
        print(f'[CSV_DIFF] 文件不存在: {path}', file=sys.stderr)
        return None
    try:
        df = pd.read_csv(path)
    except Exception as e:
        print(f'[CSV_DIFF] 读 CSV 失败 ({path}): {e}', file=sys.stderr)
        return None
    if len(df) == 0:
        print(f'[CSV_DIFF] CSV 为空: {path}', file=sys.stderr)
        return None
    # 必须字段
    for col in ('s',):
        if col not in df.columns:
            print(f'[CSV_DIFF] 缺必须字段 {col}: {path}', file=sys.stderr)
            print(f'           可用字段: {list(df.columns)}', file=sys.stderr)
            return None
    return df


def safe_col(df, name, default=np.nan):
    """容错取列，缺失时返回常量数组。"""
    if name in df.columns:
        return df[name].values
    return np.full(len(df), default)


def resample_by_s(df, s_grid, fields):
    """
    按 s 网格重采样指定字段。线性插值。

    df: 原 dataframe
    s_grid: 目标 s 数组
    fields: 要重采样的字段列表
    返回: dict{field: np.array, 长度 = len(s_grid)}
    """
    s_orig = df['s'].values
    # 保证 s 单调（车可能短暂回退，去重排序）
    idx = np.argsort(s_orig)
    s_orig = s_orig[idx]

    result = {}
    for f in fields:
        if f not in df.columns:
            result[f] = np.full(len(s_grid), np.nan)
            continue
        y_orig = df[f].values[idx]
        # 数值字段插值
        if np.issubdtype(y_orig.dtype, np.number):
            result[f] = np.interp(s_grid, s_orig, y_orig)
        else:
            # 字符串字段（如 mode）用最近邻
            interp_idx = np.searchsorted(s_orig, s_grid, side='right') - 1
            interp_idx = np.clip(interp_idx, 0, len(y_orig) - 1)
            result[f] = y_orig[interp_idx]
    return result


def find_mode_transitions(s, mode):
    """找模式切换点，返回 [(s, from_mode, to_mode), ...]"""
    transitions = []
    if len(mode) == 0:
        return transitions
    prev = str(mode[0])
    for i in range(1, len(mode)):
        cur = str(mode[i])
        if cur != prev:
            transitions.append((float(s[i]), prev, cur))
            prev = cur
    return transitions


def summarize(name, df):
    """单 CSV 的统计摘要"""
    stats = {
        'name': name,
        'frames': len(df),
        's_max': float(df['s'].max()) if 's' in df.columns else np.nan,
        'duration': float(df['t'].max() - df['t'].min()) if 't' in df.columns else np.nan,
    }
    # 平均速度
    if 'v' in df.columns:
        stats['v_mean'] = float(df['v'].mean())
        stats['v_max'] = float(df['v'].max())
    else:
        stats['v_mean'] = np.nan
        stats['v_max'] = np.nan
    # 控制输出
    if 'cmd_speed' in df.columns:
        stats['cmd_speed_mean'] = float(df['cmd_speed'].mean())
    else:
        stats['cmd_speed_mean'] = np.nan
    # 模式统计
    if 'mode' in df.columns:
        stats['mode_counts'] = df['mode'].value_counts().to_dict()
        # 模式切换次数
        stats['mode_transitions'] = int((df['mode'].shift() != df['mode']).sum() - 1)
    else:
        stats['mode_counts'] = {}
        stats['mode_transitions'] = 0
    return stats


def print_summary(s_old, s_new):
    """控制台打印对比摘要"""
    print('=' * 78)
    print(f'  {"指标":<25} {"旧版":>20} {"新版":>20} {"差异":>10}')
    print('-' * 78)

    def line(label, old, new, fmt='{:.3f}'):
        try:
            old_s = fmt.format(old) if not np.isnan(old) else 'N/A'
            new_s = fmt.format(new) if not np.isnan(new) else 'N/A'
            if not np.isnan(old) and not np.isnan(new):
                diff = new - old
                diff_s = ('+' if diff > 0 else '') + fmt.format(diff)
            else:
                diff_s = '-'
        except Exception:
            old_s, new_s, diff_s = str(old), str(new), '-'
        print(f'  {label:<25} {old_s:>20} {new_s:>20} {diff_s:>10}')

    line('帧数', s_old['frames'], s_new['frames'], '{:.0f}')
    line('总时长 (s)', s_old['duration'], s_new['duration'], '{:.2f}')
    line('累计弧长 (m)', s_old['s_max'], s_new['s_max'], '{:.2f}')
    line('平均速度', s_old['v_mean'], s_new['v_mean'])
    line('最大速度', s_old['v_max'], s_new['v_max'])
    line('cmd_speed 均值', s_old['cmd_speed_mean'], s_new['cmd_speed_mean'])
    line('模式切换次数', s_old['mode_transitions'], s_new['mode_transitions'], '{:.0f}')
    print('-' * 78)

    # 模式触发次数表
    all_modes = set(s_old['mode_counts'].keys()) | set(s_new['mode_counts'].keys())
    print(f'  {"模式":<25} {"旧版次数":>20} {"新版次数":>20} {"差异":>10}')
    for m in sorted(all_modes):
        o = s_old['mode_counts'].get(m, 0)
        n = s_new['mode_counts'].get(m, 0)
        diff = n - o
        diff_s = ('+' if diff > 0 else '') + str(diff)
        marker = '  '
        if abs(diff) >= 2:
            marker = '⚠ '  # 触发次数差 ≥2 高亮
        print(f'  {marker}{m:<23} {o:>20} {n:>20} {diff_s:>10}')
    print('=' * 78)


def plot_diff(df_old, df_new, label_old='旧版', label_new='新版'):
    """matplotlib 多 panel 对比"""
    # 字体（Linux 上 SimHei 可能没有，回退到默认）
    try:
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
    except Exception:
        pass

    fig = plt.figure(figsize=(16, 9))
    gs = GridSpec(3, 2, width_ratios=[1.4, 1], height_ratios=[1, 1, 0.6],
                  hspace=0.35, wspace=0.25)

    # ---- 左大图：轨迹叠加 ----
    ax_traj = fig.add_subplot(gs[:2, 0])
    if 'x' in df_old.columns and 'y' in df_old.columns:
        ax_traj.plot(df_old['x'], df_old['y'],
                     color='#4A90E2', alpha=0.6, lw=1.5, label=label_old)
    if 'x' in df_new.columns and 'y' in df_new.columns:
        ax_traj.plot(df_new['x'], df_new['y'],
                     color='#D0021B', alpha=0.6, lw=1.5, label=label_new)
    # 模式触发点（非 NORMAL）
    for df, marker in [(df_old, 'o'), (df_new, 'x')]:
        if 'mode' not in df.columns:
            continue
        non_normal = df[df['mode'] != 'NORMAL']
        if len(non_normal) == 0:
            continue
        if 'x' in non_normal.columns and 'y' in non_normal.columns:
            colors = [MODE_COLORS.get(m, '#888') for m in non_normal['mode']]
            ax_traj.scatter(non_normal['x'], non_normal['y'],
                            c=colors, marker=marker, s=20, alpha=0.7,
                            edgecolors='black', linewidths=0.3)
    ax_traj.set_xlabel('x (m)')
    ax_traj.set_ylabel('y (m)')
    ax_traj.set_title('轨迹叠加（圆=旧, 叉=新, 颜色=模式）')
    ax_traj.legend(loc='best')
    ax_traj.set_aspect('equal', adjustable='datalim')
    ax_traj.grid(True, alpha=0.3)

    # ---- 右上：cmd_speed vs s ----
    ax_sp = fig.add_subplot(gs[0, 1])
    if 'cmd_speed' in df_old.columns:
        ax_sp.plot(df_old['s'], df_old['cmd_speed'],
                   color='#4A90E2', alpha=0.7, lw=1.2, label=label_old)
    if 'cmd_speed' in df_new.columns:
        ax_sp.plot(df_new['s'], df_new['cmd_speed'],
                   color='#D0021B', alpha=0.7, lw=1.2, label=label_new)
    ax_sp.set_xlabel('s (m)')
    ax_sp.set_ylabel('cmd_speed')
    ax_sp.set_title('速度指令 vs 弧长')
    ax_sp.legend(loc='best')
    ax_sp.grid(True, alpha=0.3)

    # ---- 右中：cmd_steer vs s ----
    ax_st = fig.add_subplot(gs[1, 1])
    if 'cmd_steer' in df_old.columns:
        ax_st.plot(df_old['s'], df_old['cmd_steer'],
                   color='#4A90E2', alpha=0.7, lw=1.2, label=label_old)
    if 'cmd_steer' in df_new.columns:
        ax_st.plot(df_new['s'], df_new['cmd_steer'],
                   color='#D0021B', alpha=0.7, lw=1.2, label=label_new)
    ax_st.set_xlabel('s (m)')
    ax_st.set_ylabel('cmd_steer')
    ax_st.set_title('转向指令 vs 弧长')
    ax_st.legend(loc='best')
    ax_st.grid(True, alpha=0.3)

    # ---- 底部：模式时间线 ----
    ax_mode = fig.add_subplot(gs[2, :])
    for row_idx, (df, label) in enumerate(
            [(df_old, label_old), (df_new, label_new)]):
        if 'mode' not in df.columns:
            continue
        for m in df['mode'].unique():
            mask = df['mode'] == m
            color = MODE_COLORS.get(str(m), '#888')
            ax_mode.scatter(df.loc[mask, 's'], np.full(mask.sum(), row_idx),
                            c=color, marker='|', s=80, alpha=0.7, label=m)
    ax_mode.set_yticks([0, 1])
    ax_mode.set_yticklabels([label_old, label_new])
    ax_mode.set_xlabel('s (m)')
    ax_mode.set_title('模式时间线（按弧长）')
    ax_mode.set_ylim(-0.5, 1.5)
    # 去重图例
    handles, labels = ax_mode.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax_mode.legend(by_label.values(), by_label.keys(),
                   loc='upper right', fontsize=8, ncol=4)
    ax_mode.grid(True, alpha=0.3, axis='x')

    plt.suptitle(f'CSV 对比: {label_old}  vs  {label_new}', fontsize=14)
    return fig


def compute_diff_metrics(df_old, df_new):
    """按 s 重采样后，计算 target / cmd 的差异 RMSE"""
    s_min = max(df_old['s'].min(), df_new['s'].min())
    s_max = min(df_old['s'].max(), df_new['s'].max())
    if s_max <= s_min:
        print('[CSV_DIFF] WARN: 两个 CSV 的 s 范围不重叠')
        return {}
    s_step = 0.05  # 5cm 一格
    s_grid = np.arange(s_min, s_max, s_step)

    fields = ['target_x', 'target_y', 'cmd_speed', 'cmd_steer', 'v']
    old_r = resample_by_s(df_old, s_grid, fields)
    new_r = resample_by_s(df_new, s_grid, fields)

    metrics = {'s_overlap_min': s_min, 's_overlap_max': s_max,
               's_overlap_len': s_max - s_min}

    for f in fields:
        if f in old_r and f in new_r:
            valid = ~(np.isnan(old_r[f]) | np.isnan(new_r[f]))
            if valid.sum() > 0:
                diff = old_r[f][valid] - new_r[f][valid]
                metrics[f'rmse_{f}'] = float(np.sqrt(np.mean(diff ** 2)))
                metrics[f'mean_diff_{f}'] = float(np.mean(np.abs(diff)))
            else:
                metrics[f'rmse_{f}'] = np.nan
                metrics[f'mean_diff_{f}'] = np.nan
    return metrics


def print_diff_metrics(metrics):
    if not metrics:
        return
    print()
    print('=' * 78)
    print(f"  按 s 重采样对比（重叠 s 范围: "
          f"{metrics.get('s_overlap_min', 0):.2f} ~ "
          f"{metrics.get('s_overlap_max', 0):.2f} m, "
          f"长度 {metrics.get('s_overlap_len', 0):.2f} m）")
    print('-' * 78)
    for key in ['target_x', 'target_y', 'cmd_speed', 'cmd_steer', 'v']:
        rmse_k = f'rmse_{key}'
        mean_k = f'mean_diff_{key}'
        if rmse_k in metrics:
            rmse = metrics[rmse_k]
            mean_d = metrics[mean_k]
            rmse_s = f'{rmse:.4f}' if not np.isnan(rmse) else 'N/A'
            mean_s = f'{mean_d:.4f}' if not np.isnan(mean_d) else 'N/A'
            print(f'  {key:<15} RMSE = {rmse_s:>10}   |Δ| 均值 = {mean_s:>10}')
    print('=' * 78)


def main():
    parser = argparse.ArgumentParser(
        description='离线对比两份 TrackMemory CSV',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('old_csv', help='旧版/基线 CSV')
    parser.add_argument('new_csv', help='新版/待测 CSV')
    parser.add_argument('--no-plot', action='store_true', help='只打文字摘要，不显示图')
    parser.add_argument('--save-plot', metavar='PATH', help='保存图到文件而不是显示')
    parser.add_argument('--label-old', default='旧版', help='旧版的图例标签')
    parser.add_argument('--label-new', default='新版', help='新版的图例标签')
    args = parser.parse_args()

    df_old = load_csv(args.old_csv)
    df_new = load_csv(args.new_csv)
    if df_old is None or df_new is None:
        sys.exit(1)

    print(f'\n[CSV_DIFF] 旧版: {args.old_csv} ({len(df_old)} 帧)')
    print(f'[CSV_DIFF] 新版: {args.new_csv} ({len(df_new)} 帧)\n')

    s_old = summarize(args.label_old, df_old)
    s_new = summarize(args.label_new, df_new)
    print_summary(s_old, s_new)

    metrics = compute_diff_metrics(df_old, df_new)
    print_diff_metrics(metrics)

    if args.no_plot:
        return

    fig = plot_diff(df_old, df_new, args.label_old, args.label_new)
    if args.save_plot:
        fig.savefig(args.save_plot, dpi=120, bbox_inches='tight')
        print(f'\n[CSV_DIFF] 图保存到: {args.save_plot}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
