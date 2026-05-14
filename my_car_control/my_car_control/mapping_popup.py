import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

try:
    import tkinter as tk
    from tkinter import font as tkfont
except Exception as exc:  # pragma: no cover
    tk = None
    tkfont = None

from cartographer_ros_msgs.msg import SubmapList


def _get_chinese_font() -> tuple:
    """获取支持中文的字体"""
    if tk is None or tkfont is None:
        return ('Arial', 12)
    
    # 常见的中文字体列表（按优先级）
    chinese_fonts = [
        'Noto Sans CJK SC',
        'WenQuanYi Micro Hei',
        'WenQuanYi Zen Hei',
        'SimHei',
        'Microsoft YaHei',
        'DejaVu Sans',
    ]
    
    # 获取系统所有可用字体
    try:
        available_fonts = tkfont.families()
        # 尝试找到支持中文的字体
        for font_name in chinese_fonts:
            if font_name in available_fonts:
                return (font_name, 12)
    except Exception:
        pass
    
    # 如果找不到，使用系统默认字体（通常支持中文）
    try:
        default_font = tkfont.nametofont('TkDefaultFont')
        return (default_font.actual()['family'], 12)
    except Exception:
        pass
    
    # 最后回退到 Arial
    return ('Arial', 12)


class MappingPopup(Node):
    def __init__(self) -> None:
        super().__init__('mapping_popup')
        self.subscription = self.create_subscription(
            SubmapList, '/submap_list', self._on_submap_list, 10
        )
        self.last_version_sum: int = -1
        self.last_count: int = 0
        self.last_update_time: float = 0.0

        if tk is None:
            self.get_logger().error('Tkinter 不可用：无法创建弹窗。请安装 Tk 或使用 RViz2。')
            return

        self.root = tk.Tk()
        self.root.title('建图进度 (Cartographer)')
        self.root.geometry('360x140')

        self.status_var = tk.StringVar(value='等待 /submap_list 数据...')
        self.count_var = tk.StringVar(value='子图数: 0')
        self.update_var = tk.StringVar(value='最近更新时间: -')

        # 使用支持中文的字体
        chinese_font = _get_chinese_font()
        font_large = (chinese_font[0], 14)
        font_normal = (chinese_font[0], 12)

        tk.Label(self.root, textvariable=self.status_var, font=font_large).pack(pady=6)
        tk.Label(self.root, textvariable=self.count_var, font=font_normal).pack()
        tk.Label(self.root, textvariable=self.update_var, font=font_normal).pack()

        # 每 200ms 刷新一次界面提示（基于最近一次回调数据）
        self.root.after(200, self._refresh_ui)

    def _on_submap_list(self, msg: SubmapList) -> None:
        # 以 submap_version 总和 + 子图数作为“进度变化”的直观指标
        version_sum = 0
        for submap in msg.submap:
            version_sum += submap.submap_version

        self.last_count = len(msg.submap)
        self.last_update_time = time.time()

        if version_sum != self.last_version_sum:
            self.last_version_sum = version_sum

    def _refresh_ui(self) -> None:
        if tk is None:
            return
        now = time.time()
        seconds_since_update = now - self.last_update_time if self.last_update_time > 0 else None

        if self.last_version_sum >= 0:
            self.status_var.set('建图进行中 ✅')
        else:
            self.status_var.set('等待数据...')

        self.count_var.set(f'子图数: {self.last_count}')
        if seconds_since_update is None:
            self.update_var.set('最近更新时间: -')
        else:
            self.update_var.set(f'最近更新时间: {int(seconds_since_update)} 秒前')

        self.root.after(200, self._refresh_ui)


def _spin_in_thread(node: Node) -> None:
    rclpy.spin(node)


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = MappingPopup()

    if tk is None:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        return

    # 将 ROS 事件循环放到后台线程，Tk 在主线程运行
    spin_thread = threading.Thread(target=_spin_in_thread, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


