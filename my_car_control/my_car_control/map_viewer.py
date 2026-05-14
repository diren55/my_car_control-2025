import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

try:
    import cv2  # type: ignore
    HAS_CV2 = True
except Exception:  # pragma: no cover
    HAS_CV2 = False


class MapViewerNode(Node):
    def __init__(self) -> None:
        super().__init__('map_viewer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scale', 0.2)  # 默认更小窗口（20%）
        self.declare_parameter('max_window_width', 480)   # 最大窗口宽度（像素）
        self.declare_parameter('max_window_height', 360)  # 最大窗口高度（像素）
        self.map_topic: str = self.get_parameter('map_topic').get_parameter_value().string_value
        self.scale: float = float(self.get_parameter('scale').get_parameter_value().double_value)
        self.max_w: int = int(self.get_parameter('max_window_width').get_parameter_value().integer_value)
        self.max_h: int = int(self.get_parameter('max_window_height').get_parameter_value().integer_value)
        if self.scale <= 0.05:
            self.scale = 0.05
        if self.scale > 2.0:
            self.scale = 2.0
        self.subscription = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, 10
        )
        self._last_img: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self.get_logger().info(f'MapViewer subscribing: {self.map_topic}')

        if not HAS_CV2:
            self.get_logger().warn('cv2 not available. Please install OpenCV for window display.')
        else:
            try:
                import cv2  # type: ignore
                cv2.namedWindow('Mapping (Cartographer /map)', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Mapping (Cartographer /map)', max(160, self.max_w), max(120, self.max_h))
            except Exception:
                pass

        # Timer to refresh window even无新消息
        self._timer = self.create_timer(0.1, self._refresh_window)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))
        # Unknown (-1) -> 127; 0 (free) -> 255 (white); 100 (occupied) -> 0 (black)
        img = np.full((height, width), 127, dtype=np.uint8)
        free_mask = (data == 0)
        occ_mask = (data > 0)
        img[free_mask] = 255
        img[occ_mask] = 0
        # Flip vertically to match conventional view
        img = np.flipud(img)
        with self._lock:
            self._last_img = img

    def _refresh_window(self) -> None:
        if not HAS_CV2:
            return
        with self._lock:
            img = None if self._last_img is None else self._last_img.copy()
        if img is None:
            # Show placeholder（按比例缩放）
            placeholder = np.zeros((200, 400), dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for /map ...', (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 2, cv2.LINE_AA)
            if self.scale != 1.0:
                placeholder = cv2.resize(placeholder, dsize=None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_NEAREST)
            cv2.imshow('Mapping (Cartographer /map)', placeholder)
        else:
            vis = cv2.applyColorMap(img, cv2.COLORMAP_BONE)
            # 基于绝对最大尺寸限制进行自适应缩放（只缩小，不放大）
            if self.max_w > 0 and self.max_h > 0:
                sh, sw = vis.shape[:2]
                limit_sf = min(self.max_w / float(sw), self.max_h / float(sh))
                sf = min(1.0, max(0.05, limit_sf * self.scale))
                if sf < 1.0:
                    vis = cv2.resize(vis, dsize=None, fx=sf, fy=sf, interpolation=cv2.INTER_NEAREST)
            cv2.imshow('Mapping (Cartographer /map)', vis)
        key = cv2.waitKey(1)
        if key in (27, ord('q')):
            self.get_logger().info('Exit viewer requested.')
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = MapViewerNode()
    try:
        rclpy.spin(node)
    finally:
        if HAS_CV2:
            try:
                import cv2  # type: ignore
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()


if __name__ == '__main__':
    main()
