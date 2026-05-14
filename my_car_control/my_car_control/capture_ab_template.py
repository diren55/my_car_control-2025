#!/usr/bin/env python3
"""
AB标志模板拍照程序
功能：在车上拍摄AB标志，保存原始图像和二值化图像作为模板
使用方法：
  1. 运行程序
  2. 调整HSV阈值，使AB标志在掩码中清晰可见
  3. 按空格键或's'键保存当前帧
  4. 按'q'键退出
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
import numpy as np
import os
from datetime import datetime

# HSV颜色阈值（蓝色）
lower_blue = np.array([105, 59, 20])
upper_blue = np.array([140, 193, 255])


class CaptureABTemplate(Node):
    def __init__(self):
        super().__init__('capture_ab_template')
        
        # 创建保存目录（固定路径）
        self.save_dir = os.path.expanduser('~/ab_templates')
        os.makedirs(self.save_dir, exist_ok=True)
        # 创建A和B子目录
        self.save_dir_A = os.path.join(self.save_dir, 'A')
        self.save_dir_B = os.path.join(self.save_dir, 'B')
        os.makedirs(self.save_dir_A, exist_ok=True)
        os.makedirs(self.save_dir_B, exist_ok=True)
        self.get_logger().info(f'📁 模板保存目录: {self.save_dir}')
        self.get_logger().info(f'   A模板目录: {self.save_dir_A}')
        self.get_logger().info(f'   B模板目录: {self.save_dir_B}')
        
        # 参数
        self.declare_parameter('blue_h_min', 105)
        self.declare_parameter('blue_h_max', 140)
        self.declare_parameter('blue_s_min', 59)
        self.declare_parameter('blue_s_max', 193)
        self.declare_parameter('blue_v_min', 20)
        self.declare_parameter('blue_v_max', 255)
        self.declare_parameter('crop_bottom', True)  # 是否裁剪下半部分
        self.declare_parameter('crop_right_ratio', 0.2)  # 右侧裁剪比例（0.2表示去掉20%）
        
        # 获取参数
        self.blue_h_min = self.get_parameter('blue_h_min').value
        self.blue_h_max = self.get_parameter('blue_h_max').value
        self.blue_s_min = self.get_parameter('blue_s_min').value
        self.blue_s_max = self.get_parameter('blue_s_max').value
        self.blue_v_min = self.get_parameter('blue_v_min').value
        self.blue_v_max = self.get_parameter('blue_v_max').value
        self.crop_bottom = self.get_parameter('crop_bottom').value
        self.crop_right_ratio = self.get_parameter('crop_right_ratio').value
        
        # 更新HSV阈值
        global lower_blue, upper_blue
        lower_blue = np.array([self.blue_h_min, self.blue_s_min, self.blue_v_min])
        upper_blue = np.array([self.blue_h_max, self.blue_s_max, self.blue_v_max])
        
        # 状态变量
        self.bridge = CvBridge()
        self.frame_count = 0
        self.save_count_A = 0
        self.save_count_B = 0
        self.save_count_general = 0
        
        # 创建窗口
        cv2.namedWindow("AB模板拍照工具", cv2.WINDOW_NORMAL)
        cv2.namedWindow("HSV掩码（二值化）", cv2.WINDOW_NORMAL)
        
        # QoS配置
        qos = QoSProfile(depth=10)
        
        # 订阅摄像头
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('📸 AB标志模板拍照工具已启动!')
        self.get_logger().info(f'   订阅话题: /image_raw')
        self.get_logger().info(f'   保存目录: {self.save_dir}')
        self.get_logger().info(f'   HSV阈值: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]')
        self.get_logger().info('')
        self.get_logger().info('操作说明:')
        self.get_logger().info('  "a"键: 保存A模板（保存到A目录）')
        self.get_logger().info('  "b"键: 保存B模板（保存到B目录）')
        self.get_logger().info('  空格键或"s"键: 保存通用模板（保存到根目录）')
        self.get_logger().info('  "q"键: 退出程序')
        self.get_logger().info('=' * 60)
    
    def update_blue_thresholds(self):
        """更新蓝色HSV阈值"""
        global lower_blue, upper_blue
        try:
            self.blue_h_min = self.get_parameter('blue_h_min').value
            self.blue_h_max = self.get_parameter('blue_h_max').value
            self.blue_s_min = self.get_parameter('blue_s_min').value
            self.blue_s_max = self.get_parameter('blue_s_max').value
            self.blue_v_min = self.get_parameter('blue_v_min').value
            self.blue_v_max = self.get_parameter('blue_v_max').value
        except:
            pass
        
        lower_blue = np.array([self.blue_h_min, self.blue_s_min, self.blue_v_min])
        upper_blue = np.array([self.blue_h_max, self.blue_s_max, self.blue_v_max])
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将ROS图像转为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge错误: {e}")
            return
        
        self.frame_count += 1
        
        # 更新HSV阈值
        self.update_blue_thresholds()
        
        # 裁剪区域（下半部分，右侧去掉一部分）
        if self.crop_bottom:
            half_row = frame.shape[0] // 2
            col_left = 0
            col_right = frame.shape[1] - int(frame.shape[1] * self.crop_right_ratio)
            crop_frame = frame[half_row:, col_left:col_right]
        else:
            crop_frame = frame.copy()
            half_row = 0
            col_left = 0
            col_right = frame.shape[1]
        
        # HSV转换和掩码提取
        hsv_img = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV)
        mask_img = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        # 二值化
        thresh = cv2.threshold(mask_img, 254, 255, 0)[1]
        
        # 准备显示图像
        display_frame = crop_frame.copy()
        
        # 在显示图像上添加信息
        info_y = 30
        cv2.putText(display_frame, "AB模板拍照工具", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        info_y += 35
        
        cv2.putText(display_frame, f"HSV: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]",
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 25
        
        blue_pixel_count = np.sum(mask_img > 0)
        cv2.putText(display_frame, f"蓝色像素数: {blue_pixel_count}",
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 25
        
        cv2.putText(display_frame, f"已保存: A={self.save_count_A}, B={self.save_count_B}, 通用={self.save_count_general}",
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 30
        
        cv2.putText(display_frame, "按'a'保存A模板 | 按'b'保存B模板 | 按空格保存通用",
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        info_y += 25
        cv2.putText(display_frame, "按'q'退出",
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 显示图像
        cv2.imshow("AB模板拍照工具", display_frame)
        cv2.imshow("HSV掩码（二值化）", thresh)
        
        # 处理按键
        key = cv2.waitKey(1) & 0xFF
        if key == ord('a') or key == ord('A'):
            self.save_template(crop_frame, thresh, mask_img, 'A')
        elif key == ord('b') or key == ord('B'):
            self.save_template(crop_frame, thresh, mask_img, 'B')
        elif key == ord(' ') or key == ord('s') or key == ord('S'):
            self.save_template(crop_frame, thresh, mask_img, 'general')
        elif key == ord('q') or key == ord('Q'):
            self.get_logger().info('退出程序...')
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return
    
    def save_template(self, original_frame, binary_frame, mask_frame, template_type='general'):
        """保存模板图像
        
        Args:
            template_type: 'A', 'B', 或 'general'
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 精确到毫秒
        
        # 根据类型选择保存目录
        if template_type == 'A':
            save_dir = self.save_dir_A
            self.save_count_A += 1
            count = self.save_count_A
        elif template_type == 'B':
            save_dir = self.save_dir_B
            self.save_count_B += 1
            count = self.save_count_B
        else:
            save_dir = self.save_dir
            self.save_count_general += 1
            count = self.save_count_general
        
        # 只保存二值化图像（用于模板匹配）
        binary_path = os.path.join(save_dir, f'template_{template_type}_{timestamp}.png')
        cv2.imwrite(binary_path, binary_frame)
        
        self.get_logger().info(f'✅ 已保存{template_type}模板 #{count}: {timestamp}')
        self.get_logger().info(f'   二值化图像: {binary_path}')


def main(args=None):
    rclpy.init(args=args)
    
    capture = CaptureABTemplate()
    
    try:
        rclpy.spin(capture)
    except KeyboardInterrupt:
        pass
    finally:
        capture.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

