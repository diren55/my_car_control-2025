#!/usr/bin/env python3
"""
视觉调试查看器 - 实时显示摄像头画面并支持阈值调整
功能：
1. 实时显示摄像头画面
2. 显示检测结果（红灯、A/B标识、黄线）
3. 支持通过ROS参数动态调整阈值
使用方法：ros2 run my_car_control vision_debug_viewer
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from rclpy.qos import QoSProfile
import numpy as np


class VisionDebugViewer(Node):
    """视觉调试查看器节点"""
    
    def __init__(self):
        super().__init__('vision_debug_viewer')
        
        # 参数声明（用于调整阈值）
        self.declare_parameter('red_filter_x', 500)
        self.declare_parameter('yellow_stop_height', 0.5)
        self.declare_parameter('show_detection', True)
        self.declare_parameter('window_name', 'Vision Debug')
        
        # 状态变量
        self.bridge = CvBridge()
        self.current_frame = None
        self.detection_result = [0, 0, 0]  # [红灯, A/B, 黄线]
        self.show_detection = True
        
        # QoS配置
        qos = QoSProfile(depth=10)
        
        # 订阅摄像头图像
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos
        )
        
        # 订阅检测结果
        self.detection_sub = self.create_subscription(
            Int16MultiArray,
            '/image_detection',
            self.detection_callback,
            qos
        )
        
        # 定时器（更新显示）
        self.timer = self.create_timer(0.033, self.update_display)  # ~30fps
        
        # 创建OpenCV窗口（如果GUI可用）
        window_name = self.get_parameter('window_name').value
        self.gui_available = False
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 800, 600)
            self.gui_available = True
        except cv2.error:
            self.get_logger().warn("⚠️  GUI不可用，将运行在无显示模式")
            self.gui_available = False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ 视觉调试查看器已启动')
        self.get_logger().info(f'   订阅图像话题: /image_raw')
        self.get_logger().info(f'   订阅检测结果: /image_detection')
        self.get_logger().info('=' * 60)
        self.get_logger().info('使用说明：')
        self.get_logger().info('  按 Q 或 ESC 退出')
        self.get_logger().info('  通过ROS参数调整阈值：')
        self.get_logger().info('    ros2 param set /image_detector red_filter_x 500')
        self.get_logger().info('    ros2 param set /image_detector yellow_stop_height 0.5')
        self.get_logger().info('=' * 60)
    
    def image_callback(self, msg: Image):
        """图像回调函数"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge错误: {e}")
    
    def detection_callback(self, msg: Int16MultiArray):
        """检测结果回调函数"""
        if len(msg.data) >= 3:
            self.detection_result = msg.data
    
    def update_display(self):
        """更新显示"""
        if self.current_frame is None:
            return
        
        # 获取参数
        self.show_detection = self.get_parameter('show_detection').value
        window_name = self.get_parameter('window_name').value
        
        # 复制帧用于显示
        display_frame = self.current_frame.copy()
        
        # 绘制检测结果
        if self.show_detection:
            # 检测结果文本
            red_text = f"Red: {self.detection_result[0]}"
            ab_text = f"AB: {self.detection_result[1]} (1=A, 2=B, 0=None)"
            yellow_text = f"Yellow: {self.detection_result[2]}"
            
            # 绘制文本背景
            cv2.rectangle(display_frame, (10, 10), (450, 140), (0, 0, 0), -1)
            cv2.rectangle(display_frame, (10, 10), (450, 140), (255, 255, 255), 2)
            
            # 绘制文本
            cv2.putText(display_frame, red_text, (20, 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_frame, ab_text, (20, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(display_frame, yellow_text, (20, 85), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 状态指示（彩色圆点）
            status_y = 115
            if self.detection_result[0]:
                cv2.circle(display_frame, (15, status_y), 8, (0, 0, 255), -1)
                cv2.circle(display_frame, (15, status_y), 10, (255, 255, 255), 2)
            if self.detection_result[1]:
                cv2.circle(display_frame, (85, status_y), 8, (255, 0, 0), -1)
                cv2.circle(display_frame, (85, status_y), 10, (255, 255, 255), 2)
            if self.detection_result[2]:
                cv2.circle(display_frame, (155, status_y), 8, (0, 255, 255), -1)
                cv2.circle(display_frame, (155, status_y), 10, (255, 255, 255), 2)
            
            # 显示当前阈值参数（从image_detector节点读取）
            try:
                red_filter_x = self.get_parameter('red_filter_x').value
                yellow_stop_height = self.get_parameter('yellow_stop_height').value
                
                param_text = [
                    f"Red Filter X: {red_filter_x}",
                    f"Yellow Height: {yellow_stop_height:.2f}",
                ]
                
                y_offset = 160
                for i, text in enumerate(param_text):
                    cv2.putText(display_frame, text, (10, y_offset + i * 25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            except:
                pass
        
        # 显示图像
        # 显示图像（如果GUI可用）
        if self.gui_available:
            try:
                cv2.imshow(window_name, display_frame)
                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q 或 ESC
                    self.get_logger().info('退出调试查看器')
                    cv2.destroyAllWindows()
                    self.gui_available = False
            except cv2.error:
                self.gui_available = False
                self.get_logger().warn("GUI窗口关闭，切换到无显示模式")
            return


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    viewer = VisionDebugViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
