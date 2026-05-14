#!/usr/bin/env python3
"""
红灯检测HSV调试工具
功能：实时显示摄像头画面，调节HSV参数，查看红灯检测效果
使用：ros2 run my_car_control debug_red_hsv
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DebugRedHSV(Node):
    def __init__(self):
        super().__init__('debug_red_hsv')
        
        self.bridge = CvBridge()
        
        # 订阅摄像头
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        # 当前HSV参数（双区间：0-10 和 160-180）
        # 区间1: 0-10 (红色一端)
        self.red1_h_min = 0
        self.red1_h_max = 10
        self.red1_s_min = 100
        self.red1_s_max = 255
        self.red1_v_min = 100
        self.red1_v_max = 255
        
        # 区间2: 160-180 (红色另一端)
        self.red2_h_min = 160
        self.red2_h_max = 180
        self.red2_s_min = 100
        self.red2_s_max = 255
        self.red2_v_min = 100
        self.red2_v_max = 255
        
        # 红灯过滤参数
        self.red_filter_x = 500  # X坐标过滤（只检测右侧）
        
        # 最新帧
        self.latest_frame = None
        
        # 创建窗口和滑动条
        cv2.namedWindow('1. 原始画面')
        cv2.namedWindow('2. HSV掩码（红灯提取）')
        cv2.namedWindow('3. 形态学处理后')
        cv2.namedWindow('4. 检测结果')
        cv2.namedWindow('参数调节-区间1 (0-10)')
        cv2.namedWindow('参数调节-区间2 (160-180)')
        cv2.namedWindow('其他参数')
        
        # 区间1 HSV参数滑动条
        cv2.createTrackbar('H_min', '参数调节-区间1 (0-10)', self.red1_h_min, 10, self.on_red1_h_min)
        cv2.createTrackbar('H_max', '参数调节-区间1 (0-10)', self.red1_h_max, 10, self.on_red1_h_max)
        cv2.createTrackbar('S_min', '参数调节-区间1 (0-10)', self.red1_s_min, 255, self.on_red1_s_min)
        cv2.createTrackbar('S_max', '参数调节-区间1 (0-10)', self.red1_s_max, 255, self.on_red1_s_max)
        cv2.createTrackbar('V_min', '参数调节-区间1 (0-10)', self.red1_v_min, 255, self.on_red1_v_min)
        cv2.createTrackbar('V_max', '参数调节-区间1 (0-10)', self.red1_v_max, 255, self.on_red1_v_max)
        
        # 区间2 HSV参数滑动条
        cv2.createTrackbar('H_min', '参数调节-区间2 (160-180)', self.red2_h_min, 179, self.on_red2_h_min)
        cv2.createTrackbar('H_max', '参数调节-区间2 (160-180)', self.red2_h_max, 179, self.on_red2_h_max)
        cv2.createTrackbar('S_min', '参数调节-区间2 (160-180)', self.red2_s_min, 255, self.on_red2_s_min)
        cv2.createTrackbar('S_max', '参数调节-区间2 (160-180)', self.red2_s_max, 255, self.on_red2_s_max)
        cv2.createTrackbar('V_min', '参数调节-区间2 (160-180)', self.red2_v_min, 255, self.on_red2_v_min)
        cv2.createTrackbar('V_max', '参数调节-区间2 (160-180)', self.red2_v_max, 255, self.on_red2_v_max)
        
        # 其他参数
        cv2.createTrackbar('X过滤', '其他参数', self.red_filter_x, 640, self.on_filter_x)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔧 红灯检测HSV调试工具已启动')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📹 等待摄像头画面...')
        self.get_logger().info('')
        self.get_logger().info('【操作说明】')
        self.get_logger().info('  1. 调节滑动条，实时查看红灯检测效果')
        self.get_logger().info('  2. 窗口1：原始摄像头画面')
        self.get_logger().info('  3. 窗口2：HSV掩码（红色部分会显示为白色）')
        self.get_logger().info('  4. 窗口3：形态学处理后（去噪）')
        self.get_logger().info('  5. 窗口4：检测结果（显示圆圈和轮廓）')
        self.get_logger().info('  6. 按 "q" 退出')
        self.get_logger().info('  7. 按 "s" 保存当前参数到终端')
        self.get_logger().info('=' * 60)
        
        # 定时器：处理图像
        self.timer = self.create_timer(0.03, self.process_image)  # 30Hz
    
    # 区间1回调函数
    def on_red1_h_min(self, val):
        self.red1_h_min = val
        # 确保H_min <= H_max
        if self.red1_h_min > self.red1_h_max:
            cv2.setTrackbarPos('H_max', '参数调节-区间1 (0-10)', self.red1_h_min)
            self.red1_h_max = self.red1_h_min
    
    def on_red1_h_max(self, val):
        self.red1_h_max = val
        # 确保H_min <= H_max
        if self.red1_h_max < self.red1_h_min:
            cv2.setTrackbarPos('H_min', '参数调节-区间1 (0-10)', self.red1_h_max)
            self.red1_h_min = self.red1_h_max
    
    def on_red1_s_min(self, val):
        self.red1_s_min = val
        if self.red1_s_min > self.red1_s_max:
            cv2.setTrackbarPos('S_max', '参数调节-区间1 (0-10)', self.red1_s_min)
            self.red1_s_max = self.red1_s_min
    
    def on_red1_s_max(self, val):
        self.red1_s_max = val
        if self.red1_s_max < self.red1_s_min:
            cv2.setTrackbarPos('S_min', '参数调节-区间1 (0-10)', self.red1_s_max)
            self.red1_s_min = self.red1_s_max
    
    def on_red1_v_min(self, val):
        self.red1_v_min = val
        if self.red1_v_min > self.red1_v_max:
            cv2.setTrackbarPos('V_max', '参数调节-区间1 (0-10)', self.red1_v_min)
            self.red1_v_max = self.red1_v_min
    
    def on_red1_v_max(self, val):
        self.red1_v_max = val
        if self.red1_v_max < self.red1_v_min:
            cv2.setTrackbarPos('V_min', '参数调节-区间1 (0-10)', self.red1_v_max)
            self.red1_v_min = self.red1_v_max
    
    # 区间2回调函数
    def on_red2_h_min(self, val):
        self.red2_h_min = val
        if self.red2_h_min > self.red2_h_max:
            cv2.setTrackbarPos('H_max', '参数调节-区间2 (160-180)', self.red2_h_min)
            self.red2_h_max = self.red2_h_min
    
    def on_red2_h_max(self, val):
        self.red2_h_max = val
        if self.red2_h_max < self.red2_h_min:
            cv2.setTrackbarPos('H_min', '参数调节-区间2 (160-180)', self.red2_h_max)
            self.red2_h_min = self.red2_h_max
    
    def on_red2_s_min(self, val):
        self.red2_s_min = val
        if self.red2_s_min > self.red2_s_max:
            cv2.setTrackbarPos('S_max', '参数调节-区间2 (160-180)', self.red2_s_min)
            self.red2_s_max = self.red2_s_min
    
    def on_red2_s_max(self, val):
        self.red2_s_max = val
        if self.red2_s_max < self.red2_s_min:
            cv2.setTrackbarPos('S_min', '参数调节-区间2 (160-180)', self.red2_s_max)
            self.red2_s_min = self.red2_s_max
    
    def on_red2_v_min(self, val):
        self.red2_v_min = val
        if self.red2_v_min > self.red2_v_max:
            cv2.setTrackbarPos('V_max', '参数调节-区间2 (160-180)', self.red2_v_min)
            self.red2_v_max = self.red2_v_min
    
    def on_red2_v_max(self, val):
        self.red2_v_max = val
        if self.red2_v_max < self.red2_v_min:
            cv2.setTrackbarPos('V_min', '参数调节-区间2 (160-180)', self.red2_v_max)
            self.red2_v_min = self.red2_v_max
    
    def on_filter_x(self, val):
        self.red_filter_x = val
    
    def image_callback(self, msg):
        """接收摄像头画面"""
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def process_image(self):
        """处理图像，显示调试信息"""
        if self.latest_frame is None:
            return
        
        frame = self.latest_frame.copy()
        
        # 1. 显示原始画面
        cv2.imshow('1. 原始画面', frame)
        
        # 2. HSV转换和红色提取（双区间）
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 区间1: 0-10
        lower_red1 = np.array([self.red1_h_min, self.red1_s_min, self.red1_v_min])
        upper_red1 = np.array([self.red1_h_max, self.red1_s_max, self.red1_v_max])
        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        
        # 区间2: 160-180
        lower_red2 = np.array([self.red2_h_min, self.red2_s_min, self.red2_v_min])
        upper_red2 = np.array([self.red2_h_max, self.red2_s_max, self.red2_v_max])
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        
        # 合并掩码
        mask_red = cv2.add(mask1, mask2)
        cv2.imshow('2. HSV掩码（红灯提取）', mask_red)
        
        # 3. 形态学处理（与image_detector.py一致）
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        threshed_img_smooth_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=1)
        threshed_img_smooth_red = cv2.dilate(threshed_img_smooth_red, kernel, iterations=2)
        cv2.imshow('3. 形态学处理后', threshed_img_smooth_red)
        
        # 4. 霍夫圆检测
        circles = None
        circle_detected = False
        valid_circles = []
        if threshed_img_smooth_red is not None:
            circles = cv2.HoughCircles(
                threshed_img_smooth_red,
                cv2.HOUGH_GRADIENT,
                1, 20,
                param1=50,
                param2=15,
                minRadius=3,
                maxRadius=100
            )
            
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    x, y, r = int(i[0]), int(i[1]), int(i[2])
                    if x >= self.red_filter_x:
                        valid_circles.append((x, y, r))
                        circle_detected = True
        
        # 5. 轮廓检测
        contours1, _ = cv2.findContours(
            threshed_img_smooth_red,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE
        )
        
        contour_detected = False
        valid_contours = []
        for cnt in contours1:
            (x, y, w, h) = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h if h > 0 else 0
            if w*h > 30 and x >= self.red_filter_x and 0.5 < aspect_ratio < 2.0:
                valid_contours.append((x, y, w, h))
                contour_detected = True
        
        # 6. 综合判断
        detected = circle_detected or contour_detected
        
        # 7. 显示检测结果
        result_frame = frame.copy()
        
        # 绘制检测到的圆圈
        for x, y, r in valid_circles:
            cv2.circle(result_frame, (x, y), r, (0, 255, 0), 2)  # 绿色圆圈
            cv2.circle(result_frame, (x, y), 2, (0, 255, 0), 3)  # 圆心
            cv2.putText(result_frame, f"R:{r}", (x, y-r-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 绘制检测到的轮廓
        for x, y, w, h in valid_contours:
            cv2.rectangle(result_frame, (x, y), (x+w, y+h), (0, 255, 255), 1)
        
        # 绘制X过滤线
        cv2.line(result_frame, (self.red_filter_x, 0), 
                (self.red_filter_x, frame.shape[0]), (255, 0, 0), 2)
        cv2.putText(result_frame, f"X>={self.red_filter_x}", 
                   (self.red_filter_x + 5, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # 显示状态
        status = "检测到红灯！" if detected else "未检测到红灯"
        color = (0, 0, 255) if detected else (0, 255, 0)
        cv2.putText(result_frame, status, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        # 显示参数信息
        info_y = 60
        cv2.putText(result_frame, f"区间1: H[{self.red1_h_min}-{self.red1_h_max}] S[{self.red1_s_min}-{self.red1_s_max}] V[{self.red1_v_min}-{self.red1_v_max}]", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 25
        cv2.putText(result_frame, f"区间2: H[{self.red2_h_min}-{self.red2_h_max}] S[{self.red2_s_min}-{self.red2_s_max}] V[{self.red2_v_min}-{self.red2_v_max}]", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 25
        cv2.putText(result_frame, f"圆圈检测: {'✓' if circle_detected else '✗'} ({len(valid_circles)}个)", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        info_y += 25
        cv2.putText(result_frame, f"轮廓检测: {'✓' if contour_detected else '✗'} ({len(valid_contours)}个)", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.imshow('4. 检测结果', result_frame)
        
        # 处理按键
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('退出调试工具')
            rclpy.shutdown()
        elif key == ord('s'):
            self.save_parameters()
    
    def save_parameters(self):
        """保存当前参数到终端"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📋 当前红灯检测参数：')
        self.get_logger().info('=' * 60)
        self.get_logger().info('区间1 (0-10):')
        self.get_logger().info(f'  lower_red1 = np.array([{self.red1_h_min}, {self.red1_s_min}, {self.red1_v_min}])')
        self.get_logger().info(f'  upper_red1 = np.array([{self.red1_h_max}, {self.red1_s_max}, {self.red1_v_max}])')
        self.get_logger().info('区间2 (160-180):')
        self.get_logger().info(f'  lower_red2 = np.array([{self.red2_h_min}, {self.red2_s_min}, {self.red2_v_min}])')
        self.get_logger().info(f'  upper_red2 = np.array([{self.red2_h_max}, {self.red2_s_max}, {self.red2_v_max}])')
        self.get_logger().info(f'red_filter_x = {self.red_filter_x}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📝 复制以上参数到 image_detector.py 的 redLightDetect 函数中')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    node = DebugRedHSV()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

