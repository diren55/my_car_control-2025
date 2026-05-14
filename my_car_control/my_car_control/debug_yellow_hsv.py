#!/usr/bin/env python3
"""
黄线检测HSV调试工具
功能：实时显示摄像头画面，调节HSV参数，查看黄线检测效果
使用：ros2 run my_car_control debug_yellow_hsv
"""

import cv2
import numpy as np
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DebugYellowHSV(Node):
    def __init__(self):
        super().__init__('debug_yellow_hsv')
        
        self.bridge = CvBridge()
        
        # 订阅摄像头
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        # 当前HSV参数（调整为更宽松的默认值，便于调试）
        self.yellow_h_min = 15
        self.yellow_h_max = 40
        self.yellow_s_min = 80    # 降低饱和度要求
        self.yellow_s_max = 255
        self.yellow_v_min = 20    # 降低亮度要求，更容易看到黄线
        self.yellow_v_max = 255
        
        # 黄线停止高度阈值
        self.yellow_stop_height = 50  # 百分比 0-100
        
        # 最新帧
        self.latest_frame = None
        
        # 创建窗口和滑动条
        cv2.namedWindow('1. 原始画面')
        cv2.namedWindow('2. HSV掩码（黄线提取）')
        cv2.namedWindow('3. 形态学处理后')
        cv2.namedWindow('4. 检测结果')
        cv2.namedWindow('参数调节')
        
        # HSV参数滑动条
        cv2.createTrackbar('H_min', '参数调节', self.yellow_h_min, 179, self.on_h_min)
        cv2.createTrackbar('H_max', '参数调节', self.yellow_h_max, 179, self.on_h_max)
        cv2.createTrackbar('S_min', '参数调节', self.yellow_s_min, 255, self.on_s_min)
        cv2.createTrackbar('S_max', '参数调节', self.yellow_s_max, 255, self.on_s_max)
        cv2.createTrackbar('V_min', '参数调节', self.yellow_v_min, 255, self.on_v_min)
        cv2.createTrackbar('V_max', '参数调节', self.yellow_v_max, 255, self.on_v_max)
        cv2.createTrackbar('停止高度%', '参数调节', self.yellow_stop_height, 100, self.on_height)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔧 黄线检测HSV调试工具已启动')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📹 等待摄像头画面...')
        self.get_logger().info('')
        self.get_logger().info('【操作说明】')
        self.get_logger().info('  1. 调节滑动条，实时查看黄线检测效果')
        self.get_logger().info('  2. 窗口1：原始摄像头画面')
        self.get_logger().info('  3. 窗口2：HSV掩码（黄色部分会显示为白色）')
        self.get_logger().info('  4. 窗口3：形态学处理后（去噪）')
        self.get_logger().info('  5. 窗口4：检测结果（显示黄线位置和状态）')
        self.get_logger().info('  6. 按 "q" 退出')
        self.get_logger().info('  7. 按 "s" 保存当前参数到终端')
        self.get_logger().info('=' * 60)
        
        # 定时器：处理图像
        self.timer = self.create_timer(0.03, self.process_image)  # 30Hz
    
    def on_h_min(self, val):
        self.yellow_h_min = val
    
    def on_h_max(self, val):
        self.yellow_h_max = val
    
    def on_s_min(self, val):
        self.yellow_s_min = val
    
    def on_s_max(self, val):
        self.yellow_s_max = val
    
    def on_v_min(self, val):
        self.yellow_v_min = val
    
    def on_v_max(self, val):
        self.yellow_v_max = val
    
    def on_height(self, val):
        self.yellow_stop_height = val
    
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
        
        # 2. HSV转换和黄线提取
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        minYellow = np.array([self.yellow_h_min, self.yellow_s_min, self.yellow_v_min])
        maxYellow = np.array([self.yellow_h_max, self.yellow_s_max, self.yellow_v_max])
        mask_yellow = cv2.inRange(img_hsv, minYellow, maxYellow)
        cv2.imshow('2. HSV掩码（黄线提取）', mask_yellow)
        
        # 3. 形态学处理（与image_detector.py完全一致）
        thresh = cv2.threshold(mask_yellow, 254, 255, 0)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
        threshed_img_smooth = cv2.erode(thresh, kernel, iterations=1)
        threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations=2)
        cv2.imshow('3. 形态学处理后', threshed_img_smooth)
        
        # 4. 检测黄线（与image_detector.py完全一致）
        image = cv2.cvtColor(threshed_img_smooth, cv2.COLOR_GRAY2BGR)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 裁剪中间三分之一
        quarter_col_left = gray.shape[1] // 3
        quarter_col_right = gray.shape[1] - (gray.shape[1] // 3)
        crop_gray = gray[:, quarter_col_left:quarter_col_right]
        
        crop_gray_copy = copy.deepcopy(crop_gray)
        _, thresh1 = cv2.threshold(crop_gray_copy, 127, 255, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # 计算平均Y坐标
        aver_y = 0
        cnt = 0
        contour_length = 0
        
        for contour in contours:
            contour_length += contour.shape[0]
            for point in contour:
                aver_y += point[0][1]
                cnt += 1
        
        # 5. 显示检测结果
        result_frame = frame.copy()
        
        # 绘制裁剪区域
        cv2.rectangle(result_frame, 
                     (quarter_col_left, 0), 
                     (quarter_col_right, frame.shape[0]), 
                     (255, 0, 255), 2)
        
        detected = False
        if cnt > 0:
            aver_y = aver_y // cnt
            yellow_stop_height_ratio = self.yellow_stop_height / 100.0
            
            # 判断是否检测到黄线
            if (aver_y >= crop_gray.shape[0] * yellow_stop_height_ratio and 
                aver_y != crop_gray.shape[0] and 
                contour_length >= 12):
                detected = True
        
        # 显示检测状态
        if detected:
            status_text = "检测到黄线！"
            status_color = (0, 255, 0)  # 绿色
            # 绘制黄线位置
            line_y = aver_y
            cv2.line(result_frame, 
                    (quarter_col_left, line_y), 
                    (quarter_col_right, line_y), 
                    (0, 255, 255), 3)
        else:
            status_text = "未检测到黄线"
            status_color = (0, 0, 255)  # 红色
        
        # 显示参数信息
        cv2.putText(result_frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
        
        info_y = 60
        cv2.putText(result_frame, f"HSV: [{self.yellow_h_min},{self.yellow_s_min},{self.yellow_v_min}]", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 25
        cv2.putText(result_frame, f"     [{self.yellow_h_max},{self.yellow_s_max},{self.yellow_v_max}]", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 25
        cv2.putText(result_frame, f"Height: {self.yellow_stop_height}%", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        info_y += 25
        cv2.putText(result_frame, f"Contour Len: {contour_length}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if cnt > 0:
            info_y += 25
            cv2.putText(result_frame, f"Aver Y: {aver_y}/{crop_gray.shape[0]}", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
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
        self.get_logger().info('📋 当前黄线检测参数：')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'minYellow = np.array([{self.yellow_h_min}, {self.yellow_s_min}, {self.yellow_v_min}])')
        self.get_logger().info(f'maxYellow = np.array([{self.yellow_h_max}, {self.yellow_s_max}, {self.yellow_v_max}])')
        self.get_logger().info(f'yellow_stop_height = {self.yellow_stop_height / 100.0}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📝 复制以上参数到 image_detector.py 的第21-22行')
        self.get_logger().info('   复制 yellow_stop_height 参数到 control_node.py')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    node = DebugYellowHSV()
    
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

