#!/usr/bin/env python3
"""
ROS2 图像检测节点 - 兼容您车上的系统
功能: 红灯检测、A/B标识牌识别、黄线检测
话题: 订阅 /image_raw, 发布 /image_detection
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from rclpy.qos import QoSProfile
import numpy as np
import copy
import os
import glob

# =============HSV颜色阈值=============
minRed = np.array([0, 100, 200])
maxRed = np.array([255, 255, 255])
minYellow = np.array([14, 44, 173])  # 用户调试的完美参数：H[14,114] S[63,192] V[175,225]
maxYellow = np.array([119, 216, 255])
lower_blue = np.array([100, 90, 50])
upper_blue = np.array([130, 255, 255])


class ImageDetector(Node):
    def __init__(self):
        super().__init__('image_detector')
        
        # 声明参数
        self.declare_parameter('enable_red_detect', True)  # 是否检测红灯
        self.declare_parameter('enable_ab_detect', True)   # 是否检测AB
        self.declare_parameter('enable_yellow_detect', False)  # 是否检测黄线
        self.declare_parameter('yellow_stop_height', 0.5)
        self.declare_parameter('enable_ab_parking_detect', False)  # 是否检测AB车库区域
        self.declare_parameter('ab_parking_min_pixels', 6000)  # AB库黄色区域最小白色像素数（只有≥此值才判定为进入停车区）
        self.declare_parameter('red_filter_x', 500)  # 红灯x坐标过滤
        self.declare_parameter('min_red_diameter', 19)  # 最小红灯直径（像素），只有直径>=此值才判定为检测到
        self.declare_parameter('show_debug', False)  # 是否显示调试窗口
        self.declare_parameter('show_ab_parking_only', False)  # 是否只显示AB车库检测窗口（隐藏其他窗口）
        self.declare_parameter('show_red_light_only', False)  # 是否只显示红灯检测窗口（隐藏其他窗口）
        self.declare_parameter('show_ab_only', False)  # 是否只显示AB标识检测窗口（HSV掩码和检测结果）
        
        # A/B检测HSV阈值参数（可调）
        self.declare_parameter('blue_h_min', 105)   # 蓝色H最小值
        self.declare_parameter('blue_h_max', 140)   # 蓝色H最大值
        self.declare_parameter('blue_s_min', 59)    # 蓝色S最小值
        self.declare_parameter('blue_s_max', 193)   # 蓝色S最大值
        self.declare_parameter('blue_v_min', 20)     # 蓝色V最小值
        self.declare_parameter('blue_v_max', 255)    # 蓝色V最大值
        self.declare_parameter('use_ring_detection', True)  # 是否使用圆环检测方法
        self.declare_parameter('ring_min_radius', 30)  # 圆环最小半径（像素）
        self.declare_parameter('ring_max_radius', 200)  # 圆环最大半径（像素）
        self.declare_parameter('save_debug_image', False)  # 是否保存调试图像
        self.declare_parameter('judge_threshold', 2/3)  # A/B判断阈值（0-1，默认2/3≈0.667，与原版一致）
        self.declare_parameter('use_original_algorithm', True)  # 是否使用原版循环算法
        self.declare_parameter('use_template_matching', True)  # 是否使用模板匹配
        self.declare_parameter('template_match_threshold', 0.6)  # 模板匹配阈值（0-1，相似度）
        self.declare_parameter('template_dir', os.path.expanduser('~/ab_templates'))  # 模板目录
        
        # 获取参数
        self.enable_red = self.get_parameter('enable_red_detect').value
        self.enable_ab = self.get_parameter('enable_ab_detect').value
        self.enable_yellow = self.get_parameter('enable_yellow_detect').value
        self.yellow_stop_height = self.get_parameter('yellow_stop_height').value
        self.enable_ab_parking = self.get_parameter('enable_ab_parking_detect').value
        self.ab_parking_min_pixels = self.get_parameter('ab_parking_min_pixels').value
        self.red_filter_x = self.get_parameter('red_filter_x').value
        self.min_red_diameter = self.get_parameter('min_red_diameter').value
        self.show_debug = self.get_parameter('show_debug').value
        self.show_ab_parking_only = self.get_parameter('show_ab_parking_only').value
        self.show_red_light_only = self.get_parameter('show_red_light_only').value
        self.show_ab_only = self.get_parameter('show_ab_only').value
        self.save_debug_image = self.get_parameter('save_debug_image').value
        
        # A/B检测HSV阈值（动态可调）
        self.blue_h_min = self.get_parameter('blue_h_min').value
        self.blue_h_max = self.get_parameter('blue_h_max').value
        self.blue_s_min = self.get_parameter('blue_s_min').value
        self.blue_s_max = self.get_parameter('blue_s_max').value
        self.blue_v_min = self.get_parameter('blue_v_min').value
        self.blue_v_max = self.get_parameter('blue_v_max').value
        self.judge_threshold = self.get_parameter('judge_threshold').value
        self.use_original_algorithm = self.get_parameter('use_original_algorithm').value
        self.use_ring_detection = self.get_parameter('use_ring_detection').value
        self.ring_min_radius = self.get_parameter('ring_min_radius').value
        self.ring_max_radius = self.get_parameter('ring_max_radius').value
        self.use_template_matching = self.get_parameter('use_template_matching').value
        self.template_match_threshold = self.get_parameter('template_match_threshold').value
        self.template_dir = self.get_parameter('template_dir').value
        
        # 加载模板
        self.templates_A = []
        self.templates_B = []
        if self.use_template_matching:
            self.load_templates()
        
        # 更新HSV阈值数组（动态更新）
        self.update_blue_thresholds()
        
        # 状态变量
        self.bridge = CvBridge()
        self.count = 0
        self.global_confidence = [0, 0]  # [A置信度, B置信度]
        self.combo = 10
        self.last_ab_result = 0
        self.imaged_yellow = False
        
        # 调试窗口初始化（如果启用）
        if self.show_debug:
            if self.show_red_light_only:
                # 只显示红灯检测窗口（HSV掩码和检测结果）
                cv2.namedWindow("5. 红灯-HSV掩码", cv2.WINDOW_NORMAL)
                cv2.namedWindow("5. 红灯-检测结果", cv2.WINDOW_NORMAL)
            elif self.show_ab_parking_only:
                # 只显示AB车库检测窗口
                cv2.namedWindow("AB车库检测-处理后图像(只保留竖线)", cv2.WINDOW_NORMAL)
                cv2.namedWindow("AB车库检测-原图", cv2.WINDOW_NORMAL)
            elif self.show_ab_only:
                # 只显示AB标识检测窗口（HSV掩码和检测结果）
                cv2.namedWindow("AB标识-原始画面", cv2.WINDOW_NORMAL)
                cv2.namedWindow("AB标识-HSV掩码", cv2.WINDOW_NORMAL)
                cv2.namedWindow("AB标识-轮廓标注", cv2.WINDOW_NORMAL)
                cv2.namedWindow("AB标识-检测结果", cv2.WINDOW_NORMAL)
            else:
                # 显示所有调试窗口
                # 黄线检测：4个窗口（黑白显示方式）
                cv2.namedWindow("1. 黄线-原始画面", cv2.WINDOW_NORMAL)
                cv2.namedWindow("2. 黄线-HSV掩码", cv2.WINDOW_NORMAL)
                cv2.namedWindow("3. 黄线-形态学处理后", cv2.WINDOW_NORMAL)
                cv2.namedWindow("4. 黄线-检测结果", cv2.WINDOW_NORMAL)
                
                # 红灯检测窗口（类似黄线检测的显示方式）
                cv2.namedWindow("5. 红灯-原始画面", cv2.WINDOW_NORMAL)
                cv2.namedWindow("5. 红灯-HSV掩码", cv2.WINDOW_NORMAL)
                cv2.namedWindow("5. 红灯-检测结果", cv2.WINDOW_NORMAL)
                
                # A/B检测窗口
                cv2.namedWindow("6. A/B检测", cv2.WINDOW_NORMAL)
                
                # 综合检测结果
                cv2.namedWindow("7. 综合检测结果", cv2.WINDOW_NORMAL)
        
        # QoS配置 (与您的line_follow2.py一致)
        qos = QoSProfile(depth=10)
        
        # 订阅摄像头 (使用您车上的话题名)
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',  # ← 您车上使用的话题
            self.image_callback,
            qos
        )
        
        # 发布检测结果
        self.detection_pub = self.create_publisher(
            Int16MultiArray,
            '/image_detection',  # 发布 [红灯, A/B, 黄线, AB车库方向]
            qos
        )
        
        # AB车库检测状态
        self.ab_parking_detected = False
        self.ab_parking_line_offset = 0.0  # 竖线位置偏移（-1.0到1.0，负数=左侧，正数=右侧）
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ 图像检测节点已启动!')
        self.get_logger().info(f'   订阅话题: /image_raw')
        self.get_logger().info(f'   发布话题: /image_detection')
        self.get_logger().info(f'   红灯检测: {"开启" if self.enable_red else "关闭"}')
        self.get_logger().info(f'   A/B识别: {"开启" if self.enable_ab else "关闭"}')
        if self.enable_ab:
            self.get_logger().info(f'   蓝色HSV阈值: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]')
            self.get_logger().info(f'   A/B判断阈值: {self.judge_threshold:.3f} (越小越严格，A更容易识别)')
            if self.use_template_matching:
                self.get_logger().info(f'   模板匹配: 开启 (阈值={self.template_match_threshold:.3f}, A模板={len(self.templates_A)}个, B模板={len(self.templates_B)}个)')
        self.get_logger().info(f'   黄线检测: {"开启" if self.enable_yellow else "关闭"}')
        self.get_logger().info(f'   保存调试图像: {"开启" if self.save_debug_image else "关闭"}')
        self.get_logger().info('=' * 60)
    
    def update_blue_thresholds(self):
        """更新蓝色HSV阈值（动态读取参数）"""
        global lower_blue, upper_blue
        # 动态读取参数（支持运行时调整）
        try:
            self.blue_h_min = self.get_parameter('blue_h_min').value
            self.blue_h_max = self.get_parameter('blue_h_max').value
            self.blue_s_min = self.get_parameter('blue_s_min').value
            self.blue_s_max = self.get_parameter('blue_s_max').value
            self.blue_v_min = self.get_parameter('blue_v_min').value
            self.blue_v_max = self.get_parameter('blue_v_max').value
        except:
            pass  # 如果参数不存在，使用初始值
        
        lower_blue = np.array([self.blue_h_min, self.blue_s_min, self.blue_v_min])
        upper_blue = np.array([self.blue_h_max, self.blue_s_max, self.blue_v_max])
    
    def load_templates(self):
        """从指定目录加载所有模板"""
        template_dir_A = os.path.join(self.template_dir, 'A')
        template_dir_B = os.path.join(self.template_dir, 'B')
        
        # 加载A模板
        if os.path.exists(template_dir_A):
            template_files_A = glob.glob(os.path.join(template_dir_A, 'template_A_*.png'))
            for template_file in template_files_A:
                template = cv2.imread(template_file, cv2.IMREAD_GRAYSCALE)
                if template is not None:
                    self.templates_A.append(template)
                    self.get_logger().info(f'📁 加载A模板: {os.path.basename(template_file)} ({template.shape[1]}x{template.shape[0]})')
        
        # 加载B模板
        if os.path.exists(template_dir_B):
            template_files_B = glob.glob(os.path.join(template_dir_B, 'template_B_*.png'))
            for template_file in template_files_B:
                template = cv2.imread(template_file, cv2.IMREAD_GRAYSCALE)
                if template is not None:
                    self.templates_B.append(template)
                    self.get_logger().info(f'📁 加载B模板: {os.path.basename(template_file)} ({template.shape[1]}x{template.shape[0]})')
        
        # 也加载根目录下的通用模板
        if os.path.exists(self.template_dir):
            template_files_general = glob.glob(os.path.join(self.template_dir, 'template_general_*.png'))
            for template_file in template_files_general:
                template = cv2.imread(template_file, cv2.IMREAD_GRAYSCALE)
                if template is not None:
                    # 根据文件名判断是A还是B（如果文件名包含A或B）
                    filename = os.path.basename(template_file).lower()
                    if 'a' in filename and 'b' not in filename:
                        self.templates_A.append(template)
                        self.get_logger().info(f'📁 加载通用A模板: {os.path.basename(template_file)}')
                    elif 'b' in filename:
                        self.templates_B.append(template)
                        self.get_logger().info(f'📁 加载通用B模板: {os.path.basename(template_file)}')
        
        self.get_logger().info(f'📊 模板加载完成: A模板={len(self.templates_A)}个, B模板={len(self.templates_B)}个')
        
        if len(self.templates_A) == 0 and len(self.templates_B) == 0:
            self.get_logger().warn(f'⚠️  未找到任何模板！请先使用拍照程序拍摄模板。')
            self.get_logger().warn(f'   模板目录: {self.template_dir}')
    
    def template_match(self, binary_image):
        """
        使用模板匹配识别A/B
        
        Returns:
            (result, best_score_A, best_score_B)
            result: 0=未检测到, 1=A, 2=B
        """
        if len(self.templates_A) == 0 and len(self.templates_B) == 0:
            return 0, 0.0, 0.0
        
        best_score_A = 0.0
        best_score_B = 0.0
        matched_A_count = 0  # 实际参与匹配的A模板数量
        matched_B_count = 0  # 实际参与匹配的B模板数量
        
        # 匹配A模板（多尺度）
        for template in self.templates_A:
            # 如果模板比图像大，跳过
            if template.shape[0] > binary_image.shape[0] or template.shape[1] > binary_image.shape[1]:
                continue
            
            matched_A_count += 1
            # 多尺度匹配
            scales = [0.8, 0.9, 1.0, 1.1, 1.2]
            for scale in scales:
                template_scaled = cv2.resize(template, None, fx=scale, fy=scale)
                if template_scaled.shape[0] > binary_image.shape[0] or template_scaled.shape[1] > binary_image.shape[1]:
                    continue
                
                result = cv2.matchTemplate(binary_image, template_scaled, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, _ = cv2.minMaxLoc(result)
                best_score_A = max(best_score_A, max_val)
        
        # 匹配B模板（多尺度）
        for template in self.templates_B:
            # 如果模板比图像大，跳过
            if template.shape[0] > binary_image.shape[0] or template.shape[1] > binary_image.shape[1]:
                continue
            
            matched_B_count += 1
            # 多尺度匹配
            scales = [0.8, 0.9, 1.0, 1.1, 1.2]
            for scale in scales:
                template_scaled = cv2.resize(template, None, fx=scale, fy=scale)
                if template_scaled.shape[0] > binary_image.shape[0] or template_scaled.shape[1] > binary_image.shape[1]:
                    continue
                
                result = cv2.matchTemplate(binary_image, template_scaled, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, _ = cv2.minMaxLoc(result)
                best_score_B = max(best_score_B, max_val)
        
        # 判断结果
        if best_score_A >= self.template_match_threshold and best_score_A > best_score_B:
            return 1, best_score_A, best_score_B  # A
        elif best_score_B >= self.template_match_threshold and best_score_B > best_score_A:
            return 2, best_score_A, best_score_B  # B
        else:
            return 0, best_score_A, best_score_B  # 未检测到
    
    def redLightDetect(self, frame):
        """
        红灯检测 - 修正版
        包含：
        1. 双区间HSV过滤 (0-10 和 160-180)，防止把黄灯当红灯
        2. 正确的形态学处理 (3x3核，去噪+膨胀)，不再使用扁长的核
        3. 严格的窗口显示控制
        """
        detected = False
        
        # 1. HSV转换
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # === 修改点 1: HSV阈值改为双区间 (更精准，排除黄光干扰) ===
        # 区间1: 0-10 (红色一端)
        lower_red1 = np.array([0, 100, 100])   
        upper_red1 = np.array([10, 255, 255])
        # 区间2: 160-180 (红色另一端)
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        mask_red = cv2.add(mask1, mask2) # 合并掩码
        
        # === 修改点 2: 形态学处理 (参考黄线逻辑，但参数适合红灯) ===
        # 原代码用了 (1,3) 扁核且腐蚀6次，太极端了，容易把灯腐蚀没
        # 现改为 (3,3) 正方核，适合圆形物体
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        
        # 开运算: 先腐蚀后膨胀 -> 去除细小噪点
        threshed_img_smooth_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=1)
        # 膨胀: 让剩下的红灯光斑变大、变实心，方便轮廓识别
        threshed_img_smooth_red = cv2.dilate(threshed_img_smooth_red, kernel, iterations=2)
        
        # 3. 霍夫圆检测 (保持原有逻辑，但输入图像质量更好了)
        circle_detected = False
        valid_circles = []
        max_diameter = 0  # 记录最大直径
        if threshed_img_smooth_red is not None:
            circles = cv2.HoughCircles(
                threshed_img_smooth_red,
                cv2.HOUGH_GRADIENT,
                1, 20,
                param1=50,
                param2=15,      # 稍微降低阈值，更容易检测到圆
                minRadius=3,    # 允许更小的红灯
                maxRadius=100
            )
            
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    x, y, r = int(i[0]), int(i[1]), int(i[2])
                    # 过滤条件：X坐标
                    if x >= self.red_filter_x:
                        diameter = r * 2  # 计算直径
                        valid_circles.append((x, y, r, diameter))
                        circle_detected = True
                        max_diameter = max(max_diameter, diameter)
        
        # 4. 轮廓检测 (作为补充)
        contours1, _ = cv2.findContours(
            threshed_img_smooth_red, # 使用处理后的图，而不是原始mask
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE
        )
        
        contour_detected = False
        valid_contours = []
        for cnt in contours1:
            (x, y, w, h) = cv2.boundingRect(cnt)
            # 过滤条件：面积、位置、长宽比
            aspect_ratio = float(w) / h
            # 面积不能太小，必须在右侧，长宽比接近1(正方形或圆形)
            if w*h > 30 and x >= self.red_filter_x and 0.5 < aspect_ratio < 2.0:
                # 计算等效直径（使用长宽的平均值作为直径估算）
                equivalent_diameter = (w + h) / 2.0
                valid_contours.append((x, y, w, h, equivalent_diameter))
                contour_detected = True
                # 更新最大直径（取圆圈和轮廓中的最大值）
                max_diameter = max(max_diameter, equivalent_diameter)
        
        # 5. 综合判断（需要满足直径要求）
        # 只有直径>=最小直径阈值才判定为检测到红灯
        detected = (circle_detected or contour_detected) and max_diameter >= self.min_red_diameter
        
        # 打印直径信息到控制台
        if (circle_detected or contour_detected) and max_diameter > 0:
            if detected:
                self.get_logger().info(f'🔴 检测到红灯 - 直径: {max_diameter:.1f} 像素 (半径: {max_diameter/2:.1f}) >= {self.min_red_diameter}像素，立即停车！')
            else:
                self.get_logger().debug(f'⚠️  检测到红灯但直径: {max_diameter:.1f} 像素 < {self.min_red_diameter}像素，未达到停车阈值')
        
        # === 修改点 3: 调试显示逻辑 (严格遵守只显示红灯窗口的指令) ===
        if self.show_debug:
            # 只有在"非"只显示红灯模式下，才显示原始画面窗口
            if not self.show_red_light_only and not self.show_ab_parking_only:
                cv2.imshow("5. 红灯-原始画面", frame)
            
            # 只要不是"只显示AB"，就显示红灯调试窗口
            if self.show_red_light_only or not self.show_ab_parking_only:
                # 窗口A：HSV掩码 (黑白)
                cv2.imshow("5. 红灯-HSV掩码", threshed_img_smooth_red)
                
                # 窗口B：检测结果 (彩色)
                result_frame = frame.copy()
                # 画圆并显示直径
                for circle_data in valid_circles:
                    if len(circle_data) == 4:
                        x, y, r, diameter = circle_data
                    else:
                        # 兼容旧格式
                        x, y, r = circle_data
                        diameter = r * 2
                    cv2.circle(result_frame, (x, y), r, (0, 255, 0), 2)
                    # 显示半径和直径
                    cv2.putText(result_frame, f"R:{r}", (x, y-r-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(result_frame, f"D:{diameter:.0f}", (x, y+r+15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # 画框并显示等效直径
                for contour_data in valid_contours:
                    if len(contour_data) == 5:
                        x, y, w, h, equivalent_diameter = contour_data
                    else:
                        # 兼容旧格式
                        x, y, w, h = contour_data
                        equivalent_diameter = (w + h) / 2.0
                    cv2.rectangle(result_frame, (x, y), (x+w, y+h), (0, 255, 255), 1)
                    cv2.putText(result_frame, f"D:{equivalent_diameter:.0f}", (x, y-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                status = "RED LIGHT DETECTED" if detected else "No Red Light"
                color = (0, 0, 255) if detected else (0, 255, 0)
                cv2.putText(result_frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                # 显示直径信息
                if (circle_detected or contour_detected) and max_diameter > 0:
                    info_y = 70
                    # 显示当前检测到的直径
                    diameter_text = f"Diameter: {max_diameter:.1f} px"
                    diameter_color = (0, 255, 0) if max_diameter >= self.min_red_diameter else (0, 165, 255)
                    cv2.putText(result_frame, diameter_text, 
                               (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, diameter_color, 2)
                    info_y += 30
                    # 显示阈值信息
                    threshold_text = f"Threshold: >= {self.min_red_diameter} px"
                    if max_diameter >= self.min_red_diameter:
                        threshold_text += " ✓ STOP!"
                        threshold_color = (0, 0, 255)  # 红色
                    else:
                        threshold_text += f" (Need {self.min_red_diameter - max_diameter:.1f} more)"
                        threshold_color = (0, 165, 255)  # 橙色
                    cv2.putText(result_frame, threshold_text, 
                               (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, threshold_color, 2)
                    info_y += 30
                    # 根据直径估算距离提示（需要根据实际标定）
                    if max_diameter > 80:
                        distance_hint = "Very Close"
                        hint_color = (0, 0, 255)  # 红色
                    elif max_diameter > 50:
                        distance_hint = "Close"
                        hint_color = (0, 165, 255)  # 橙色
                    elif max_diameter > 30:
                        distance_hint = "Medium"
                        hint_color = (0, 255, 255)  # 黄色
                    else:
                        distance_hint = "Far"
                        hint_color = (0, 255, 0)  # 绿色
                    cv2.putText(result_frame, f"Distance: {distance_hint}", 
                               (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, hint_color, 2)
                
                cv2.imshow("5. 红灯-检测结果", result_frame)
        
        return 1 if detected else 0
    
    def yellowStopDetect(self, frame):
        """黄线停止检测 - 参考debug_yellow_hsv.py的显示方式（黑白显示）"""
        # 1. 显示原始画面
        if self.show_debug and not self.show_ab_parking_only and not self.show_red_light_only:
            cv2.imshow("1. 黄线-原始画面", frame)
        
        # 2. HSV转换和黄线提取
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(img_hsv, minYellow, maxYellow)
        if self.show_debug and not self.show_ab_parking_only and not self.show_red_light_only:
            cv2.imshow("2. 黄线-HSV掩码", mask_yellow)  # 黑白显示
        
        # 3. 形态学处理（与image_detector.py完全一致）
        thresh = cv2.threshold(mask_yellow, 254, 255, 0)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
        threshed_img_smooth = cv2.erode(thresh, kernel, iterations=1)
        threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations=2)
        if self.show_debug and not self.show_ab_parking_only and not self.show_red_light_only:
            cv2.imshow("3. 黄线-形态学处理后", threshed_img_smooth)  # 黑白显示
        
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
        
        # 5. 显示检测结果（参考debug_yellow_hsv.py）
        detected = False
        if cnt > 0:
            aver_y = aver_y // cnt
            yellow_stop_height_ratio = self.yellow_stop_height
            
            # 判断是否检测到黄线
            if (aver_y >= crop_gray.shape[0] * yellow_stop_height_ratio and 
                aver_y != crop_gray.shape[0] and 
                contour_length >= 12):
                detected = True
                if not self.imaged_yellow:
                    self.imaged_yellow = True
                    self.get_logger().info('🟡 检测到黄线停止区域!')
        
        if self.show_debug:
            result_frame = frame.copy()
            
            # 绘制裁剪区域
            cv2.rectangle(result_frame, 
                         (quarter_col_left, 0), 
                         (quarter_col_right, frame.shape[0]), 
                         (255, 0, 255), 2)
            
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
            
            # 显示参数信息（参考debug_yellow_hsv.py）
            cv2.putText(result_frame, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
            
            info_y = 60
            cv2.putText(result_frame, f"HSV: [{minYellow[0]},{minYellow[1]},{minYellow[2]}]", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            info_y += 25
            cv2.putText(result_frame, f"     [{maxYellow[0]},{maxYellow[1]},{maxYellow[2]}]", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            info_y += 25
            cv2.putText(result_frame, f"Height: {self.yellow_stop_height*100:.0f}%", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            info_y += 25
            cv2.putText(result_frame, f"Contour Len: {contour_length}", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            if cnt > 0:
                info_y += 25
                cv2.putText(result_frame, f"Aver Y: {aver_y}/{crop_gray.shape[0]}", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            if not self.show_ab_parking_only and not self.show_red_light_only:
                cv2.imshow("4. 黄线-检测结果", result_frame)
        
        return 1 if detected else 0
    
    def detect_blue_cones(self, frame):
        """
        检测正前方两个蓝色锥桶（直角弯标志）
        要求：一左一右，大小与距离一致
        返回：0=未检测，1=检测到
        """
        try:
            h, w = frame.shape[:2]
            
            # 只检测图像下方2/3区域（正前方）
            roi_y_start = h // 3
            roi = frame[roi_y_start:, :]
            roi_h, roi_w = roi.shape[:2]
            
            # 转换为HSV
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # 更新蓝色阈值
            self.update_blue_thresholds()
            global lower_blue, upper_blue
            
            # 创建蓝色掩码
            mask = cv2.inRange(hsv_roi, lower_blue, upper_blue)
            
            # 形态学操作，去除噪声
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) < 2:
                return 0
            
            # 过滤轮廓：面积足够大，且形状接近圆形/锥形
            valid_cones = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 100:  # 面积太小，忽略
                    continue
                
                # 计算外接矩形
                x, y, w_rect, h_rect = cv2.boundingRect(contour)
                
                # 计算宽高比（锥桶应该比较接近正方形或略高）
                aspect_ratio = h_rect / w_rect if w_rect > 0 else 0
                if aspect_ratio < 0.5 or aspect_ratio > 2.5:  # 宽高比不合理
                    continue
                
                # 计算中心点（相对于ROI）
                center_x = x + w_rect // 2
                center_y = y + h_rect // 2
                
                # 计算等效直径（用于判断大小）
                equivalent_diameter = np.sqrt(4 * area / np.pi)
                
                valid_cones.append({
                    'center_x': center_x,
                    'center_y': center_y,
                    'area': area,
                    'diameter': equivalent_diameter,
                    'x': x,
                    'y': y,
                    'w': w_rect,
                    'h': h_rect
                })
            
            if len(valid_cones) < 2:
                return 0
            
            # 按X坐标排序（从左到右）
            valid_cones.sort(key=lambda c: c['center_x'])
            
            # 检查是否有两个锥桶一左一右
            # 要求：一个在左半部分，一个在右半部分
            left_cones = [c for c in valid_cones if c['center_x'] < roi_w // 2]
            right_cones = [c for c in valid_cones if c['center_x'] >= roi_w // 2]
            
            if len(left_cones) == 0 or len(right_cones) == 0:
                return 0
            
            # 选择最靠近中心线的左右锥桶
            left_cone = max(left_cones, key=lambda c: c['center_x'])  # 左侧最靠右的
            right_cone = min(right_cones, key=lambda c: c['center_x'])  # 右侧最靠左的
            
            # 检查大小一致性（直径差异不超过30%）
            size_ratio = min(left_cone['diameter'], right_cone['diameter']) / max(left_cone['diameter'], right_cone['diameter'])
            if size_ratio < 0.7:
                return 0
            
            # 检查距离一致性（Y坐标差异不超过20%）
            y_diff = abs(left_cone['center_y'] - right_cone['center_y'])
            avg_y = (left_cone['center_y'] + right_cone['center_y']) / 2
            if avg_y > 0:
                y_ratio = 1 - (y_diff / avg_y)
                if y_ratio < 0.8:  # Y坐标差异太大
                    return 0
            
            # 检查水平距离（两个锥桶应该在合理的距离范围内）
            x_distance = right_cone['center_x'] - left_cone['center_x']
            if x_distance < roi_w * 0.2 or x_distance > roi_w * 0.8:  # 距离太近或太远
                return 0
            
            # 所有条件满足，检测到直角弯标志
            if self.show_debug:
                debug_frame = roi.copy()
                # 绘制检测到的锥桶
                cv2.rectangle(debug_frame, 
                            (left_cone['x'], left_cone['y']),
                            (left_cone['x'] + left_cone['w'], left_cone['y'] + left_cone['h']),
                            (255, 0, 0), 2)
                cv2.rectangle(debug_frame,
                            (right_cone['x'], right_cone['y']),
                            (right_cone['x'] + right_cone['w'], right_cone['y'] + right_cone['h']),
                            (255, 0, 0), 2)
                cv2.putText(debug_frame, "Left", 
                          (left_cone['x'], left_cone['y'] - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(debug_frame, "Right",
                          (right_cone['x'], right_cone['y'] - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(debug_frame, f"Blue Cones Detected (Right Angle Turn)",
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("8. 蓝色锥桶检测", debug_frame)
            
            return 1
            
        except Exception as e:
            self.get_logger().warn(f'蓝色锥桶检测异常: {e}')
            return 0
    
    def detectAbParkingZone(self, frame):
        """
        AB车库区域检测 - 黄色区域像素统计版
        核心逻辑：统计整个图像中连通的白色像素总数（T型黄色胶带面积很大）
        当白色像素总数达到阈值时，认为进入停车区
        """
        # 1. HSV转换和黄线提取 (复用参数)
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(img_hsv, minYellow, maxYellow)
        
        # 2. 基础去噪
        _, thresh = cv2.threshold(mask_yellow, 254, 255, cv2.THRESH_BINARY)
        
        # 3. 过滤横线（包括畸变成圆弧的横线）
        # 使用竖向核进行开运算：只有竖长条才能保留，横线会被去除
        # 核的大小：宽小（如3-5），高大（如15-20），这样可以过滤掉横线和圆弧
        vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,50))  # 宽4，高18
        
        # 开运算：先腐蚀后膨胀，去除横线
        # 只有竖长条（高度>18像素）才能保留，横线和短线段会被去除
        vertical_only = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, vertical_kernel, iterations=1)
        
        # 4. 形态学处理（去噪，让竖线区域更连续）
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 5))
        processed_img = cv2.morphologyEx(vertical_only, cv2.MORPH_CLOSE, kernel, iterations=2)  # 连接断裂的竖线
        
        # 5. 统计整个图像中连通的白色像素总数（只统计竖线部分，横线已被过滤）
        total_white_pixels = np.sum(processed_img == 255)
        
        # 6. 实时打印白色像素总数（只统计竖线，横线已过滤）
        self.get_logger().info(f'📊 AB库黄色区域(竖线): {total_white_pixels} 个白色像素 (阈值: ≥{self.ab_parking_min_pixels})')
        
        height, width = processed_img.shape
        img_center_x = width // 2
        
        # 7. 判断是否进入停车区（基于总像素数）
        detected = False
        line_offset = 0.0
        
        # 像素阈值：只有≥此值才认为进入停车区（可配置参数）
        MIN_WHITE_PIXELS = self.ab_parking_min_pixels
        
        # 找到所有白色像素的坐标（用于计算重心和调试显示）
        white_pixel_coords = np.column_stack(np.where(processed_img == 255))
        
        if total_white_pixels >= MIN_WHITE_PIXELS:
            # 进入停车区
            detected = True
            
            if len(white_pixel_coords) > 0:
                # 计算重心的X坐标
                center_x = int(np.mean(white_pixel_coords[:, 1]))  # 列坐标（X）
                # 计算偏移量 (-1.0 左, 1.0 右)
                line_offset = (center_x - img_center_x) / (width / 2.0)
                line_offset = max(-1.0, min(1.0, line_offset))  # 限幅
            else:
                line_offset = 0.0
            
            # 保存到类变量
            self.ab_parking_line_offset = line_offset
            self.ab_parking_detected = True
            
            self.get_logger().warn(f'🅿️  进入AB停车区！白色像素: {total_white_pixels} >= {MIN_WHITE_PIXELS}, 偏移: {line_offset:.2f}')
        else:
            self.ab_parking_detected = False
            # 不打印警告，只打印信息（避免刷屏）
        
        # 8. 调试显示
        if self.show_debug:
            # 准备显示图像
            # display_img: 处理后的二值图转彩色（只显示竖线，横线已被过滤）
            display_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2BGR)
            # original_img: 原图
            original_img = frame.copy()
            
            # 画出图像中心线 (白色)
            cv2.line(display_img, (img_center_x, 0), (img_center_x, height), (255, 255, 255), 1)
            cv2.line(original_img, (img_center_x, 0), (img_center_x, height), (255, 255, 255), 1)

            # 如果检测到，绘制黄色区域的重心位置
            if detected and len(white_pixel_coords) > 0:
                center_x = int(np.mean(white_pixel_coords[:, 1]))
                center_y = int(np.mean(white_pixel_coords[:, 0]))
                
                # 绘制重心位置（红色十字）
                cv2.line(display_img, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 3)
                cv2.line(display_img, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 3)
                cv2.line(original_img, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 3)
                cv2.line(original_img, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 3)
                
                # 显示像素统计信息
                pixel_info = f"总像素: {total_white_pixels} (阈值: {MIN_WHITE_PIXELS})"
                status_color = (0, 0, 255) if detected else (0, 255, 0)
                cv2.putText(original_img, pixel_info, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
                
                offset_info = f"Offset: {line_offset:.2f}"
                cv2.putText(original_img, offset_info, (10, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # 未检测到，显示当前像素数
                pixel_info = f"总像素: {total_white_pixels} / {MIN_WHITE_PIXELS}"
                cv2.putText(original_img, pixel_info, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                
                # 显示进度条（可视化）
                progress = min(total_white_pixels / MIN_WHITE_PIXELS, 1.0)
                bar_width = 200
                bar_height = 20
                bar_x = 10
                bar_y = 60
                cv2.rectangle(original_img, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (100, 100, 100), -1)
                cv2.rectangle(original_img, (bar_x, bar_y), (bar_x + int(bar_width * progress), bar_y + bar_height), (0, 255, 0), -1)
                cv2.putText(original_img, f"{progress*100:.0f}%", (bar_x + bar_width + 10, bar_y + bar_height - 5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # 显示窗口
            cv2.imshow("AB车库检测-处理后图像(只保留竖线)", display_img)
            cv2.imshow("AB车库检测-原图", original_img)

        return detected, line_offset
    
    def crop_and_HSV(self, res_img):
        """裁剪并提取蓝色"""
        # 更新HSV阈值（从ROS参数读取，支持动态调整）
        self.update_blue_thresholds()
        
        quarter_row_upper = res_img.shape[0] - (res_img.shape[0] // 3)
        quarter_row_lower = res_img.shape[0]
        quarter_col_left = res_img.shape[1] // 4
        quarter_col_right = res_img.shape[1] - (res_img.shape[1] // 4)
        
        crop_mask_img = res_img[quarter_row_upper:quarter_row_lower, 
                                quarter_col_left:quarter_col_right]
        
        hsv_img = cv2.cvtColor(crop_mask_img, cv2.COLOR_BGR2HSV)
        mask_img = cv2.inRange(hsv_img, lower_blue, upper_blue)
        
        # 调试：保存蓝色掩码图像
        if self.save_debug_image:
            try:
                cv2.imwrite('/tmp/debug_blue_mask.png', mask_img)
                cv2.imwrite('/tmp/debug_crop_region.png', crop_mask_img)
            except:
                pass
        
        return mask_img
    
    def crop_min_range(self, crop_mask):
        """裁剪最小蓝色区域"""
        contours, _ = cv2.findContours(crop_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return crop_mask
        
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = -float('inf'), -float('inf')
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            min_x, min_y = min(min_x, x), min(min_y, y)
            max_x, max_y = max(max_x, x+w), max(max_y, y+h)
        
        if min_x == float('inf'):
            return crop_mask
        
        crop_img = crop_mask[min_y:max_y, min_x:max_x]
        
        ker = np.ones((6, 6), np.uint8)
        crop_img = cv2.morphologyEx(crop_img, cv2.MORPH_OPEN, ker)
        
        ker = np.ones((5, 5), np.uint8)
        crop_img = cv2.morphologyEx(crop_img, cv2.MORPH_CLOSE, ker)
        
        return crop_img
    
    def detect_ring_roi(self, mask_img, frame_shape):
        """
        检测圆环区域，返回ROI坐标
        使用霍夫圆检测或轮廓分析来找到圆环
        """
        # 方法1: 使用霍夫圆检测
        # 根据图像大小调整minDist
        img_height, img_width = mask_img.shape[:2]
        min_dist = min(img_width, img_height) // 3  # 最小距离设为图像尺寸的1/3
        
        circles = cv2.HoughCircles(
            mask_img,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=max(30, min_dist),  # 至少30像素，但不超过图像尺寸的1/3
            param1=50,
            param2=15,  # 降低阈值，更容易检测到圆
            minRadius=self.ring_min_radius,
            maxRadius=self.ring_max_radius
        )
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # 选择最大的圆（最可能是AB标志）
            best_circle = None
            max_radius = 0
            for i in circles[0, :]:
                x, y, r = int(i[0]), int(i[1]), int(i[2])
                if r > max_radius:
                    max_radius = r
                    best_circle = (x, y, r)
            
            if best_circle:
                x, y, r = best_circle
                # 扩展ROI区域（稍微扩大一点，确保包含完整标志）
                margin = int(r * 0.2)  # 20%的边距
                x1 = max(0, x - r - margin)
                y1 = max(0, y - r - margin)
                x2 = min(frame_shape[1], x + r + margin)
                y2 = min(frame_shape[0], y + r + margin)
                return (x1, y1, x2, y2), (x, y, r)
        
        # 方法2: 如果霍夫圆检测失败，使用轮廓分析
        contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # 找到最大的轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # 检查轮廓面积是否合理（圆环应该有一定大小）
            min_area = np.pi * self.ring_min_radius ** 2
            max_area = np.pi * self.ring_max_radius ** 2
            
            if min_area <= area <= max_area:
                # 计算最小外接圆
                (x, y), r = cv2.minEnclosingCircle(largest_contour)
                x, y, r = int(x), int(y), int(r)
                
                if self.ring_min_radius <= r <= self.ring_max_radius:
                    margin = int(r * 0.2)
                    x1 = max(0, x - r - margin)
                    y1 = max(0, y - r - margin)
                    x2 = min(frame_shape[1], x + r + margin)
                    y2 = min(frame_shape[0], y + r + margin)
                    return (x1, y1, x2, y2), (x, y, r)
        
        return None, None
    
    def detect_vertical_line(self, roi_mask):
        """
        检测竖线（用于识别B）
        B的特征：左侧有很粗很笔直的竖线
        """
        # 使用霍夫线变换检测竖线
        # 调整参数以适应不同大小的ROI
        min_line_length = max(20, int(roi_mask.shape[0] * 0.25))  # 至少20像素，或高度的25%
        lines = cv2.HoughLinesP(
            roi_mask,
            rho=1,
            theta=np.pi/180,
            threshold=max(15, int(roi_mask.shape[0] * 0.1)),  # 动态阈值
            minLineLength=min_line_length,
            maxLineGap=15  # 允许更大的间隙
        )
        
        if lines is not None:
            vertical_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 计算角度，检查是否接近垂直（±15度）
                angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
                # 垂直线的角度应该接近90度（75-105度之间）
                if 75 <= angle <= 105:  # 接近90度（垂直）
                    # 检查是否在左侧区域（左侧1/3）
                    if x1 < roi_mask.shape[1] / 3 or x2 < roi_mask.shape[1] / 3:
                        line_length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                        vertical_lines.append((line_length, (x1, y1, x2, y2)))
            
            if len(vertical_lines) > 0:
                # 找到最长的竖线
                vertical_lines.sort(reverse=True, key=lambda x: x[0])
                longest_line = vertical_lines[0]
                # 检查是否足够粗（长度足够）
                if longest_line[0] > roi_mask.shape[0] * 0.4:
                    return True, longest_line[1]
        
        return False, None
    
    def detect_triangle(self, roi_mask):
        """
        检测三角形（用于识别A）
        A的特征：中间有明显的白色（蓝色）三角形
        """
        # 查找轮廓
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            min_area = max(50, roi_mask.shape[0] * roi_mask.shape[1] * 0.01)  # 至少1%的面积
            if area < min_area:  # 太小忽略
                continue
            
            # 近似轮廓为多边形（允许更大的误差，适应非标准三角形）
            epsilon = 0.03 * cv2.arcLength(contour, True)  # 增加到3%
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # 检查是否是三角形（3个顶点）或接近三角形（3-5个顶点）
            if 3 <= len(approx) <= 5:  # 允许3-5个顶点（适应非标准三角形）
                # 检查是否在中间区域（中间1/2宽度，更宽松）
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    center_region_start = roi_mask.shape[1] / 4  # 更宽松：中间1/2区域
                    center_region_end = roi_mask.shape[1] * 3 / 4
                    
                    if center_region_start <= cx <= center_region_end:
                        # 检查三角形是否足够大（至少2%的面积）
                        if area > roi_mask.shape[0] * roi_mask.shape[1] * 0.02:
                            return True, approx
        
        return False, None
    
    def detect_ab_by_features(self, roi_mask, debug_frame=None, roi_coords=None):
        """
        基于特征识别A/B
        B: 左侧有竖线
        A: 中间有三角形
        """
        # 检测竖线（B的特征）
        has_vertical_line, line_coords = self.detect_vertical_line(roi_mask)
        
        # 检测三角形（A的特征）
        has_triangle, triangle_coords = self.detect_triangle(roi_mask)
        
        # 调试显示
        if self.show_debug and debug_frame is not None and roi_coords is not None:
            x1, y1, x2, y2 = roi_coords
            # 在原始图像上绘制ROI区域
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            if has_vertical_line and line_coords:
                # 绘制竖线（转换为原始图像坐标）
                lx1, ly1, lx2, ly2 = line_coords
                cv2.line(debug_frame, 
                        (x1 + lx1, y1 + ly1), 
                        (x1 + lx2, y1 + ly2), 
                        (255, 0, 0), 3)
            
            if has_triangle and triangle_coords is not None:
                # 绘制三角形（转换为原始图像坐标）
                triangle_points = triangle_coords.reshape(-1, 2)
                triangle_points = triangle_points + np.array([x1, y1])
                cv2.drawContours(debug_frame, [triangle_points], -1, (0, 255, 255), 2)
        
        # 判断逻辑
        if has_vertical_line and not has_triangle:
            return 2  # B
        elif has_triangle and not has_vertical_line:
            return 1  # A
        elif has_vertical_line and has_triangle:
            # 如果两者都有，优先判断竖线（B的特征更明显）
            return 2  # B
        else:
            return 0  # 未检测到
    
    def A_B_detect(self, frame):
        """A/B标识牌识别 - 增强版：支持圆环检测和特征识别"""
        detected_result = 0
        try:
            # 调试窗口显示准备
            debug_frame = None
            if self.show_debug:
                debug_frame = frame.copy()
            
            # 更新HSV阈值
            self.update_blue_thresholds()
            
            # 裁剪到检测区域（下半部分画面，右侧去掉五分之一，左侧不变）
            half_row = frame.shape[0] // 2  # 从中间开始
            col_left = 0  # 左侧不变
            col_right = frame.shape[1] - (frame.shape[1] // 5)  # 右侧去掉五分之一
            
            crop_frame = frame[half_row:, col_left:col_right]
            
            # HSV转换和掩码提取
            hsv_img = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV)
            mask_img = cv2.inRange(hsv_img, lower_blue, upper_blue)
                
            # 显示HSV掩码窗口（如果启用AB专用窗口）
            if self.show_debug and self.show_ab_only:
                cv2.imshow("AB标识-原始画面", crop_frame)
                cv2.imshow("AB标识-HSV掩码", mask_img)  # 黑白显示
            
            # 调试：检查是否检测到蓝色区域
            blue_pixel_count = np.sum(mask_img > 0)
            if blue_pixel_count < 100:
                if self.show_debug:
                    # 绘制裁剪区域
                    cv2.rectangle(debug_frame, 
                                 (col_left, half_row), 
                                 (col_right, frame.shape[0]), 
                                 (255, 0, 0), 2)
                    cv2.putText(debug_frame, "未检测到蓝色区域", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(debug_frame, f"蓝色像素数: {blue_pixel_count}", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                    cv2.putText(debug_frame, f"HSV阈值: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]", 
                               (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                    if self.show_ab_only:
                        cv2.imshow("AB标识-检测结果", debug_frame)
                    elif not self.show_ab_parking_only and not self.show_red_light_only:
                        cv2.imshow("6. A/B检测", debug_frame)
                if self.save_debug_image:
                    self.get_logger().debug(f'蓝色像素太少: {blue_pixel_count}')
                return 0
            
            # ===== 基于连通轮廓数量的A/B识别方法 =====
            # 直接使用掩码进行轮廓检测（不进行形态学处理）
            thresh = cv2.threshold(mask_img, 254, 255, 0)[1]
            
            if self.show_debug:
                # 绘制裁剪区域
                cv2.rectangle(debug_frame, 
                             (col_left, half_row), 
                             (col_right, frame.shape[0]), 
                             (255, 0, 0), 2)
                # 在原始画面上叠加蓝色掩码（半透明）
                mask_colored = cv2.cvtColor(mask_img, cv2.COLOR_GRAY2BGR)
                crop_debug = debug_frame[half_row:, col_left:col_right]
                crop_debug = cv2.addWeighted(crop_debug, 0.7, mask_colored, 0.3, 0)
                debug_frame[half_row:, col_left:col_right] = crop_debug
            
            # 直接使用二值化后的图像（用于模板匹配）
            crop_image_1 = thresh.copy()
                
            # ===== 轮廓检测代码已注释（现在使用模板匹配方法）=====
            # # 提取轮廓数量（使用RETR_TREE检测内外轮廓）
            # # RETR_TREE: 检测所有轮廓，包括外部和内部轮廓（如A的三角形）
            # contours, hierarchy = cv2.findContours(crop_image_1, cv2.RETR_TREE, 
            #                                   cv2.CHAIN_APPROX_NONE)
            # contours = list(contours)
            #     
            # # 过滤掉太小的轮廓（噪声）
            # min_contour_area = crop_image_1.shape[0] * crop_image_1.shape[1] * 0.01  # 至少占1%的面积
            # valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
            # contour_count = len(valid_contours)
            # 
            # # 计算总面积
            # total_area = sum(cv2.contourArea(c) for c in valid_contours)
            # 
            # # 打印轮廓数量信息
            # self.get_logger().info(
            #     f'📊 检测到 {contour_count} 个轮廓 '
            #     f'(总面积: {total_area:.0f} 像素)'
            # )
            # 
            # # 打印每个轮廓的详细信息
            # for idx, cnt in enumerate(valid_contours):
            #     area = cv2.contourArea(cnt)
            #     self.get_logger().info(
            #         f'   轮廓{idx+1}: 面积={area:.0f} 像素, 周长={cv2.arcLength(cnt, True):.0f} 像素'
            #     )
            # 
            # # 在调试窗口中绘制所有轮廓
            # if self.show_debug:
            #     # 定义不同颜色用于绘制轮廓
            #     colors = [
            #         (0, 255, 0),    # 绿色 - 轮廓1
            #         (255, 0, 0),    # 蓝色 - 轮廓2
            #         (0, 0, 255),    # 红色 - 轮廓3
            #         (255, 255, 0),  # 青色 - 轮廓4
            #         (255, 0, 255),  # 洋红 - 轮廓5
            #         (0, 255, 255),  # 黄色 - 轮廓6
            #     ]
            #     
            #     # 创建轮廓绘制图像（在检测轮廓的二值图上绘制，转彩色显示）
            #     contour_draw_img = cv2.cvtColor(crop_image_1, cv2.COLOR_GRAY2BGR)
            #     
            #     # 绘制所有轮廓
            #     for idx, cnt in enumerate(valid_contours):
            #         color = colors[idx % len(colors)]
            #         # 绘制轮廓线条（线宽2）
            #         cv2.drawContours(contour_draw_img, [cnt], -1, color, 2)
            #         
            #         # 计算轮廓中心点
            #         M = cv2.moments(cnt)
            #         if M["m00"] != 0:
            #             cx = int(M["m10"] / M["m00"])
            #             cy = int(M["m01"] / M["m00"])
            #             
            #             # 在中心点绘制圆点
            #             cv2.circle(contour_draw_img, (cx, cy), 5, color, -1)
            #             
            #             # 标注轮廓编号和面积
            #             area = cv2.contourArea(cnt)
            #             label = f"#{idx+1}: {area:.0f}"
            #             cv2.putText(contour_draw_img, label, (cx-30, cy-10),
            #                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            #     
            #     # 在原始彩色图像上绘制轮廓（crop_image_1和crop_frame尺寸相同）
            #     for idx, cnt in enumerate(valid_contours):
            #         color = colors[idx % len(colors)]
            #         cv2.drawContours(crop_frame, [cnt], -1, color, 2)
            #         
            #         M = cv2.moments(cnt)
            #         if M["m00"] != 0:
            #             cx = int(M["m10"] / M["m00"])
            #             cy = int(M["m01"] / M["m00"])
            #             cv2.circle(crop_frame, (cx, cy), 5, color, -1)
            #             area = cv2.contourArea(cnt)
            #             label = f"#{idx+1}: {area:.0f}"
            #             cv2.putText(crop_frame, label, (cx-30, cy-10),
            #                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            #     
            #     # 更新原始画面窗口
            #     if self.show_ab_only:
            #         cv2.imshow("AB标识-原始画面", crop_frame)
            #     
            #     # 在HSV掩码窗口也绘制轮廓（mask_img和crop_image_1尺寸相同）
            #     if self.show_ab_only:
            #         mask_with_contours = cv2.cvtColor(mask_img, cv2.COLOR_GRAY2BGR)
            #         for idx, cnt in enumerate(valid_contours):
            #             color = colors[idx % len(colors)]
            #             cv2.drawContours(mask_with_contours, [cnt], -1, color, 2)
            #             
            #             M = cv2.moments(cnt)
            #             if M["m00"] != 0:
            #                 cx = int(M["m10"] / M["m00"])
            #                 cy = int(M["m01"] / M["m00"])
            #                 cv2.putText(mask_with_contours, f"#{idx+1}", (cx-15, cy),
            #                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            #         cv2.imshow("AB标识-HSV掩码", mask_with_contours)
            #         
            #         # 显示轮廓绘制图像（专门的轮廓窗口）
            #         cv2.imshow("AB标识-轮廓标注", contour_draw_img)
            # 
            # # 调试：保存裁剪后的图像
            # if self.save_debug_image and contour_count > 0:
            #     try:
            #         cv2.imwrite(f'/tmp/debug_crop_image_contours{contour_count}.png', crop_image_1)
            #     except:
            #         pass
            
            # ===== 模板匹配方法（唯一方法）=====
            detected_result = 0
            if self.use_template_matching and (len(self.templates_A) > 0 or len(self.templates_B) > 0):
                # 记录图像尺寸用于调试
                img_h, img_w = crop_image_1.shape[:2]
                self.get_logger().info(
                    f'🔍 开始模板匹配: 图像尺寸={img_w}x{img_h}, A模板={len(self.templates_A)}个, B模板={len(self.templates_B)}个'
                )
                match_result, score_A, score_B = self.template_match(crop_image_1)
                
                if match_result > 0:
                    sign = 'A' if match_result == 1 else 'B'
                    if self.show_debug:
                        cv2.putText(debug_frame, f"检测到标识牌: {sign} (模板匹配)", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                   (0, 255, 255) if match_result == 1 else (255, 0, 255), 2)
                        cv2.putText(debug_frame, f"匹配分数: A={score_A:.3f}, B={score_B:.3f}", 
                                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                        cv2.putText(debug_frame, f"HSV阈值: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]", 
                                       (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                        if self.show_ab_only:
                            cv2.imshow("AB标识-检测结果", debug_frame)
                        elif not self.show_ab_parking_only and not self.show_red_light_only:
                                cv2.imshow("2. A/B检测", debug_frame)
                        self.get_logger().info(
                        f'🔵 识别到标识牌: {sign} (模板匹配, A={score_A:.3f}, B={score_B:.3f})'
                        )
                    return match_result
                    else:
                        self.get_logger().info(
                        f'📊 模板匹配未成功: A={score_A:.3f}, B={score_B:.3f} (阈值={self.template_match_threshold:.3f})'
                    )
            
            # ===== 特征检测方法已注释（仅使用模板匹配）=====
            # # ===== 方法2: 基于特征检测判断A/B（备选）=====
            # # A的特征：中间有三角形
            # # B的特征：左侧有竖线
            # 
            # # 检测三角形（A的特征）
            # has_triangle, triangle_coords = self.detect_triangle(crop_image_1)
            # 
            # # 检测竖线（B的特征）
            # has_vertical_line, line_coords = self.detect_vertical_line(crop_image_1)
            # 
            # # 打印特征检测结果
            # self.get_logger().info(
            #     f'🔍 特征检测: 三角形={has_triangle}, 竖线={has_vertical_line}'
            # )
            # 
            # # 在调试窗口中绘制检测到的特征
            # if self.show_debug:
            #     # 绘制三角形
            #     if has_triangle and triangle_coords is not None:
            #         triangle_points = triangle_coords.reshape(-1, 2)
            #         cv2.drawContours(crop_frame, [triangle_points], -1, (0, 255, 255), 3)
            #         cv2.putText(crop_frame, "TRIANGLE", (10, crop_frame.shape[0] - 20),
            #                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            #         # 更新原始画面窗口
            #         if self.show_ab_only:
            #             cv2.imshow("AB标识-原始画面", crop_frame)
            #     
            #     # 绘制竖线
            #     if has_vertical_line and line_coords is not None:
            #         lx1, ly1, lx2, ly2 = line_coords
            #         cv2.line(crop_frame, (lx1, ly1), (lx2, ly2), (255, 0, 255), 3)
            #         cv2.putText(crop_frame, "VERTICAL LINE", (10, crop_frame.shape[0] - 50),
            #                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            #         # 更新原始画面窗口
            #         if self.show_ab_only:
            #             cv2.imshow("AB标识-原始画面", crop_frame)
            # 
            # # 判断逻辑
            # if has_triangle and not has_vertical_line:
            #     # 有三角形，没有竖线 → A
            #     detected_result = 1  # A
            #     if self.show_debug:
            #         cv2.putText(debug_frame, "检测到标识牌: A (三角形特征)", (10, 30), 
            #                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            #         cv2.putText(debug_frame, f"特征: 三角形=✓, 竖线=✗", 
            #                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            #         cv2.putText(debug_frame, f"HSV阈值: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]", 
            #                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            #         if self.show_ab_only:
            #             cv2.imshow("AB标识-检测结果", debug_frame)
            #         elif not self.show_ab_parking_only and not self.show_red_light_only:
            #             cv2.imshow("2. A/B检测", debug_frame)
            #     self.get_logger().info(
            #         f'🔵 识别到标识牌: A (三角形特征检测)'
            #     )
            #     return detected_result
            # elif has_vertical_line and not has_triangle:
            #     # 有竖线，没有三角形 → B
            #     detected_result = 2  # B
            #     if self.show_debug:
            #         cv2.putText(debug_frame, "检测到标识牌: B (竖线特征)", (10, 30), 
            #                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            #         cv2.putText(debug_frame, f"特征: 三角形=✗, 竖线=✓", 
            #                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            #         cv2.putText(debug_frame, f"HSV阈值: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]", 
            #                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            #         if self.show_ab_only:
            #             cv2.imshow("AB标识-检测结果", debug_frame)
            #         elif not self.show_ab_parking_only and not self.show_red_light_only:
            #             cv2.imshow("2. A/B检测", debug_frame)
            #     self.get_logger().info(
            #         f'🔵 识别到标识牌: B (竖线特征检测)'
            #     )
            #     return detected_result
            # elif has_triangle and has_vertical_line:
            #     # 两者都有，优先判断竖线（B的特征更明显）
            #     detected_result = 2  # B
            #     if self.show_debug:
            #         cv2.putText(debug_frame, "检测到标识牌: B (竖线特征优先)", (10, 30), 
            #                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            #         cv2.putText(debug_frame, f"特征: 三角形=✓, 竖线=✓ (竖线优先)", 
            #                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            #         if self.show_ab_only:
            #             cv2.imshow("AB标识-检测结果", debug_frame)
            #         elif not self.show_ab_parking_only and not self.show_red_light_only:
            #             cv2.imshow("2. A/B检测", debug_frame)
            #     self.get_logger().info(
            #         f'🔵 识别到标识牌: B (竖线特征优先)'
            #     )
            #     return detected_result
            # else:
            #     # 都没有检测到
            #     self.get_logger().warn(
            #         f'⚠️  特征检测失败: 三角形={has_triangle}, 竖线={has_vertical_line}'
            #     )
            
            # 模板匹配失败
            if self.show_debug:
                cv2.putText(debug_frame, "未检测到A/B标识", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(debug_frame, f"模板匹配未成功", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(debug_frame, f"HSV阈值: H[{self.blue_h_min}-{self.blue_h_max}] S[{self.blue_s_min}-{self.blue_s_max}] V[{self.blue_v_min}-{self.blue_v_max}]", 
                           (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                if self.show_ab_only:
                    cv2.imshow("AB标识-检测结果", debug_frame)
                elif not self.show_red_light_only:
                    cv2.imshow("6. A/B检测", debug_frame)
            return 0
                
        except Exception as e:
            if self.show_debug:
                cv2.putText(debug_frame, f"A/B检测异常: {str(e)}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                if self.show_ab_only:
                    cv2.imshow("AB标识-检测结果", debug_frame)
                elif not self.show_red_light_only:
                    cv2.imshow("2. A/B检测", debug_frame)
            self.get_logger().warn(f'A/B检测异常: {e}')
            return 0
    
    def image_callback(self, msg):
        """图像回调函数 - 主处理逻辑"""
        data = [0, 0, 0, 0, 0]  # [红灯, A/B, 黄线, AB车库方向, 蓝色锥桶检测]
        
        try:
            # 将ROS图像转为OpenCV格式 (与您的line_follow2.py一致)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge错误: {e}")
            return
        
        # 1. 红灯检测
        if self.enable_red:
            data[0] = self.redLightDetect(frame)
            if data[0]:
                self.get_logger().info('🔴 检测到红灯!')
        
        # 2. A/B识别 (带置信度累积)
        if self.enable_ab:
            A_B = self.A_B_detect(frame)
            
            if A_B == 0:
                if self.combo:
                    self.combo -= 1
                if self.combo == 0:
                    self.global_confidence = [0, 0]
                    data[1] = 0
                else:
                    data[1] = self.last_ab_result
            else:
                self.combo = 10
                self.global_confidence[A_B - 1] += 1
                data[1] = 1 if self.global_confidence[0] > self.global_confidence[1] else 2
                
                if data[1] != self.last_ab_result and data[1] != 0:
                    sign = 'A' if data[1] == 1 else 'B'
                    self.get_logger().info(f'🔵 识别到标识牌: {sign} (置信度: A={self.global_confidence[0]}, B={self.global_confidence[1]})')
            
            self.last_ab_result = data[1]
        
        # 3. 黄线检测
        if self.enable_yellow:
            data[2] = self.yellowStopDetect(frame)
        
        # 4. AB车库区域检测
        if self.enable_ab_parking:
            detected, line_offset = self.detectAbParkingZone(frame)
            # 将位置偏移转换为整数（-100到100，0=未检测）
            # 如果检测到，返回偏移值*100（范围-100到100）
            # 如果未检测到，返回0
            data[3] = int(line_offset * 100) if detected else 0
        
        # 5. 蓝色锥桶检测（直角弯标志）
        data[4] = self.detect_blue_cones(frame)
        
        # 发布检测结果
        msg_out = Int16MultiArray()
        msg_out.data = data
        self.detection_pub.publish(msg_out)
        
        # 综合检测结果显示（第4个窗口）
        if self.show_debug:
            result_frame = frame.copy()
            
            # 显示检测结果
            y_pos = 30
            # 红灯检测结果
            if self.enable_red:
                red_text = f"红灯: {'检测到' if data[0] else '未检测'}"
                red_color = (0, 0, 255) if data[0] else (0, 255, 0)
                cv2.putText(result_frame, red_text, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, red_color, 2)
                y_pos += 40
            
            # A/B检测结果
            if self.enable_ab:
                if data[1] == 1:
                    ab_text = "A/B标识: A"
                    ab_color = (0, 255, 255)
                elif data[1] == 2:
                    ab_text = "A/B标识: B"
                    ab_color = (255, 0, 255)
                else:
                    ab_text = "A/B标识: 未检测"
                    ab_color = (0, 255, 0)
                cv2.putText(result_frame, ab_text, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, ab_color, 2)
                y_pos += 40
            
            # 黄线检测结果
            if self.enable_yellow:
                yellow_text = f"黄线: {'检测到' if data[2] else '未检测'}"
                yellow_color = (0, 255, 255) if data[2] else (0, 255, 0)
                cv2.putText(result_frame, yellow_text, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, yellow_color, 2)
                y_pos += 40
            
            # 蓝色锥桶检测结果
            blue_text = f"蓝色锥桶: {'检测到' if data[4] else '未检测'}"
            blue_color = (255, 0, 0) if data[4] else (0, 255, 0)
            cv2.putText(result_frame, blue_text, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, blue_color, 2)
            y_pos += 40
            
            # 显示原始数据
            data_text = f"原始数据: [{data[0]}, {data[1]}, {data[2]}, {data[3]}, {data[4]}]"
            cv2.putText(result_frame, data_text, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            if not self.show_ab_parking_only and not self.show_red_light_only:
                cv2.imshow("7. 综合检测结果", result_frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    detector = ImageDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
