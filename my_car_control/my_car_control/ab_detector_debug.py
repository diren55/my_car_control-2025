#!/usr/bin/env python3
"""
AB标志识别调试工具
功能：
1. 实时显示处理过程的每个步骤
2. 保存调试图像到指定目录
3. 显示详细的调试信息
4. 可调节HSV阈值和判断参数

使用方法：
ros2 run my_car_control ab_detector_debug

或指定调试图像保存路径：
ros2 run my_car_control ab_detector_debug --ros-args -p debug_path:=/tmp/ab_debug
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import copy
import os
from datetime import datetime


class ABDetectorDebug(Node):
    def __init__(self):
        super().__init__('ab_detector_debug')
        
        # 声明参数
        self.declare_parameter('debug_path', '/tmp/ab_debug')  # 调试图像保存路径
        self.declare_parameter('save_images', True)  # 是否保存调试图像
        self.declare_parameter('show_windows', True)  # 是否显示实时窗口
        
        # HSV阈值参数（可调）
        self.declare_parameter('blue_h_min', 100)
        self.declare_parameter('blue_h_max', 130)
        self.declare_parameter('blue_s_min', 90)
        self.declare_parameter('blue_s_max', 255)
        self.declare_parameter('blue_v_min', 50)
        self.declare_parameter('blue_v_max', 255)
        
        # 判断参数（与原版一致）
        self.declare_parameter('judge_threshold', 2/3)  # A/B判断阈值（原版固定值）
        
        # 获取参数
        self.debug_path = self.get_parameter('debug_path').value
        self.save_images = self.get_parameter('save_images').value
        self.show_windows = self.get_parameter('show_windows').value
        
        self.blue_h_min = self.get_parameter('blue_h_min').value
        self.blue_h_max = self.get_parameter('blue_h_max').value
        self.blue_s_min = self.get_parameter('blue_s_min').value
        self.blue_s_max = self.get_parameter('blue_s_max').value
        self.blue_v_min = self.get_parameter('blue_v_min').value
        self.blue_v_max = self.get_parameter('blue_v_max').value
        
        self.judge_threshold = self.get_parameter('judge_threshold').value
        
        # 更新HSV阈值
        self.lower_blue = np.array([self.blue_h_min, self.blue_s_min, self.blue_v_min])
        self.upper_blue = np.array([self.blue_h_max, self.blue_s_max, self.blue_v_max])
        
        # 创建调试目录
        if self.save_images:
            os.makedirs(self.debug_path, exist_ok=True)
            self.get_logger().info(f'调试图像保存路径: {self.debug_path}')
        
        # 状态变量
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # QoS配置（兼容摄像头）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅摄像头
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos
        )
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('🔍 AB标志识别调试工具已启动')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'HSV蓝色范围: H[{self.blue_h_min}, {self.blue_h_max}] '
                              f'S[{self.blue_s_min}, {self.blue_s_max}] '
                              f'V[{self.blue_v_min}, {self.blue_v_max}]')
        self.get_logger().info(f'A/B判断阈值: {self.judge_threshold:.3f} (原版固定值 2/3)')
        self.get_logger().info(f'Kernel范围: 5-8 (原版循环算法)')
        self.get_logger().info(f'保存图像: {self.save_images}')
        self.get_logger().info(f'显示窗口: {self.show_windows}')
        self.get_logger().info('=' * 80)
        self.get_logger().info('提示：按 Ctrl+C 退出')
        self.get_logger().info('=' * 80)
    
    def crop_and_HSV(self, res_img):
        """裁剪ROI区域并提取蓝色"""
        # 裁剪图片识别前三分之一的区域
        quarter_row_upper = res_img.shape[0] - (res_img.shape[0] // 3)
        quarter_row_lower = res_img.shape[0]
        quarter_col_left = res_img.shape[1] // 4
        quarter_col_right = res_img.shape[1] - (res_img.shape[1] // 4)
        crop_mask_img = res_img[quarter_row_upper:quarter_row_lower, 
                                quarter_col_left:quarter_col_right]
        
        # 转换到HSV并提取蓝色
        hsv_img = cv2.cvtColor(crop_mask_img, cv2.COLOR_BGR2HSV)
        mask_img = cv2.inRange(hsv_img, self.lower_blue, self.upper_blue)
        
        return mask_img, crop_mask_img
    
    def crop_min_range(self, crop_mask):
        """裁剪最小蓝色区域"""
        if crop_mask is None or crop_mask.size == 0:
            return np.zeros((1, 1), dtype=np.uint8)
        
        contours, _ = cv2.findContours(crop_mask, cv2.RETR_EXTERNAL, 
                                      cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return np.zeros((1, 1), dtype=np.uint8)
        
        # 获取所有轮廓的边界框
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = -float('inf'), -float('inf')
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            min_x, min_y = min(min_x, x), min(min_y, y)
            max_x, max_y = max(max_x, x + w), max(max_y, y + h)
        
        # 安全检查
        if min_x >= max_x or min_y >= max_y:
            return np.zeros((1, 1), dtype=np.uint8)
        
        # 裁剪图像
        crop_img = crop_mask[min_y:max_y, min_x:max_x]
        
        # 形态学处理
        ker = np.ones((6, 6), np.uint8)
        crop_img = cv2.morphologyEx(crop_img, cv2.MORPH_OPEN, ker)
        ker = np.ones((5, 5), np.uint8)
        crop_img = cv2.morphologyEx(crop_img, cv2.MORPH_CLOSE, ker)
        
        return crop_img
    
    def A_B_detect_debug(self, frame):
        """AB检测（原版循环算法，带完整调试信息）"""
        debug_images = {}  # 存储所有中间步骤的图像
        debug_info = {}    # 存储调试信息
        
        try:
            # 步骤1：原始图像
            debug_images['1_original'] = frame.copy()
            
            # 步骤2：裁剪和HSV蓝色提取
            img, roi = self.crop_and_HSV(frame)
            debug_images['2_roi'] = roi
            debug_images['3_blue_mask'] = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            
            blue_pixel_count = np.sum(img > 0)
            debug_info['blue_pixels'] = blue_pixel_count
            
            if blue_pixel_count < 100:
                debug_info['result'] = '蓝色像素太少'
                return 0, debug_images, debug_info
            
            # 步骤3：阈值化
            thresh = cv2.threshold(img, 254, 255, 0)[1]
            debug_images['4_threshold'] = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            
            # ===== 原版算法：循环尝试kernel大小5-8 =====
            debug_info['kernel_attempts'] = []
            
            for i in range(5, 9):
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (i, i))
                threshed_img_smooth = cv2.erode(thresh, kernel, iterations=1)
                threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations=1)
                
                # 保存每个kernel的形态学处理结果
                debug_images[f'5_morphology_k{i}'] = cv2.cvtColor(threshed_img_smooth, 
                                                                    cv2.COLOR_GRAY2BGR)
                
                # 裁剪只含有蓝色区域的部分
                crop_image = self.crop_min_range(threshed_img_smooth)
                
                if crop_image.size == 0 or len(crop_image.shape) < 2:
                    debug_info['kernel_attempts'].append({
                        'kernel': i,
                        'status': '裁剪失败'
                    })
                    continue
                
                debug_images[f'6_crop_k{i}'] = cv2.cvtColor(crop_image, cv2.COLOR_GRAY2BGR)
                
                # 定义图片特定位置的横纵坐标
                x_quarter_left = crop_image.shape[1] // 4
                x_quarter_right = crop_image.shape[1] - crop_image.shape[1] // 4
                x_mid = crop_image.shape[1] // 2
                y_mid = crop_image.shape[0] // 2
                
                # 如果裁剪后的图片只有标识牌，可以判断标识牌上的字母
                if (crop_image[y_mid][x_quarter_left] == 255 and 
                    crop_image[y_mid][x_quarter_right] == 255):
                    crop_image_2 = copy.deepcopy(crop_image)
                    crop_strategy = '完整图像'
                # 如果裁剪后的图片含有锥桶等影响，继续对图像进行裁剪
                else:
                    crop2_image = crop_image[:, :x_mid]
                    crop_image_2 = self.crop_min_range(crop2_image)
                    crop_strategy = '二次裁剪（左半边）'
                
                debug_images[f'7_final_crop_k{i}'] = cv2.cvtColor(crop_image_2, 
                                                                    cv2.COLOR_GRAY2BGR)
                
                # 提取轮廓数量
                crop_image_1 = copy.deepcopy(crop_image_2)
                contours, _ = cv2.findContours(crop_image_1, cv2.RETR_EXTERNAL, 
                                              cv2.CHAIN_APPROX_NONE)
                contours = list(contours)
                
                # 绘制轮廓
                contour_image = cv2.cvtColor(crop_image_1, cv2.COLOR_GRAY2BGR)
                for idx, cnt in enumerate(contours):
                    color = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0)][idx % 4]
                    cv2.drawContours(contour_image, [cnt], -1, color, 2)
                    # 标注轮廓长度
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.putText(contour_image, f'{len(cnt)}', (cx, cy),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                debug_images[f'8_contours_k{i}'] = contour_image
                
                # 原版算法：只识别4个轮廓的情况
                if len(contours) == 4:
                    cnt_len_list = []
                    for cnt in contours:
                        cnt_len_list.append(len(cnt))
                    
                    lens = sorted(cnt_len_list)
                    ratio = lens[0] / lens[1] if lens[1] > 0 else 1.0
                    
                    # 判断大小（原版固定阈值 2/3）
                    judge = 2/3
                    
                    if lens[0] < lens[1] * judge:
                        result = 1  # A
                        debug_info['kernel_attempts'].append({
                            'kernel': i,
                            'contours': len(contours),
                            'lengths': lens,
                            'ratio': ratio,
                            'result': 'A',
                            'crop_strategy': crop_strategy
                        })
                        debug_info['result'] = f'A (kernel={i}, 比例 {ratio:.3f} < {judge:.3f})'
                        debug_info['contour_lengths'] = lens
                        debug_info['ratio'] = ratio
                        debug_info['threshold'] = judge
                        debug_info['success_kernel'] = i
                        
                        # 保存成功的最终结果
                        debug_images['9_final_result'] = contour_image
                        
                        return result, debug_images, debug_info
                    else:
                        result = 2  # B
                        debug_info['kernel_attempts'].append({
                            'kernel': i,
                            'contours': len(contours),
                            'lengths': lens,
                            'ratio': ratio,
                            'result': 'B',
                            'crop_strategy': crop_strategy
                        })
                        debug_info['result'] = f'B (kernel={i}, 比例 {ratio:.3f} >= {judge:.3f})'
                        debug_info['contour_lengths'] = lens
                        debug_info['ratio'] = ratio
                        debug_info['threshold'] = judge
                        debug_info['success_kernel'] = i
                        
                        # 保存成功的最终结果
                        debug_images['9_final_result'] = contour_image
                        
                        return result, debug_images, debug_info
                else:
                    debug_info['kernel_attempts'].append({
                        'kernel': i,
                        'contours': len(contours),
                        'status': f'轮廓数量为{len(contours)}, 不等于4'
                    })
            
            # 所有kernel都失败
            debug_info['result'] = '所有kernel(5-8)都无法识别到4个轮廓'
            return 0, debug_images, debug_info
        
        except Exception as e:
            debug_info['result'] = f'异常: {str(e)}'
            return 0, debug_images, debug_info
    
    def save_debug_images(self, debug_images, debug_info, timestamp):
        """保存所有调试图像"""
        if not self.save_images:
            return
        
        # 创建时间戳目录
        time_dir = os.path.join(self.debug_path, timestamp)
        os.makedirs(time_dir, exist_ok=True)
        
        # 保存所有中间步骤图像
        for name, img in debug_images.items():
            if img is not None and img.size > 0:
                filepath = os.path.join(time_dir, f'{name}.png')
                cv2.imwrite(filepath, img)
        
        # 保存调试信息到文本文件
        info_file = os.path.join(time_dir, 'debug_info.txt')
        with open(info_file, 'w', encoding='utf-8') as f:
            f.write('=' * 60 + '\n')
            f.write(f'AB标志识别调试信息\n')
            f.write(f'时间: {timestamp}\n')
            f.write('=' * 60 + '\n\n')
            
            f.write('HSV参数:\n')
            f.write(f'  蓝色H范围: [{self.blue_h_min}, {self.blue_h_max}]\n')
            f.write(f'  蓝色S范围: [{self.blue_s_min}, {self.blue_s_max}]\n')
            f.write(f'  蓝色V范围: [{self.blue_v_min}, {self.blue_v_max}]\n\n')
            
            f.write('检测参数:\n')
            f.write(f'  A/B判断阈值: {self.judge_threshold:.3f}\n')
            f.write(f'  初始kernel大小: {self.first_kernel_size}\n')
            f.write(f'  动态kernel大小: {debug_info.get("kernel_size", "N/A")}\n\n')
            
            f.write('处理结果:\n')
            for key, value in debug_info.items():
                f.write(f'  {key}: {value}\n')
        
        self.get_logger().info(f'调试图像已保存到: {time_dir}')
    
    def show_debug_windows(self, debug_images, debug_info):
        """显示实时调试窗口（适配循环kernel算法）"""
        if not self.show_windows:
            return
        
        # 1. 显示原始图像
        if '1_original' in debug_images and debug_images['1_original'] is not None:
            img = debug_images['1_original'].copy()
            # 在原始图像上叠加识别结果
            if 'result' in debug_info:
                result_text = f"Result: {debug_info['result']}"
                cv2.putText(img, result_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            self._show_window('1_Original_Image', img)
        
        # 2. 显示蓝色提取
        if '3_blue_mask' in debug_images:
            self._show_window('2_Blue_Mask', debug_images['3_blue_mask'])
        
        # 3. 显示阈值化
        if '4_threshold' in debug_images:
            self._show_window('3_Threshold', debug_images['4_threshold'])
        
        # 4. 显示成功的kernel的形态学结果（如果有）
        if 'success_kernel' in debug_info:
            k = debug_info['success_kernel']
            if f'5_morphology_k{k}' in debug_images:
                img = debug_images[f'5_morphology_k{k}'].copy()
                cv2.putText(img, f'Kernel={k} (Success)', (10, 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self._show_window('4_Morphology', img)
        
        # 5. 显示成功的kernel的裁剪结果
        if 'success_kernel' in debug_info:
            k = debug_info['success_kernel']
            if f'7_final_crop_k{k}' in debug_images:
                img = debug_images[f'7_final_crop_k{k}'].copy()
                cv2.putText(img, f'Kernel={k}', (10, 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self._show_window('5_Final_Crop', img)
        
        # 6. 显示最终轮廓结果（重点！）
        if '9_final_result' in debug_images:
            img = debug_images['9_final_result'].copy()
            
            # 叠加详细信息
            if 'contour_lengths' in debug_info:
                lens = debug_info['contour_lengths']
                ratio = debug_info.get('ratio', 0)
                threshold = debug_info.get('threshold', 0.667)
                result = debug_info.get('result', '')
                
                # 在图像上绘制信息
                y_pos = 20
                cv2.putText(img, f"Lengths: {lens}", (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                y_pos += 25
                cv2.putText(img, f"Ratio: {ratio:.3f} vs {threshold:.3f}", (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                y_pos += 25
                
                # 结果用大字显示
                if 'A' in str(result):
                    color = (0, 255, 0)  # 绿色
                    text = "Result: A"
                elif 'B' in str(result):
                    color = (0, 0, 255)  # 红色
                    text = "Result: B"
                else:
                    color = (128, 128, 128)  # 灰色
                    text = "No Result"
                
                cv2.putText(img, text, (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            self._show_window('6_Contours_Result', img)
        
        # 7. 显示所有kernel的尝试结果（小窗口拼接）
        if 'kernel_attempts' in debug_info and len(debug_info['kernel_attempts']) > 0:
            self._show_kernel_attempts(debug_images, debug_info)
        
        cv2.waitKey(1)
    
    def _show_window(self, window_name, img):
        """显示单个窗口（自动缩放）"""
        if img is None or img.size == 0:
            return
        
        # 自动缩放
        max_width = 800
        max_height = 600
        h, w = img.shape[:2]
        
        if w > max_width or h > max_height:
            scale = min(max_width / w, max_height / h)
            new_w = int(w * scale)
            new_h = int(h * scale)
            img = cv2.resize(img, (new_w, new_h))
        
        cv2.imshow(window_name, img)
    
    def _show_kernel_attempts(self, debug_images, debug_info):
        """显示所有kernel的尝试结果（拼接成一张图）"""
        kernel_images = []
        
        for i in range(5, 9):
            contour_key = f'8_contours_k{i}'
            if contour_key in debug_images:
                img = debug_images[contour_key].copy()
                
                # 查找这个kernel的尝试结果
                attempt_info = None
                for attempt in debug_info.get('kernel_attempts', []):
                    if attempt.get('kernel') == i:
                        attempt_info = attempt
                        break
                
                # 在图像上标注信息
                if attempt_info:
                    status = attempt_info.get('status', '')
                    contours = attempt_info.get('contours', 0)
                    result = attempt_info.get('result', '')
                    
                    # 标题
                    if result:
                        color = (0, 255, 0) if result == 'A' else (0, 0, 255)
                        text = f"K={i}: {result}"
                    else:
                        color = (128, 128, 128)
                        text = f"K={i}: {contours} contours"
                    
                    cv2.putText(img, text, (10, 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # 缩小图像
                h, w = img.shape[:2]
                scale = 200 / max(h, w)
                new_w = int(w * scale)
                new_h = int(h * scale)
                img = cv2.resize(img, (new_w, new_h))
                
                kernel_images.append(img)
        
        # 拼接成一行
        if len(kernel_images) > 0:
            # 确保所有图像高度一致
            max_h = max(img.shape[0] for img in kernel_images)
            padded_images = []
            for img in kernel_images:
                h, w = img.shape[:2]
                if h < max_h:
                    pad = max_h - h
                    img = cv2.copyMakeBorder(img, 0, pad, 0, 0, 
                                            cv2.BORDER_CONSTANT, value=(0, 0, 0))
                padded_images.append(img)
            
            combined = np.hstack(padded_images)
            cv2.imshow('7_All_Kernel_Attempts', combined)
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换图像格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge错误: {e}')
            return
        
        self.frame_count += 1
        
        # 执行AB检测（带调试）
        result, debug_images, debug_info = self.A_B_detect_debug(frame)
        
        # 生成时间戳
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        
        # 打印调试信息
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'帧 #{self.frame_count} - {timestamp}')
        self.get_logger().info('-' * 80)
        
        for key, value in debug_info.items():
            self.get_logger().info(f'  {key}: {value}')
        
        if result == 1:
            self.get_logger().info('🔵 识别结果: A')
        elif result == 2:
            self.get_logger().info('🔵 识别结果: B')
        else:
            self.get_logger().warn('⚠️  未识别到AB标志')
        
        self.get_logger().info('=' * 80)
        
        # 保存调试图像
        if result != 0:  # 只在检测到A或B时保存
            self.save_debug_images(debug_images, debug_info, timestamp)
        
        # 显示实时窗口（传入debug_info）
        self.show_debug_windows(debug_images, debug_info)


def main(args=None):
    rclpy.init(args=args)
    node = ABDetectorDebug()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到退出信号')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

