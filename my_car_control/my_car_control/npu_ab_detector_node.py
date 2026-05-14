#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NPU AB标志检测ROS2节点
使用昇腾NPU实时检测AB标志，并发布到/image_detection话题，与停车系统集成
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os
import time
import copy

# 添加infer_project路径以导入相关模块
sys.path.insert(0, '/home/davinci-mini/infer_project')

# 延迟导入torch（在NPU库导入后，因为NPU库可能依赖torch）
try:
    import torch
except ImportError:
    raise ImportError(
        "torch模块未安装！\n"
        "请在root环境下安装torch:\n"
        "  pip3 install torch\n"
        "或者:\n"
        "  conda install pytorch -c pytorch"
    )

from ais_bench.infer.interface import InferSession
from edge_infer.det_utils import letterbox, scale_coords, nms


def preprocess_img_batch(img_batch):
    """
    预处理图像批次 - 与官方om_infer.py完全一致
    BGR to RGB, HWC to CHW
    """
    img_batch = img_batch[..., ::-1].transpose(0, 3, 1, 2)
    img_batch = img_batch / 255.0
    img_batch = np.ascontiguousarray(img_batch).astype(np.float16)
    return img_batch


class NPUABDetectorNode(Node):
    def __init__(self):
        super().__init__('npu_ab_detector_node')
        
        # 声明参数
        self.declare_parameter('ab_model', '/home/davinci-mini/best.om')
        self.declare_parameter('device_id', 0)
        self.declare_parameter('input_shape', [640, 640])
        self.declare_parameter('conf_thres', 0.4)
        self.declare_parameter('iou_thres', 0.5)
        self.declare_parameter('ab_classes', 'A,B')
        self.declare_parameter('enable_redlight', False)  # 是否同时检测红绿灯
        self.declare_parameter('redlight_model', '/home/davinci-mini/redlight.om')
        self.declare_parameter('redlight_classes', 'red,green')
        self.declare_parameter('enable_yellow_detect', True)  # 是否检测黄线
        self.declare_parameter('enable_ab_parking_detect', True)  # 是否检测AB车库方向
        self.declare_parameter('yellow_stop_height', 0.65)  # 黄线停止高度比例
        self.declare_parameter('ab_parking_min_pixels', 6000)  # AB车库最小像素数
        
        # 获取参数
        ab_model_path = self.get_parameter('ab_model').value
        device_id = self.get_parameter('device_id').value
        input_shape = self.get_parameter('input_shape').value
        conf_thres = self.get_parameter('conf_thres').value
        iou_thres = self.get_parameter('iou_thres').value
        ab_classes_str = self.get_parameter('ab_classes').value
        enable_redlight = self.get_parameter('enable_redlight').value
        redlight_model_path = self.get_parameter('redlight_model').value
        redlight_classes_str = self.get_parameter('redlight_classes').value
        
        # 解析类别名称
        self.ab_class_names = {i: name.strip() for i, name in enumerate(ab_classes_str.split(','))}
        self.redlight_class_names = {i: name.strip() for i, name in enumerate(redlight_classes_str.split(','))}
        
        # 获取黄线和AB车库检测参数
        self.enable_yellow = self.get_parameter('enable_yellow_detect').value
        self.enable_ab_parking = self.get_parameter('enable_ab_parking_detect').value
        self.yellow_stop_height = self.get_parameter('yellow_stop_height').value
        self.ab_parking_min_pixels = self.get_parameter('ab_parking_min_pixels').value
        
        # HSV颜色范围（黄线检测用）
        self.minYellow = np.array([20, 100, 100])
        self.maxYellow = np.array([30, 255, 255])
        
        # 配置参数
        self.cfg = {
            'conf_thres': conf_thres,
            'iou_thres': iou_thres,
            'input_shape': input_shape,
        }
        
        # 设置昇腾NPU库路径
        ascend_lib_paths = [
            '/usr/local/Ascend/ascend-toolkit/7.0.RC1/aarch64-linux/devlib',
            '/usr/local/Ascend/ascend-toolkit/7.0.RC1/atc/lib64',
            '/usr/local/Ascend/driver/lib64',
            '/usr/local/Ascend/add-ons',
        ]
        current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
        new_paths = [p for p in ascend_lib_paths if os.path.exists(p)]
        if new_paths:
            new_ld_path = ':'.join(new_paths)
            if current_ld_path:
                os.environ['LD_LIBRARY_PATH'] = new_ld_path + ':' + current_ld_path
            else:
                os.environ['LD_LIBRARY_PATH'] = new_ld_path
        
        # 加载AB标志检测模型
        self.get_logger().info(f"加载AB标志检测模型: {ab_model_path}")
        try:
            self.ab_model = InferSession(device_id, ab_model_path)
            self.get_logger().info("✅ AB标志检测模型加载成功")
        except Exception as e:
            self.get_logger().error(f"❌ AB标志检测模型加载失败: {e}")
            raise
        
        # 加载红绿灯检测模型（如果启用）
        self.enable_redlight = enable_redlight
        self.redlight_model = None
        if enable_redlight:
            self.get_logger().info(f"加载红绿灯检测模型: {redlight_model_path}")
            try:
                self.redlight_model = InferSession(device_id, redlight_model_path)
                self.get_logger().info("✅ 红绿灯检测模型加载成功")
            except Exception as e:
                self.get_logger().warn(f"⚠️  红绿灯检测模型加载失败: {e}")
                self.enable_redlight = False
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 声明摄像头话题参数（默认与image_detector.py保持一致）
        self.declare_parameter('camera_topic', '/image_raw')
        self.camera_topic = self.get_parameter('camera_topic').value
        
        # 订阅摄像头话题
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # 发布检测结果到/image_detection话题
        # 数据格式: [红灯, A/B, 黄线, AB车库方向, 蓝色锥桶]
        # 与image_detector.py保持一致
        self.detection_pub = self.create_publisher(
            Int16MultiArray,
            '/image_detection',
            10
        )
        
        # AB检测状态（带置信度累积，与image_detector.py逻辑一致）
        self.global_confidence = [0, 0]  # [A的置信度, B的置信度]
        self.combo = 0  # 连续未检测计数
        self.last_ab_result = 0  # 上次的AB结果：0=未检测，1=A，2=B
        
        # 黄线检测状态
        self.imaged_yellow = False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('NPU AB标志检测节点已启动')
        self.get_logger().info(f'订阅话题: {self.camera_topic}')
        self.get_logger().info(f'发布话题: /image_detection')
        self.get_logger().info(f'AB类别: {self.ab_class_names}')
        if self.enable_redlight:
            self.get_logger().info(f'红绿灯类别: {self.redlight_class_names}')
        self.get_logger().info(f'黄线检测: {"启用" if self.enable_yellow else "禁用"}')
        self.get_logger().info(f'AB车库检测: {"启用" if self.enable_ab_parking else "禁用"}')
        if self.enable_ab_parking:
            self.get_logger().info(f'  AB车库最小像素数: {self.ab_parking_min_pixels}')
        self.get_logger().info('=' * 60)
    
    def detect_ab(self, frame):
        """
        检测AB标志
        返回: 0=未检测，1=A，2=B
        """
        # 使用letterbox进行预处理
        img_padded, scale_ratio, pad_size = letterbox(frame, new_shape=self.cfg['input_shape'])
        padding_args = (scale_ratio, pad_size)
        
        # 添加batch维度并预处理
        img_batch = np.expand_dims(img_padded, axis=0)  # [1, H, W, 3]
        img_batch = preprocess_img_batch(img_batch)  # [1, 3, H, W]
        
        # AB标志检测
        ab_output = self.ab_model.infer([img_batch])
        ab_output = torch.tensor(ab_output[0])
        ab_boxout = nms(ab_output, conf_thres=self.cfg["conf_thres"], iou_thres=self.cfg["iou_thres"])
        
        # 处理检测结果
        ab_result = 0  # 0=未检测
        if len(ab_boxout) > 0 and ab_boxout[0].shape[0] > 0:
            ab_pred_all = ab_boxout[0].numpy()
            # 坐标缩放
            scale_coords(self.cfg['input_shape'], ab_pred_all[:, :4], frame.shape,
                         ratio_pad=padding_args)
            
            # 找到置信度最高的检测结果
            max_conf_idx = np.argmax(ab_pred_all[:, 4])
            class_id = int(ab_pred_all[max_conf_idx, 5])
            conf = float(ab_pred_all[max_conf_idx, 4])
            
            # 类别ID: 0=A, 1=B -> 转换为: 1=A, 2=B
            if class_id == 0:  # A
                ab_result = 1
            elif class_id == 1:  # B
                ab_result = 2
        
        return ab_result
    
    def detect_redlight(self, frame):
        """
        检测红绿灯
        返回: 0=未检测，1=检测到红灯
        """
        if not self.enable_redlight or self.redlight_model is None:
            return 0
        
        # 使用letterbox进行预处理
        img_padded, scale_ratio, pad_size = letterbox(frame, new_shape=self.cfg['input_shape'])
        padding_args = (scale_ratio, pad_size)
        
        # 添加batch维度并预处理
        img_batch = np.expand_dims(img_padded, axis=0)
        img_batch = preprocess_img_batch(img_batch)
        
        # 红绿灯检测
        redlight_output = self.redlight_model.infer([img_batch])
        redlight_output = torch.tensor(redlight_output[0])
        redlight_boxout = nms(redlight_output, conf_thres=self.cfg["conf_thres"], iou_thres=self.cfg["iou_thres"])
        
        # 处理检测结果
        redlight_result = 0  # 0=未检测到红灯
        if len(redlight_boxout) > 0 and redlight_boxout[0].shape[0] > 0:
            redlight_pred_all = redlight_boxout[0].numpy()
            # 坐标缩放
            scale_coords(self.cfg['input_shape'], redlight_pred_all[:, :4], frame.shape,
                         ratio_pad=padding_args)
            
            # 检查是否有红灯（类别0=red）
            for pred in redlight_pred_all:
                class_id = int(pred[5])
                conf = float(pred[4])
                if class_id == 0 and conf >= self.cfg['conf_thres']:  # red
                    redlight_result = 1
                    break
        
        return redlight_result
    
    def detect_yellow_line(self, frame):
        """
        黄线停止检测 - 与image_detector.py逻辑一致
        返回: 0=未检测，1=检测到
        """
        if not self.enable_yellow:
            return 0
        
        try:
            # HSV转换和黄线提取
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(img_hsv, self.minYellow, self.maxYellow)
            
            # 形态学处理
            thresh = cv2.threshold(mask_yellow, 254, 255, 0)[1]
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
            threshed_img_smooth = cv2.erode(thresh, kernel, iterations=1)
            threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations=2)
            
            # 检测黄线
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
            
            return 1 if detected else 0
        except Exception as e:
            self.get_logger().warn(f'黄线检测异常: {e}')
            return 0
    
    def detect_ab_parking_zone(self, frame):
        """
        AB车库区域检测 - 与image_detector.py逻辑一致
        返回: (detected, line_offset)
        detected: True/False
        line_offset: -1.0到1.0（负数=左侧，正数=右侧，0=中心）
        """
        if not self.enable_ab_parking:
            return False, 0.0
        
        try:
            # HSV转换和黄线提取
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(img_hsv, self.minYellow, self.maxYellow)
            
            # 基础去噪
            _, thresh = cv2.threshold(mask_yellow, 254, 255, cv2.THRESH_BINARY)
            
            # 过滤横线（使用竖向核进行开运算）
            vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 50))
            vertical_only = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, vertical_kernel, iterations=1)
            
            # 形态学处理（去噪，让竖线区域更连续）
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 5))
            processed_img = cv2.morphologyEx(vertical_only, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            # 统计整个图像中连通的白色像素总数（只统计竖线部分）
            total_white_pixels = np.sum(processed_img == 255)
            
            height, width = processed_img.shape
            img_center_x = width // 2
            
            # 判断是否进入停车区（基于总像素数）
            detected = False
            line_offset = 0.0
            
            MIN_WHITE_PIXELS = self.ab_parking_min_pixels
            
            # 找到所有白色像素的坐标（用于计算重心）
            white_pixel_coords = np.column_stack(np.where(processed_img == 255))
            
            if total_white_pixels >= MIN_WHITE_PIXELS:
                # 进入停车区
                detected = True
                
                if len(white_pixel_coords) > 0:
                    # 计算重心的X坐标
                    center_x = int(np.mean(white_pixel_coords[:, 1]))
                    # 计算偏移量 (-1.0 左, 1.0 右)
                    line_offset = (center_x - img_center_x) / (width / 2.0)
                    line_offset = max(-1.0, min(1.0, line_offset))  # 限幅
                else:
                    line_offset = 0.0
                
                self.get_logger().info(
                    f'🅿️  进入AB停车区！白色像素: {total_white_pixels} >= {MIN_WHITE_PIXELS}, 偏移: {line_offset:.2f}'
                )
            
            return detected, line_offset
        except Exception as e:
            self.get_logger().warn(f'AB车库检测异常: {e}')
            return False, 0.0
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将ROS图像转为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge错误: {e}")
            return
        
        # 初始化检测结果
        # 数据格式: [红灯, A/B, 黄线, AB车库方向, 蓝色锥桶]
        data = [0, 0, 0, 0, 0]
        
        # 1. 红绿灯检测（如果启用）
        if self.enable_redlight:
            data[0] = self.detect_redlight(frame)
        
        # 2. AB标志检测（带置信度累积，与image_detector.py逻辑一致）
        ab_detected = self.detect_ab(frame)
        
        if ab_detected == 0:
            # 未检测到，减少置信度
            if self.combo > 0:
                self.combo -= 1
            if self.combo == 0:
                self.global_confidence = [0, 0]
                data[1] = 0
            else:
                data[1] = self.last_ab_result
        else:
            # 检测到AB标志，增加对应置信度
            self.combo = 10  # 重置连续未检测计数
            self.global_confidence[ab_detected - 1] += 1
            # 选择置信度更高的
            data[1] = 1 if self.global_confidence[0] > self.global_confidence[1] else 2
            
            # 如果结果发生变化，打印日志
            if data[1] != self.last_ab_result and data[1] != 0:
                sign = 'A' if data[1] == 1 else 'B'
                self.get_logger().info(
                    f'🔵 NPU识别到标识牌: {sign} '
                    f'(置信度: A={self.global_confidence[0]}, B={self.global_confidence[1]})'
                )
        
        self.last_ab_result = data[1]
        
        # 3. 黄线检测
        if self.enable_yellow:
            data[2] = self.detect_yellow_line(frame)
        
        # 4. AB车库方向检测
        if self.enable_ab_parking:
            ab_parking_detected, line_offset = self.detect_ab_parking_zone(frame)
            # 将位置偏移转换为整数（-100到100，0=未检测）
            # 如果检测到，返回偏移值*100（范围-100到100）
            # 如果未检测到，返回0
            data[3] = int(line_offset * 100) if ab_parking_detected else 0
        
        # 5. 蓝色锥桶 - 保持为0（如果需要可以后续添加）
        data[4] = 0
        
        # 发布检测结果
        msg_out = Int16MultiArray()
        msg_out.data = data
        self.detection_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = NPUABDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

