#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO A/B标志识别节点
使用训练好的best.onnx模型进行A/B标志识别
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import onnxruntime as ort
import os


class YOLOABDetector(Node):
    """使用YOLO模型进行A/B标志识别"""
    
    def __init__(self):
        super().__init__('yolo_ab_detector')
        
        # 声明参数
        self.declare_parameter('model_path', '/home/davinci-mini/best.onnx')
        self.declare_parameter('conf_threshold', 0.25)  # 置信度阈值
        self.declare_parameter('iou_threshold', 0.45)   # NMS IoU阈值
        self.declare_parameter('input_size', 640)       # 模型输入尺寸
        self.declare_parameter('show_result', True)      # 是否显示结果窗口
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.input_size = self.get_parameter('input_size').value
        self.show_result = self.get_parameter('show_result').value
        
        # 检查模型文件是否存在
        if not os.path.exists(model_path):
            self.get_logger().error(f'模型文件不存在: {model_path}')
            raise FileNotFoundError(f'模型文件不存在: {model_path}')
        
        # 加载ONNX模型（优先使用GPU，如果不可用则使用CPU）
        try:
            # 检查可用的providers
            available_providers = ort.get_available_providers()
            self.get_logger().info(f'可用的推理后端: {available_providers}')
            
            # 优先使用GPU（CUDA或TensorRT），如果不可用则使用CPU
            if 'CUDAExecutionProvider' in available_providers:
                providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
                self.get_logger().info('✅ 使用GPU (CUDA) 进行推理')
            elif 'TensorrtExecutionProvider' in available_providers:
                providers = ['TensorrtExecutionProvider', 'CPUExecutionProvider']
                self.get_logger().info('✅ 使用GPU (TensorRT) 进行推理')
            else:
                providers = ['CPUExecutionProvider']
                self.get_logger().info('⚠️  未检测到GPU，使用CPU进行推理')
            
            self.session = ort.InferenceSession(
                model_path,
                providers=providers
            )
            self.get_logger().info(f'✅ 成功加载模型: {model_path}')
        except Exception as e:
            self.get_logger().error(f'加载模型失败: {e}')
            raise
        
        # 获取模型输入输出信息
        self.input_name = self.session.get_inputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        self.get_logger().info(f'模型输入: {self.input_name}, 形状: {input_shape}')
        
        # 类别名称（A和B）
        self.class_names = ['A', 'B']
        
        # 状态变量
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅摄像头话题
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos
        )
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('🚀 YOLO A/B标志识别节点已启动')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'模型路径: {model_path}')
        self.get_logger().info(f'置信度阈值: {self.conf_threshold}')
        self.get_logger().info(f'IoU阈值: {self.iou_threshold}')
        self.get_logger().info(f'输入尺寸: {self.input_size}x{self.input_size}')
        self.get_logger().info(f'显示结果: {self.show_result}')
        self.get_logger().info('=' * 80)
    
    def preprocess_image(self, image):
        """
        预处理图像：resize、归一化、转换为模型输入格式
        """
        # 获取原始图像尺寸
        h, w = image.shape[:2]
        
        # 计算缩放比例，保持宽高比
        scale = min(self.input_size / h, self.input_size / w)
        new_h, new_w = int(h * scale), int(w * scale)
        
        # Resize图像
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        # 创建填充后的图像（填充到input_size x input_size）
        padded = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        padded[:new_h, :new_w] = resized
        
        # 转换为RGB（YOLO通常使用RGB）
        padded_rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        
        # 归一化到[0, 1]并转换为float32
        normalized = padded_rgb.astype(np.float32) / 255.0
        
        # 转换为NCHW格式 (1, 3, H, W)
        transposed = np.transpose(normalized, (2, 0, 1))
        batched = np.expand_dims(transposed, axis=0)
        
        return batched, scale, (new_w, new_h), (w, h)
    
    def postprocess_output(self, outputs, scale, padded_size, original_size):
        """
        后处理模型输出：NMS、坐标转换
        处理YOLO多尺度输出格式: (1, 3, H, W, 8)
        """
        # YOLO输出格式: (1, 3, H, W, 8)
        # 8 = 4(bbox: x, y, w, h) + 1(objectness) + 2(class_scores) + 1(可能是其他)
        # 有三个不同尺度的输出
        
        orig_w, orig_h = original_size
        pad_w, pad_h = padded_size
        
        # 计算实际图像在填充图像中的位置
        scale_factor = min(self.input_size / orig_h, self.input_size / orig_w)
        new_w = int(orig_w * scale_factor)
        new_h = int(orig_h * scale_factor)
        
        # 计算填充偏移
        pad_x = (self.input_size - new_w) / 2.0
        pad_y = (self.input_size - new_h) / 2.0
        
        # 不同尺度的stride（相对于640x640输入）
        strides = [8, 16, 32]  # 对应80x80, 40x40, 20x20
        
        all_detections = []
        
        # 处理每个尺度的输出
        for scale_idx, output in enumerate(outputs):
            # output shape: (1, 3, H, W, 8)
            output = output[0]  # 移除batch维度: (3, H, W, 8)
            stride = strides[scale_idx]
            h, w = output.shape[1:3]
            
            # 创建网格坐标
            yv, xv = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
            
            # 遍历所有anchor和位置
            for anchor_idx in range(3):  # 3个anchor
                # 获取该anchor的所有预测
                anchor_preds = output[anchor_idx]  # (H, W, 8)
                
                # 解析预测: [x, y, w, h, obj, cls_A, cls_B, ?]
                x_offset = anchor_preds[:, :, 0]
                y_offset = anchor_preds[:, :, 1]
                width = anchor_preds[:, :, 2]
                height = anchor_preds[:, :, 3]
                objectness = anchor_preds[:, :, 4]
                class_scores = anchor_preds[:, :, 5:7]  # (H, W, 2)
                
                # 应用sigmoid（YOLO输出通常需要sigmoid）
                objectness = 1.0 / (1.0 + np.exp(-np.clip(objectness, -500, 500)))
                
                # 计算类别分数（sigmoid + softmax）
                class_scores_sigmoid = 1.0 / (1.0 + np.exp(-np.clip(class_scores, -500, 500)))
                # 使用softmax归一化
                class_probs = np.exp(class_scores_sigmoid - np.max(class_scores_sigmoid, axis=2, keepdims=True))
                class_probs = class_probs / (np.sum(class_probs, axis=2, keepdims=True) + 1e-8)
                
                # 获取每个位置的最佳类别
                class_ids = np.argmax(class_probs, axis=2)
                class_max_scores = np.max(class_probs, axis=2)
                
                # 计算置信度
                confidence = objectness * class_max_scores
                
                # 过滤低置信度检测
                valid_mask = confidence >= self.conf_threshold
                
                # 调试：打印置信度统计
                if self.frame_count == 1 and scale_idx == 0 and anchor_idx == 0:
                    self.get_logger().info(
                        f'置信度统计: min={confidence.min():.4f}, max={confidence.max():.4f}, '
                        f'mean={confidence.mean():.4f}, threshold={self.conf_threshold}, '
                        f'有效检测数={np.sum(valid_mask)}'
                    )
                
                if not np.any(valid_mask):
                    continue
                
                # 获取有效检测的位置
                valid_y, valid_x = np.where(valid_mask)
                
                for i in range(len(valid_y)):
                    y, x = valid_y[i], valid_x[i]
                    
                    # 获取预测值
                    x_off = x_offset[y, x]
                    y_off = y_offset[y, x]
                    w = width[y, x]
                    h = height[y, x]
                    obj = objectness[y, x]
                    cls_id = class_ids[y, x]
                    cls_score = class_max_scores[y, x]
                    conf = confidence[y, x]
                    
                    # 解码坐标（YOLO标准方式：sigmoid + 网格偏移）
                    # x, y 偏移需要sigmoid，范围在[0,1]
                    x_off = 1.0 / (1.0 + np.exp(-np.clip(x_off, -500, 500)))
                    y_off = 1.0 / (1.0 + np.exp(-np.clip(y_off, -500, 500)))
                    
                    # 计算绝对坐标（在640x640输入图像上）
                    x_center = (x + x_off) * stride
                    y_center = (y + y_off) * stride
                    
                    # 处理宽高（YOLO通常使用exp，但这里可能需要乘以anchor尺寸）
                    # 先尝试直接使用，如果值太小则用exp
                    if abs(w) < 0.1 or abs(h) < 0.1:
                        w = np.exp(np.clip(w, -10, 10))
                        h = np.exp(np.clip(h, -10, 10))
                    
                    # 使用stride作为基础尺寸
                    width_abs = abs(w) * stride
                    height_abs = abs(h) * stride
                    
                    # 转换到原始图像坐标
                    x_center = (x_center - pad_x) / scale_factor
                    y_center = (y_center - pad_y) / scale_factor
                    width_abs = width_abs / scale_factor
                    height_abs = height_abs / scale_factor
                    
                    # 转换为左上角和右下角坐标
                    x1 = int(x_center - width_abs / 2)
                    y1 = int(y_center - height_abs / 2)
                    x2 = int(x_center + width_abs / 2)
                    y2 = int(y_center + height_abs / 2)
                    
                    # 限制在图像范围内
                    x1 = max(0, min(x1, orig_w))
                    y1 = max(0, min(y1, orig_h))
                    x2 = max(0, min(x2, orig_w))
                    y2 = max(0, min(y2, orig_h))
                    
                    # 检查边界框是否有效
                    if x2 > x1 and y2 > y1:
                        all_detections.append({
                            'bbox': [x1, y1, x2, y2],
                            'confidence': float(conf),
                            'class_id': int(cls_id),
                            'class_name': self.class_names[cls_id]
                        })
        
        # NMS (非极大值抑制)
        if len(all_detections) > 0:
            all_detections = self.nms(all_detections)
        
        return all_detections
    
    def nms(self, detections):
        """
        非极大值抑制
        """
        if len(detections) == 0:
            return []
        
        # 按置信度排序
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
        
        keep = []
        while len(detections) > 0:
            # 保留置信度最高的
            keep.append(detections[0])
            
            if len(detections) == 1:
                break
            
            # 计算IoU并移除重叠的检测
            bbox1 = detections[0]['bbox']
            remaining = []
            
            for det in detections[1:]:
                bbox2 = det['bbox']
                iou = self.calculate_iou(bbox1, bbox2)
                if iou < self.iou_threshold:
                    remaining.append(det)
            
            detections = remaining
        
        return keep
    
    def calculate_iou(self, bbox1, bbox2):
        """
        计算两个边界框的IoU
        """
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # 计算交集
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        inter_area = (x2_i - x1_i) * (y2_i - y1_i)
        
        # 计算并集
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = area1 + area2 - inter_area
        
        if union_area == 0:
            return 0.0
        
        return inter_area / union_area
    
    def draw_detections(self, image, detections):
        """
        在图像上绘制检测结果
        """
        result_image = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            confidence = det['confidence']
            class_name = det['class_name']
            class_id = det['class_id']
            
            # 选择颜色：A用绿色，B用红色
            color = (0, 255, 0) if class_id == 0 else (0, 0, 255)
            
            # 绘制边界框
            cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制标签
            label = f'{class_name} {confidence:.2f}'
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            label_y = max(y1, label_size[1] + 10)
            
            # 绘制标签背景
            cv2.rectangle(
                result_image,
                (x1, label_y - label_size[1] - 5),
                (x1 + label_size[0], label_y + 5),
                color,
                -1
            )
            
            # 绘制标签文字
            cv2.putText(
                result_image,
                label,
                (x1, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )
        
        return result_image
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换ROS图像消息为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge错误: {e}')
            return
        
        self.frame_count += 1
        
        # 预处理图像
        input_tensor, scale, padded_size, original_size = self.preprocess_image(frame)
        
        # 运行推理
        try:
            outputs = self.session.run(None, {self.input_name: input_tensor})
        except Exception as e:
            self.get_logger().error(f'推理错误: {e}')
            return
        
        # 后处理输出
        detections = self.postprocess_output(outputs, scale, padded_size, original_size)
        
        # 调试：打印检测统计信息
        if self.frame_count % 30 == 0:  # 每30帧打印一次
            total_candidates = sum(out[0].size // 8 for out in outputs)
            self.get_logger().debug(f'处理了 {total_candidates} 个候选检测，最终保留 {len(detections)} 个')
            # 打印输出统计
            for i, out in enumerate(outputs):
                obj_vals = out[0, :, :, :, 4]
                cls_vals = out[0, :, :, :, 5:7]
                self.get_logger().debug(
                    f'输出{i}: objectness范围[{obj_vals.min():.3f}, {obj_vals.max():.3f}], '
                    f'均值{obj_vals.mean():.3f}, '
                    f'class_scores范围[{cls_vals.min():.3f}, {cls_vals.max():.3f}]'
                )
        
        # 打印检测结果
        if len(detections) > 0:
            self.get_logger().info('=' * 80)
            self.get_logger().info(f'帧 #{self.frame_count} - 检测到 {len(detections)} 个目标:')
            for i, det in enumerate(detections):
                self.get_logger().info(
                    f'  [{i+1}] {det["class_name"]}: '
                    f'置信度={det["confidence"]:.3f}, '
                    f'位置=({det["bbox"][0]}, {det["bbox"][1]}, {det["bbox"][2]}, {det["bbox"][3]})'
                )
            self.get_logger().info('=' * 80)
        else:
            if self.frame_count % 30 == 0:  # 每30帧打印一次，避免刷屏
                self.get_logger().info(f'帧 #{self.frame_count} - 未检测到目标')
        
        # 绘制检测结果
        result_image = self.draw_detections(frame, detections)
        
        # 显示结果
        if self.show_result:
            # 如果图像太大，缩放显示
            h, w = result_image.shape[:2]
            max_display_size = 1280
            if w > max_display_size or h > max_display_size:
                scale_display = min(max_display_size / w, max_display_size / h)
                new_w = int(w * scale_display)
                new_h = int(h * scale_display)
                display_image = cv2.resize(result_image, (new_w, new_h))
            else:
                display_image = result_image
            
            cv2.imshow('YOLO A/B Detection', display_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOABDetector()
    
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

