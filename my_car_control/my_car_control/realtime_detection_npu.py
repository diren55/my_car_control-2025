#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时摄像头检测脚本 - 使用昇腾NPU
功能: 实时调用摄像头，使用best.om检测AB标志，使用redlight.om检测红绿灯
"""

import os
import sys
import cv2
import numpy as np
import time
import argparse
import torch

# 添加infer_project路径以导入相关模块
sys.path.insert(0, '/home/davinci-mini/infer_project')
from ais_bench.infer.interface import InferSession
from edge_infer.det_utils import letterbox, scale_coords, nms


def preprocess_img(img, input_shape=(640, 640)):
    """
    预处理单张图像
    Args:
        img: BGR格式的OpenCV图像
        input_shape: 模型输入尺寸 (height, width)
    Returns:
        img_padded: 预处理后的图像
        padding_args: (scale_ratio, pad_size) 用于后处理坐标缩放
    """
    img_padded, scale_ratio, pad_size = letterbox(img, new_shape=input_shape)
    # BGR to RGB, HWC to CHW
    img_batch = img_padded[..., ::-1].transpose(2, 0, 1)
    # 归一化到0-1
    img_batch = img_batch / 255.0
    # 转为float16并添加batch维度
    img_batch = np.ascontiguousarray(img_batch).astype(np.float16)
    img_batch = np.expand_dims(img_batch, axis=0)  # [1, 3, H, W]
    return img_batch, (scale_ratio, pad_size)


def draw_detections(img, detections, class_names, colors=None, conf_threshold=0.25):
    """
    在图像上绘制检测结果
    Args:
        img: 原始图像
        detections: 检测结果，numpy数组，shape为[N, 6]，每行为[x1, y1, x2, y2, conf, cls]
        class_names: 类别名称字典
        colors: 每个类别的颜色，如果为None则自动生成
        conf_threshold: 置信度阈值
    Returns:
        img: 绘制了检测框的图像
    """
    if colors is None:
        # 为不同类别生成不同颜色
        colors = {}
        for cls_id in range(len(class_names)):
            colors[cls_id] = tuple(np.random.randint(0, 255, 3).tolist())
    
    for det in detections:
        x1, y1, x2, y2, conf, cls_id = det[:6]
        if conf < conf_threshold:
            continue
        
        cls_id = int(cls_id)
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        
        # 获取类别名称
        if cls_id in class_names:
            cls_name = class_names[cls_id]
        else:
            cls_name = f"class_{cls_id}"
        
        # 绘制边界框
        color = colors.get(cls_id, (0, 255, 0))
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        
        # 绘制标签和置信度
        label = f"{cls_name} {conf:.2f}"
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_y = max(y1, label_size[1] + 10)
        cv2.rectangle(img, (x1, label_y - label_size[1] - 10), 
                     (x1 + label_size[0], label_y), color, -1)
        cv2.putText(img, label, (x1, label_y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return img


def main():
    parser = argparse.ArgumentParser(description='Real-time detection using Ascend NPU')
    parser.add_argument('--ab-model', type=str, default='/home/davinci-mini/best.om',
                       help='AB标志检测模型路径')
    parser.add_argument('--redlight-model', type=str, default='/home/davinci-mini/redlight.om',
                       help='红绿灯检测模型路径')
    parser.add_argument('--device-id', type=int, default=0, help='昇腾NPU设备ID')
    parser.add_argument('--camera-id', type=int, default=0, help='摄像头设备ID')
    parser.add_argument('--input-shape', type=int, nargs=2, default=[640, 640],
                       help='模型输入尺寸 [height, width]')
    parser.add_argument('--conf-threshold', type=float, default=0.25, help='置信度阈值')
    parser.add_argument('--iou-threshold', type=float, default=0.5, help='IoU阈值')
    parser.add_argument('--show-fps', action='store_true', help='显示FPS')
    parser.add_argument('--ab-classes', type=str, default='A,B',
                       help='AB标志类别名称，用逗号分隔')
    parser.add_argument('--redlight-classes', type=str, default='red,green,yellow',
                       help='红绿灯类别名称，用逗号分隔')
    
    args = parser.parse_args()
    
    # 检查模型文件是否存在
    if not os.path.exists(args.ab_model):
        print(f"错误: AB模型文件不存在: {args.ab_model}")
        return
    if not os.path.exists(args.redlight_model):
        print(f"错误: 红绿灯模型文件不存在: {args.redlight_model}")
        return
    
    # 解析类别名称
    ab_class_names = {i: name.strip() for i, name in enumerate(args.ab_classes.split(','))}
    redlight_class_names = {i: name.strip() for i, name in enumerate(args.redlight_classes.split(','))}
    
    print("=" * 60)
    print("初始化昇腾NPU模型")
    print("=" * 60)
    
    # 加载AB标志检测模型
    print(f"加载AB标志检测模型: {args.ab_model}")
    try:
        ab_model = InferSession(args.device_id, args.ab_model)
        print("✅ AB标志检测模型加载成功")
    except Exception as e:
        print(f"❌ AB标志检测模型加载失败: {e}")
        return
    
    # 加载红绿灯检测模型
    print(f"加载红绿灯检测模型: {args.redlight_model}")
    try:
        redlight_model = InferSession(args.device_id, args.redlight_model)
        print("✅ 红绿灯检测模型加载成功")
    except Exception as e:
        print(f"❌ 红绿灯检测模型加载失败: {e}")
        return
    
    # 打开摄像头
    print("\n" + "=" * 60)
    print("打开摄像头")
    print("=" * 60)
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"❌ 无法打开摄像头: {args.camera_id}")
        return
    
    # 获取摄像头分辨率
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"摄像头分辨率: {width}x{height}, FPS: {fps}")
    
    print("\n" + "=" * 60)
    print("开始实时检测 (按 'q' 退出)")
    print("=" * 60)
    
    frame_count = 0
    fps_start_time = time.time()
    fps_counter = 0
    
    input_shape = tuple(args.input_shape)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 无法读取摄像头画面")
                break
            
            frame_count += 1
            fps_counter += 1
            
            # 预处理图像
            img_batch, padding_args = preprocess_img(frame, input_shape)
            
            # AB标志检测
            ab_start_time = time.time()
            ab_output = ab_model.infer([img_batch])
            ab_output = torch.tensor(ab_output[0])
            ab_detections = nms(ab_output, conf_thres=args.conf_threshold, iou_thres=args.iou_threshold)
            ab_time = (time.time() - ab_start_time) * 1000
            
            # 红绿灯检测
            redlight_start_time = time.time()
            redlight_output = redlight_model.infer([img_batch])
            redlight_output = torch.tensor(redlight_output[0])
            redlight_detections = nms(redlight_output, conf_thres=args.conf_threshold, iou_thres=args.iou_threshold)
            redlight_time = (time.time() - redlight_start_time) * 1000
            
            # 处理检测结果
            result_img = frame.copy()
            
            # 绘制AB标志检测结果
            if len(ab_detections) > 0 and ab_detections[0].shape[0] > 0:
                ab_pred = ab_detections[0].numpy()
                scale_coords(input_shape, ab_pred[:, :4], frame.shape, ratio_pad=padding_args)
                result_img = draw_detections(result_img, ab_pred, ab_class_names, 
                                           colors={0: (0, 255, 0), 1: (255, 0, 0)}, 
                                           conf_threshold=args.conf_threshold)
            
            # 绘制红绿灯检测结果
            if len(redlight_detections) > 0 and redlight_detections[0].shape[0] > 0:
                redlight_pred = redlight_detections[0].numpy()
                scale_coords(input_shape, redlight_pred[:, :4], frame.shape, ratio_pad=padding_args)
                result_img = draw_detections(result_img, redlight_pred, redlight_class_names,
                                           colors={0: (0, 0, 255), 1: (0, 255, 0), 2: (0, 255, 255)},
                                           conf_threshold=args.conf_threshold)
            
            # 显示FPS和推理时间
            if args.show_fps:
                current_time = time.time()
                if current_time - fps_start_time >= 1.0:
                    fps_value = fps_counter / (current_time - fps_start_time)
                    fps_counter = 0
                    fps_start_time = current_time
                else:
                    fps_value = fps_counter / (current_time - fps_start_time) if fps_counter > 0 else 0
                
                info_text = [
                    f"FPS: {fps_value:.1f}",
                    f"AB Inference: {ab_time:.1f}ms",
                    f"RedLight Inference: {redlight_time:.1f}ms"
                ]
                y_offset = 20
                for text in info_text:
                    cv2.putText(result_img, text, (10, y_offset), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    y_offset += 25
            
            # 显示结果
            cv2.imshow('Real-time Detection (NPU)', result_img)
            
            # 按'q'退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # 打印检测信息（每30帧打印一次）
            if frame_count % 30 == 0:
                ab_count = ab_detections[0].shape[0] if len(ab_detections) > 0 and ab_detections[0].shape[0] > 0 else 0
                redlight_count = redlight_detections[0].shape[0] if len(redlight_detections) > 0 and redlight_detections[0].shape[0] > 0 else 0
                print(f"Frame {frame_count}: AB={ab_count}, RedLight={redlight_count}, "
                      f"AB_Time={ab_time:.1f}ms, RedLight_Time={redlight_time:.1f}ms")
    
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\n检测结束")


if __name__ == '__main__':
    main()

