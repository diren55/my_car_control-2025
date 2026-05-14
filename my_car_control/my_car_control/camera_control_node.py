#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头参数实时调整节点
功能: 通过ROS2参数动态调整摄像头硬件参数（曝光、亮度、对比度等）
话题: 可选发布 /image_raw
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CameraControlNode(Node):
    """摄像头参数实时调整节点"""
    
    def __init__(self):
        super().__init__('camera_control_node')
        
        # ========== 声明参数 ==========
        # 摄像头设备参数
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('publish_image', True)  # 是否发布图像
        self.declare_parameter('image_topic', '/image_raw')  # 发布话题名
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('buffer_size', 1)
        
        # 显示窗口参数
        self.declare_parameter('show_window', True)  # 是否显示画面窗口
        self.declare_parameter('show_trackbars', True)  # 是否显示滑动条
        
        # 摄像头硬件参数（-1表示使用默认值或自动）
        self.declare_parameter('brightness', -1.0)      # 亮度: 0-255, -1=自动
        self.declare_parameter('contrast', -1.0)        # 对比度: 0-255, -1=自动
        self.declare_parameter('saturation', -1.0)      # 饱和度: 0-255, -1=自动
        self.declare_parameter('hue', -1.0)              # 色调: 0-255, -1=自动
        self.declare_parameter('gain', -1.0)             # 增益: 0-100, -1=自动
        self.declare_parameter('exposure', -1.0)        # 曝光: 通常1-10000, -1=自动
        self.declare_parameter('auto_exposure', -1.0)    # 自动曝光: 0=关闭, 1=开启, -1=默认
        self.declare_parameter('white_balance', -1.0)   # 白平衡: 通常2800-6500, -1=自动
        self.declare_parameter('auto_white_balance', -1.0)  # 自动白平衡: 0=关闭, 1=开启, -1=默认
        self.declare_parameter('sharpness', -1.0)        # 锐度: 0-255, -1=自动
        self.declare_parameter('gamma', -1.0)           # 伽马: 通常100-300, -1=自动
        self.declare_parameter('backlight_compensation', -1.0)  # 背光补偿: 0-1, -1=自动
        self.declare_parameter('power_line_frequency', -1.0)  # 电源频率: 0=禁用, 1=50Hz, 2=60Hz, -1=默认
        
        # 获取参数
        video_device = self.get_parameter('video_device').value
        self.publish_image = self.get_parameter('publish_image').value
        image_topic = self.get_parameter('image_topic').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        buffer_size = self.get_parameter('buffer_size').value
        self.show_window = self.get_parameter('show_window').value
        self.show_trackbars = self.get_parameter('show_trackbars').value
        
        # ========== 初始化摄像头 ==========
        # 使用与video.py相同的方式打开摄像头
        self.bridge = CvBridge()
        self.cap = None
        
        # 将设备路径转换为索引（如果提供的是/dev/video0，转换为0）
        import os
        if video_device.startswith('/dev/video'):
            try:
                device_index = int(video_device.replace('/dev/video', ''))
            except:
                device_index = 0
        else:
            device_index = 0
        
        # 使用与video.py相同的方式：cv2.VideoCapture(0)
        self.get_logger().info(f"打开摄像头，设备索引: {device_index} (对应 {video_device})")
        self.cap = cv2.VideoCapture(device_index)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ 无法打开摄像头: {video_device} (索引: {device_index})")
            raise RuntimeError(f"Camera open failed: {video_device}")
        
        # 读取摄像头的实际分辨率（与video.py一致）
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f"摄像头实际分辨率: {actual_width}x{actual_height}, FPS: {actual_fps}")
        
        # 如果用户指定了分辨率，尝试设置（但不强制）
        if frame_width != actual_width or frame_height != actual_height:
            self.get_logger().info(f"尝试设置分辨率为: {frame_width}x{frame_height}")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            # 重新读取实际设置后的分辨率
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"设置后实际分辨率: {actual_width}x{actual_height}")
        
        # 设置FPS（如果指定）
        if fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # 设置缓冲区大小（如果指定）
        if buffer_size > 0:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
        
        # 自动设置并锁定曝光、白平衡、增益
        self.auto_set_and_lock_camera_params()
        
        # 应用所有硬件参数（如果用户指定了参数，会覆盖自动设置的值）
        self.apply_all_camera_parameters()
        
        # ========== 初始化显示窗口 ==========
        if self.show_window:
            cv2.namedWindow('摄像头画面 - 参数调整', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('摄像头画面 - 参数调整', 800, 600)
            
            if self.show_trackbars:
                # 创建滑动条窗口
                cv2.namedWindow('参数调节', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('参数调节', 400, 600)
                
                # 初始化滑动条值（读取当前参数或使用默认值）
                self.init_trackbar_values()
                
                # 创建滑动条
                self.create_trackbars()
        
        # ========== 创建发布者（可选）==========
        if self.publish_image:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.image_pub = self.create_publisher(Image, image_topic, qos_profile)
        
        # ========== 动态参数回调 ==========
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # ========== 定时器（用于发布图像和显示窗口）==========
        timer_period = 1.0 / fps
        if self.publish_image or self.show_window:
            self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # ========== 日志输出 ==========
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ 摄像头参数控制节点已启动!')
        self.get_logger().info(f'   设备: {video_device}')
        self.get_logger().info(f'   分辨率: {frame_width}x{frame_height}')
        self.get_logger().info(f'   帧率: {fps} fps')
        if self.publish_image:
            self.get_logger().info(f'   发布话题: {image_topic}')
        self.get_logger().info('')
        self.get_logger().info('📝 使用以下命令实时调整参数:')
        self.get_logger().info('   ros2 param set /camera_control_node brightness 128')
        self.get_logger().info('   ros2 param set /camera_control_node exposure 50')
        self.get_logger().info('   ros2 param set /camera_control_node contrast 64')
        self.get_logger().info('   ros2 param set /camera_control_node saturation 128')
        self.get_logger().info('=' * 60)
        
        # 打印当前参数值
        self.print_current_parameters()
        
        # 显示操作说明
        if self.show_window:
            self.get_logger().info('')
            self.get_logger().info('🖥️  画面窗口已打开!')
            self.get_logger().info('【操作说明】')
            if self.show_trackbars:
                self.get_logger().info('  1. 在"参数调节"窗口中拖动滑动条实时调整参数')
                self.get_logger().info('  2. 在"摄像头画面"窗口中查看实时效果')
            self.get_logger().info('  3. 按 "q" 键退出')
            self.get_logger().info('  4. 按 "s" 键保存当前参数到终端')
            self.get_logger().info('  5. 按 "r" 键重置所有参数为默认值')
            self.get_logger().info('')
    
    def init_trackbar_values(self):
        """初始化滑动条值"""
        # 读取当前参数值，如果为-1则使用合理的默认值
        self.trackbar_values = {
            'brightness': max(0, min(255, int(self.cap.get(cv2.CAP_PROP_BRIGHTNESS))) if self.cap.get(cv2.CAP_PROP_BRIGHTNESS) >= 0 else 128),
            'contrast': max(0, min(255, int(self.cap.get(cv2.CAP_PROP_CONTRAST))) if self.cap.get(cv2.CAP_PROP_CONTRAST) >= 0 else 64),
            'saturation': max(0, min(255, int(self.cap.get(cv2.CAP_PROP_SATURATION))) if self.cap.get(cv2.CAP_PROP_SATURATION) >= 0 else 128),
            'exposure': max(1, min(100, int(self.cap.get(cv2.CAP_PROP_EXPOSURE))) if self.cap.get(cv2.CAP_PROP_EXPOSURE) > 0 else 50),
            'gain': max(0, min(100, int(self.cap.get(cv2.CAP_PROP_GAIN))) if self.cap.get(cv2.CAP_PROP_GAIN) >= 0 else 50),
            'white_balance': max(2800, min(6500, int(self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE))) if self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE) > 0 else 4000),
            'sharpness': max(0, min(255, int(self.cap.get(cv2.CAP_PROP_SHARPNESS))) if self.cap.get(cv2.CAP_PROP_SHARPNESS) >= 0 else 128),
        }
    
    def create_trackbars(self):
        """创建滑动条"""
        def on_brightness(val):
            self.trackbar_values['brightness'] = val
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, val)
        
        def on_contrast(val):
            self.trackbar_values['contrast'] = val
            self.cap.set(cv2.CAP_PROP_CONTRAST, val)
        
        def on_saturation(val):
            self.trackbar_values['saturation'] = val
            self.cap.set(cv2.CAP_PROP_SATURATION, val)
        
        def on_exposure(val):
            self.trackbar_values['exposure'] = val
            # 先关闭自动曝光
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, val)
        
        def on_gain(val):
            self.trackbar_values['gain'] = val
            self.cap.set(cv2.CAP_PROP_GAIN, val)
        
        def on_white_balance(val):
            self.trackbar_values['white_balance'] = val
            # 先关闭自动白平衡
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, val)
        
        def on_sharpness(val):
            self.trackbar_values['sharpness'] = val
            self.cap.set(cv2.CAP_PROP_SHARPNESS, val)
        
        # 创建滑动条
        cv2.createTrackbar('亮度 (Brightness)', '参数调节', 
                          self.trackbar_values['brightness'], 255, on_brightness)
        cv2.createTrackbar('对比度 (Contrast)', '参数调节', 
                          self.trackbar_values['contrast'], 255, on_contrast)
        cv2.createTrackbar('饱和度 (Saturation)', '参数调节', 
                          self.trackbar_values['saturation'], 255, on_saturation)
        cv2.createTrackbar('曝光 (Exposure)', '参数调节', 
                          self.trackbar_values['exposure'], 100, on_exposure)
        cv2.createTrackbar('增益 (Gain)', '参数调节', 
                          self.trackbar_values['gain'], 100, on_gain)
        cv2.createTrackbar('白平衡 (WB)', '参数调节', 
                          (self.trackbar_values['white_balance'] - 2800) // 10, 
                          (6500 - 2800) // 10, 
                          lambda val: on_white_balance(2800 + val * 10))
        cv2.createTrackbar('锐度 (Sharpness)', '参数调节', 
                          self.trackbar_values['sharpness'], 255, on_sharpness)
    
    def parameters_callback(self, params):
        """动态参数回调函数"""
        result = []
        for param in params:
            param_name = param.name
            param_value = param.value
            
            # 应用参数到摄像头
            success = self.apply_camera_parameter(param_name, param_value)
            
            if success:
                result.append(Parameter(param_name, param_value).to_parameter_msg())
                self.get_logger().info(f'✅ 参数已更新: {param_name} = {param_value}')
            else:
                self.get_logger().warn(f'⚠️  参数设置失败: {param_name} = {param_value}')
        
        return rclpy.node.SetParametersResult(successful=True, reason='')
    
    def apply_camera_parameter(self, param_name, value):
        """应用单个摄像头参数"""
        if value == -1.0 or value is None:
            return True  # -1表示使用默认值，跳过设置
        
        try:
            if param_name == 'brightness':
                return self.cap.set(cv2.CAP_PROP_BRIGHTNESS, value)
            elif param_name == 'contrast':
                return self.cap.set(cv2.CAP_PROP_CONTRAST, value)
            elif param_name == 'saturation':
                return self.cap.set(cv2.CAP_PROP_SATURATION, value)
            elif param_name == 'hue':
                return self.cap.set(cv2.CAP_PROP_HUE, value)
            elif param_name == 'gain':
                return self.cap.set(cv2.CAP_PROP_GAIN, value)
            elif param_name == 'exposure':
                return self.cap.set(cv2.CAP_PROP_EXPOSURE, value)
            elif param_name == 'auto_exposure':
                # 先关闭自动曝光，然后设置手动曝光
                if value == 0:
                    self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = 手动模式
                elif value == 1:
                    self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # 0.75 = 自动模式
                return True
            elif param_name == 'white_balance':
                return self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, value)
            elif param_name == 'auto_white_balance':
                return self.cap.set(cv2.CAP_PROP_AUTO_WB, value)
            elif param_name == 'sharpness':
                return self.cap.set(cv2.CAP_PROP_SHARPNESS, value)
            elif param_name == 'gamma':
                return self.cap.set(cv2.CAP_PROP_GAMMA, value)
            elif param_name == 'backlight_compensation':
                return self.cap.set(cv2.CAP_PROP_BACKLIGHT, value)
            elif param_name == 'power_line_frequency':
                return self.cap.set(cv2.CAP_PROP_XI_POWER_LINE_FREQUENCY, value)
            elif param_name in ['frame_width', 'frame_height', 'fps', 'buffer_size']:
                # 这些参数需要重新打开摄像头，暂时跳过
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f'设置参数 {param_name} 时出错: {e}')
            return False
    
    def auto_set_and_lock_camera_params(self):
        """自动设置曝光、白平衡、增益，然后锁定"""
        self.get_logger().info('')
        self.get_logger().info('🔧 开始自动设置相机参数...')
        
        # 检查是否已经手动设置了这些参数
        auto_exposure_param = self.get_parameter('auto_exposure').value
        auto_white_balance_param = self.get_parameter('auto_white_balance').value
        exposure_param = self.get_parameter('exposure').value
        white_balance_param = self.get_parameter('white_balance').value
        gain_param = self.get_parameter('gain').value
        
        # 如果用户已经手动设置了参数（不是-1），则跳过自动设置
        if (auto_exposure_param != -1.0 or exposure_param != -1.0 or 
            auto_white_balance_param != -1.0 or white_balance_param != -1.0 or 
            gain_param != -1.0):
            self.get_logger().info('   检测到手动设置的参数，跳过自动设置')
            return
        
        try:
            # 步骤1: 开启自动模式
            self.get_logger().info('   步骤1: 开启自动曝光、自动白平衡、自动增益...')
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # 0.75 = 自动曝光
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)  # 1 = 自动白平衡
            # 增益通常跟随自动曝光，不需要单独设置
            
            # 步骤2: 等待相机自动调整（读取几帧让相机稳定）
            self.get_logger().info('   步骤2: 等待相机自动调整（约2秒）...')
            import time
            start_time = time.time()
            frame_count = 0
            while time.time() - start_time < 2.0:  # 等待2秒
                ret, _ = self.cap.read()
                if ret:
                    frame_count += 1
                time.sleep(0.1)  # 每100ms读取一帧
            
            self.get_logger().info(f'   已读取 {frame_count} 帧用于自动调整')
            
            # 步骤3: 读取当前自动调整后的值
            current_exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            current_white_balance = self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
            current_gain = self.cap.get(cv2.CAP_PROP_GAIN)
            
            self.get_logger().info(f'   自动调整后的值:')
            self.get_logger().info(f'     曝光 (Exposure): {current_exposure:.2f}')
            self.get_logger().info(f'     白平衡 (WB): {current_white_balance:.0f}K')
            self.get_logger().info(f'     增益 (Gain): {current_gain:.2f}')
            
            # 步骤4: 关闭自动模式，锁定这些值
            self.get_logger().info('   步骤3: 锁定参数值（关闭自动模式）...')
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = 手动曝光
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # 0 = 手动白平衡
            
            # 设置锁定后的值
            if current_exposure > 0:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
            if current_white_balance > 0:
                self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, current_white_balance)
            if current_gain >= 0:
                self.cap.set(cv2.CAP_PROP_GAIN, current_gain)
            
            # 验证锁定是否成功
            time.sleep(0.5)  # 等待设置生效
            locked_exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            locked_white_balance = self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
            locked_gain = self.cap.get(cv2.CAP_PROP_GAIN)
            
            self.get_logger().info('   ✅ 参数已锁定:')
            self.get_logger().info(f'     曝光 (Exposure): {locked_exposure:.2f}')
            self.get_logger().info(f'     白平衡 (WB): {locked_white_balance:.0f}K')
            self.get_logger().info(f'     增益 (Gain): {locked_gain:.2f}')
            self.get_logger().info('')
            
        except Exception as e:
            self.get_logger().warn(f'   ⚠️  自动设置参数时出错: {e}')
            self.get_logger().warn('   将使用默认参数或手动设置的参数')
            self.get_logger().info('')
    
    def apply_all_camera_parameters(self):
        """应用所有摄像头参数"""
        param_names = [
            'brightness', 'contrast', 'saturation', 'hue', 'gain',
            'exposure', 'auto_exposure', 'white_balance', 'auto_white_balance',
            'sharpness', 'gamma', 'backlight_compensation', 'power_line_frequency'
        ]
        
        for param_name in param_names:
            try:
                param_value = self.get_parameter(param_name).value
                if param_value != -1.0:
                    self.apply_camera_parameter(param_name, param_value)
            except Exception as e:
                self.get_logger().warn(f'应用参数 {param_name} 时出错: {e}')
    
    def print_current_parameters(self):
        """打印当前摄像头参数值"""
        self.get_logger().info('')
        self.get_logger().info('📊 当前摄像头参数值:')
        self.get_logger().info('-' * 60)
        
        # 读取实际值
        brightness = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
        contrast = self.cap.get(cv2.CAP_PROP_CONTRAST)
        saturation = self.cap.get(cv2.CAP_PROP_SATURATION)
        exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
        gain = self.cap.get(cv2.CAP_PROP_GAIN)
        white_balance = self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
        
        self.get_logger().info(f'   亮度 (Brightness): {brightness:.1f}')
        self.get_logger().info(f'   对比度 (Contrast): {contrast:.1f}')
        self.get_logger().info(f'   饱和度 (Saturation): {saturation:.1f}')
        self.get_logger().info(f'   曝光 (Exposure): {exposure:.1f}')
        self.get_logger().info(f'   增益 (Gain): {gain:.1f}')
        self.get_logger().info(f'   白平衡 (White Balance): {white_balance:.1f}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('')
    
    def timer_callback(self):
        """定时器回调（发布图像和显示窗口）"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('⚠️  无法读取摄像头帧')
            return
        
        # 发布图像（如果启用）
        if self.publish_image:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_frame"
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'发布图像时出错: {e}')
        
        # 显示窗口（如果启用）
        if self.show_window:
            display_frame = frame.copy()
            
            # 在画面上显示当前参数值
            self.draw_parameters_on_frame(display_frame)
            
            # 显示画面
            cv2.imshow('摄像头画面 - 参数调整', display_frame)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('退出摄像头控制节点')
                rclpy.shutdown()
            elif key == ord('s'):
                self.save_current_parameters()
            elif key == ord('r'):
                self.reset_parameters()
    
    def draw_parameters_on_frame(self, frame):
        """在画面上绘制当前参数值"""
        # 读取当前参数值
        brightness = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
        contrast = self.cap.get(cv2.CAP_PROP_CONTRAST)
        saturation = self.cap.get(cv2.CAP_PROP_SATURATION)
        exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
        gain = self.cap.get(cv2.CAP_PROP_GAIN)
        white_balance = self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
        sharpness = self.cap.get(cv2.CAP_PROP_SHARPNESS)
        
        # 创建信息文本
        y_offset = 30
        line_height = 25
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 1
        
        # 绘制半透明背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (350, 200), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # 绘制参数文本
        cv2.putText(frame, f'Brightness: {brightness:.1f}', 
                   (15, y_offset), font, font_scale, (0, 255, 0), thickness)
        y_offset += line_height
        
        cv2.putText(frame, f'Contrast: {contrast:.1f}', 
                   (15, y_offset), font, font_scale, (0, 255, 0), thickness)
        y_offset += line_height
        
        cv2.putText(frame, f'Saturation: {saturation:.1f}', 
                   (15, y_offset), font, font_scale, (0, 255, 0), thickness)
        y_offset += line_height
        
        cv2.putText(frame, f'Exposure: {exposure:.1f}', 
                   (15, y_offset), font, font_scale, (0, 255, 0), thickness)
        y_offset += line_height
        
        cv2.putText(frame, f'Gain: {gain:.1f}', 
                   (15, y_offset), font, font_scale, (0, 255, 0), thickness)
        y_offset += line_height
        
        cv2.putText(frame, f'WB: {white_balance:.0f}K', 
                   (15, y_offset), font, font_scale, (0, 255, 0), thickness)
        y_offset += line_height
        
        cv2.putText(frame, f'Sharpness: {sharpness:.1f}', 
                   (15, y_offset), font, font_scale, (0, 255, 0), thickness)
        y_offset += line_height
        
        # 操作提示
        y_offset += 10
        cv2.putText(frame, 'Press Q: Quit | S: Save | R: Reset', 
                   (15, y_offset), font, 0.5, (255, 255, 255), thickness)
    
    def save_current_parameters(self):
        """保存当前参数到终端"""
        brightness = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
        contrast = self.cap.get(cv2.CAP_PROP_CONTRAST)
        saturation = self.cap.get(cv2.CAP_PROP_SATURATION)
        exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
        gain = self.cap.get(cv2.CAP_PROP_GAIN)
        white_balance = self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
        sharpness = self.cap.get(cv2.CAP_PROP_SHARPNESS)
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📋 当前摄像头参数值:')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'ros2 param set /camera_control_node brightness {int(brightness)}')
        self.get_logger().info(f'ros2 param set /camera_control_node contrast {int(contrast)}')
        self.get_logger().info(f'ros2 param set /camera_control_node saturation {int(saturation)}')
        self.get_logger().info(f'ros2 param set /camera_control_node exposure {int(exposure)}')
        self.get_logger().info(f'ros2 param set /camera_control_node gain {int(gain)}')
        self.get_logger().info(f'ros2 param set /camera_control_node white_balance {int(white_balance)}')
        self.get_logger().info(f'ros2 param set /camera_control_node sharpness {int(sharpness)}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
    
    def reset_parameters(self):
        """重置所有参数为默认值"""
        self.get_logger().info('重置所有参数为默认值...')
        
        # 恢复自动模式
        try:
            # 恢复自动曝光
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # 自动模式
            # 恢复自动白平衡
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        except:
            pass
        
        # 重新初始化滑动条值（会读取当前摄像头值）
        if self.show_trackbars:
            self.init_trackbar_values()
            for param_name, value in self.trackbar_values.items():
                try:
                    if param_name == 'white_balance':
                        cv2.setTrackbarPos('白平衡 (WB)', '参数调节', 
                                         (value - 2800) // 10)
                    elif param_name == 'brightness':
                        cv2.setTrackbarPos('亮度 (Brightness)', '参数调节', value)
                    elif param_name == 'contrast':
                        cv2.setTrackbarPos('对比度 (Contrast)', '参数调节', value)
                    elif param_name == 'saturation':
                        cv2.setTrackbarPos('饱和度 (Saturation)', '参数调节', value)
                    elif param_name == 'exposure':
                        cv2.setTrackbarPos('曝光 (Exposure)', '参数调节', value)
                    elif param_name == 'gain':
                        cv2.setTrackbarPos('增益 (Gain)', '参数调节', value)
                    elif param_name == 'sharpness':
                        cv2.setTrackbarPos('锐度 (Sharpness)', '参数调节', value)
                except:
                    pass
        
        self.get_logger().info('✅ 参数已重置为自动模式')
    
    def destroy_node(self):
        """清理资源"""
        if self.show_window:
            cv2.destroyAllWindows()
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info('摄像头已关闭')
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = CameraControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

