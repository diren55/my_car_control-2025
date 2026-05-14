#!/usr/bin/env python3
"""
视觉控制测试节点 - 独立调试各项视觉功能
功能：
1. 红绿灯识别停车（识别到红灯停车3秒）
2. A/B标识牌识别并停到指定位置
3. 黄线停车
使用方法：ros2 run my_car_control vision_control_test
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Bool
import time


class VisionControlTest(Node):
    """视觉控制测试节点 - 用于独立调试各项视觉功能"""
    
    def __init__(self):
        super().__init__('vision_control_test')
        
        # 声明参数：测试模式选择
        self.declare_parameter('test_mode', 'all')  # all/red_light/ab_sign/yellow_line/ab_parking
        self.declare_parameter('ab_target', 'AUTO')   # A/B/AUTO - 目标停车位置（AUTO=自动匹配识别结果，A=左转，B=右转）
        self.declare_parameter('red_light_stop_time', 3.0)  # 红灯停车时间（秒）
        self.declare_parameter('ab_forward_time', 2.0)      # A/B识别后前进时间（秒）
        self.declare_parameter('ab_turn_angle', 45)         # A/B识别后转向角度（A=45度左转，B=-45度右转）
        self.declare_parameter('yellow_stop_time', 5.0)     # 黄线停车持续时间（秒）
        
        # AB车库停车参数
        self.declare_parameter('ab_parking_turn_angle', 45)  # AB车库转向角度（A区左转，B区右转）
        self.declare_parameter('ab_parking_forward_time', 1.5)  # AB车库前进时间（秒）
        self.declare_parameter('ab_parking_turn_time', 0.8)  # AB车库转向时间（秒）
        
        # 初始化参数
        self.declare_parameter('wheel_align_time', 2.0)  # 轮子回正时间（秒）
        
        # 速度参数（可配置）
        self.declare_parameter('base_speed', 25)      # 基础行驶速度（默认18）
        self.declare_parameter('stop_speed', -500)    # 停车速度（默认-500）
        self.declare_parameter('start_speed', 1500)   # 速度基准值（默认1500）
        self.declare_parameter('start_theta', 75)     # 角度基准值（默认75）
        
        # 获取参数
        self.test_mode = self.get_parameter('test_mode').value
        self.ab_target = self.get_parameter('ab_target').value.upper()
        self.red_light_stop_time = self.get_parameter('red_light_stop_time').value
        self.ab_forward_time = self.get_parameter('ab_forward_time').value
        self.ab_turn_angle_base = self.get_parameter('ab_turn_angle').value
        self.yellow_stop_time = self.get_parameter('yellow_stop_time').value
        
        # 获取AB车库停车参数
        self.ab_target_parking = self.get_parameter('ab_target').value.upper()  # AB车库目标位置
        self.ab_parking_turn_angle = self.get_parameter('ab_parking_turn_angle').value
        self.ab_parking_forward_time = self.get_parameter('ab_parking_forward_time').value
        self.ab_parking_turn_time = self.get_parameter('ab_parking_turn_time').value
        
        # 获取初始化参数
        self.wheel_align_time = self.get_parameter('wheel_align_time').value
        
        # 获取速度参数
        self.base_speed = self.get_parameter('base_speed').value
        self.stop_speed = self.get_parameter('stop_speed').value
        self.start_speed = self.get_parameter('start_speed').value
        self.start_theta = self.get_parameter('start_theta').value
        
        # 初始化状态
        self.wheel_align_start_time = None
        self.wheel_aligned = False
        
        # 状态变量
        self.red_light_detected = False
        self.red_light_stop_start_time = None
        self.red_light_stopped = False
        
        self.ab_detected = None  # 1=A, 2=B, None=未识别
        self.ab_recognized = False
        self.ab_stopped = False
        self.ab_turn_angle = 0  # 转向角度（A区=正角度左转，B区=负角度右转）
        
        self.yellow_line_detected = False
        self.yellow_line_stopped = False
        
        # AB车库停车状态
        self.ab_parking_detected = False  # 是否检测到车库区域（通过视觉检测"工"字型中间竖线）
        self.ab_parking_line_offset = 0.0  # 竖线位置偏移（-1.0到1.0，负数=左侧，正数=右侧，0=中心）
        self.ab_parking_started = False
        self.ab_parking_turn_start_time = None
        self.ab_parking_forward_start_time = None
        self.ab_parking_stopped = False
        
        # QoS配置
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅图像检测结果
        self.image_detection_sub = self.create_subscription(
            Int16MultiArray,
            '/image_detection',
            self.image_detection_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 注意：image_detection消息格式为 [红灯, A/B, 黄线, AB车库方向]
        
        # 发布车辆控制指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        
        # 定时器（20Hz控制频率）
        self.timer = self.create_timer(0.05, self.control_callback)
        
        # 开始轮子回正
        self.wheel_align_start_time = time.time()
        self.get_logger().info(f'🔄 开始轮子回正，持续 {self.wheel_align_time} 秒...')
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("视觉控制测试节点已启动")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"测试模式: {self.test_mode}")
        if self.test_mode == 'ab_sign':
            if self.ab_target == 'AUTO':
                self.get_logger().info(f"目标停车位置: AUTO（自动匹配识别结果）")
            else:
                self.get_logger().info(f"目标停车位置: {self.ab_target}")
        self.get_logger().info(f"红灯停车时间: {self.red_light_stop_time}秒")
        self.get_logger().info(f"基础行驶速度: {self.base_speed}")
        self.get_logger().info(f"速度基准值: {self.start_speed}")
        self.get_logger().info(f"角度基准值: {self.start_theta}")
        self.get_logger().info("=" * 60)
        
        # 启动提示
        if self.test_mode == 'all':
            self.get_logger().warn("⚠️  测试模式：所有功能（红绿灯、A/B标识、黄线）")
        elif self.test_mode == 'red_light':
            self.get_logger().warn("⚠️  测试模式：红绿灯停车（识别到红灯停车3秒）")
        elif self.test_mode == 'ab_sign':
            self.get_logger().warn(f"⚠️  测试模式：A/B标识识别停车（目标：{self.ab_target}）")
        elif self.test_mode == 'yellow_line':
            self.get_logger().warn("⚠️  测试模式：黄线停车")
        elif self.test_mode == 'ab_parking':
            self.get_logger().warn("⚠️  测试模式：AB车库停车")
    
    def drive(self, speed, theta):
        """驱动小车"""
        twist = Twist()
        twist.linear.x = float(self.start_speed + speed)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.start_theta + theta)
        twist.angular.z = float(max(min(twist.angular.z, 180.0), 0.0))
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_car(self):
        """停车"""
        self.drive(self.stop_speed, 0)
    
    def image_detection_callback(self, msg: Int16MultiArray):
        """图像检测结果回调"""
        if len(msg.data) < 3:
            return
        
        red_detect = msg.data[0]
        ab_detect = msg.data[1]
        yellow_detect = msg.data[2]
        ab_parking_dir = msg.data[3] if len(msg.data) > 3 else 0  # AB车库方向
        
        # 红绿灯检测
        if red_detect and not self.red_light_stopped:
            if not self.red_light_detected:
                self.get_logger().warn("🔴 检测到红灯！")
                self.red_light_detected = True
                self.red_light_stop_start_time = time.time()
        
        # A/B标识检测
        if ab_detect:
            if self.ab_detected != ab_detect:
                sign = 'A' if ab_detect == 1 else 'B'
                self.get_logger().info(f"🔵 识别到标识牌: {sign}")
                self.ab_detected = ab_detect
        
        # 黄线检测
        if yellow_detect:
            if not self.yellow_line_detected:
                self.get_logger().warn("🟡 检测到黄线！")
                self.yellow_line_detected = True
        
        # AB车库区域检测（检测黄色区域，当像素数达到阈值时触发）
        if ab_parking_dir != 0 and not self.ab_parking_stopped:  # ab_parking_dir是-100到100的整数
            if not self.ab_parking_detected:
                self.get_logger().warn(f"🅿️  检测到AB车库区域！立即触发停车")
                self.ab_parking_detected = True
                # 立即标记为已开始，避免延迟
                self.ab_parking_started = True
                self.ab_parking_turn_start_time = time.time()
            # 将整数偏移（-100到100）转换回浮点数（-1.0到1.0）
            self.ab_parking_line_offset = ab_parking_dir / 100.0
    
    def control_red_light(self):
        """红绿灯停车控制 - 识别到红灯后永久停车"""
        if self.red_light_detected:
            # 识别到红灯，永久停车
            self.stop_car()
            if not self.red_light_stopped:
                self.get_logger().warn("🔴 识别到红灯！永久停车（不再继续前进）")
                self.red_light_stopped = True
        else:
            # 未检测到红灯，正常行驶
            self.drive(self.base_speed, 0)
    
    def control_ab_sign(self):
        """
        A/B标识牌识别停车控制
        根据比赛规则：
        - A区在左侧（小车运行方向为参考）
        - B区在右侧（小车运行方向为参考）
        - 识别到A后停到左侧，识别到B后停到右侧
        """
        if self.ab_detected is None:
            # 未识别到标识牌，继续行驶
            self.drive(self.base_speed, 0)
            return
        
        sign = 'A' if self.ab_detected == 1 else 'B'
        target_sign = self.ab_target
        
        if not self.ab_recognized:
            # 刚识别到标识牌
            self.get_logger().warn(f"🔵 已识别标识牌: {sign}")
            self.ab_recognized = True
            self.ab_forward_start_time = time.time()
            # 根据识别结果决定转向方向
            # A区在左侧（y>0），B区在右侧（y<0）
            if sign == 'A':
                self.ab_turn_angle = 45  # 左转角度（增大y值）
                self.get_logger().info(f"  → 目标：A区（左侧），执行左转")
            else:
                self.ab_turn_angle = -45  # 右转角度（减小y值）
                self.get_logger().info(f"  → 目标：B区（右侧），执行右转")
        
        if not self.ab_stopped:
            # 识别后继续前进并转向进入停车区域
            elapsed = time.time() - self.ab_forward_start_time
            
            if elapsed < self.ab_forward_time:
                # 前进并转向进入对应停车区域
                self.drive(self.base_speed, self.ab_turn_angle)
            else:
                # 前进时间到，停车
                if sign == target_sign or target_sign == 'AUTO':  # AUTO模式自动匹配
                    self.get_logger().info(f"✅ 停车成功：已识别到{sign}，停到{sign}区域")
                else:
                    self.get_logger().warn(f"⚠️  识别到{sign}，但目标位置是{target_sign}")
                self.stop_car()
                self.ab_stopped = True
        else:
            # 已停车，保持停车状态
            self.stop_car()
    
    def control_yellow_line(self):
        """黄线停车控制"""
        if self.yellow_line_detected and not self.yellow_line_stopped:
            # 检测到黄线，停车
            self.stop_car()
            self.yellow_line_stopped = True
            self.get_logger().info("✅ 黄线停车完成")
        elif self.yellow_line_stopped:
            # 已停车，保持停车状态
            self.stop_car()
        else:
            # 未检测到黄线，正常行驶
            self.drive(self.base_speed, 0)
    
    def control_ab_parking(self):
        """
        AB车库停车 - 极简开环版 (Trigger & Action)
        策略：
        1. 看到竖线 = 触发信号
        2. 根据A/B目标，执行固定的"转向+前进"动作持续一定时间
        3. 时间到，停车
        """
        
        # 1. 如果已经完成停车，直接锁死
        if self.ab_parking_stopped:
            self.stop_car()
            return

        # 2. 确定目标方向 (A左 / B右)
        target_dir = 'A' # 默认A
        
        # 解析目标
        if self.ab_target_parking == 'A':
            target_dir = 'A'
        elif self.ab_target_parking == 'B':
            target_dir = 'B'
        elif self.ab_target_parking == 'AUTO':
            # 如果之前识别到了AB牌，就用识别结果
            if self.ab_detected == 2: # 2代表B
                target_dir = 'B'
            else:
                target_dir = 'A' # 1代表A，或者没识别到都默认A

        # 3. 触发逻辑：检测到后立即执行停车动作
        if not self.ab_parking_started:
            if self.ab_parking_detected:
                # 检测到后立即开始停车流程
                self.ab_parking_started = True
                self.ab_parking_turn_start_time = time.time()
                self.get_logger().warn(f"🅿️ 触发AB停车动作！目标: {target_dir}区，立即执行")
            else:
                # 还没检测到，继续直行找线
                self.drive(self.base_speed, 0)
                return

        # 4. 执行停车动作（检测到后立即执行，减少延迟）
        elapsed_time = time.time() - self.ab_parking_turn_start_time
        
        # 设定动作持续时间
        ACTION_DURATION = self.ab_parking_forward_time 
        
        if elapsed_time < ACTION_DURATION:
            # 正在执行动作：转向+前进（检测到后立即减速，避免超过停车区域）
            turn_angle = 0
            
            if target_dir == 'A':
                # A区：左转 (正角度)
                turn_angle = self.ab_parking_turn_angle 
            else:
                # B区：右转 (负角度)
                turn_angle = -self.ab_parking_turn_angle
            
            # 速度立即降低：检测到后立即减速到50%，然后逐渐降低
            if elapsed_time < ACTION_DURATION * 0.3:
                # 前30%时间：减速到50%速度（立即减速，减少超过停车区域的风险）
                current_speed = int(self.base_speed * 0.5)
            elif elapsed_time < ACTION_DURATION * 0.7:
                # 30%-70%：继续降低到30%
                speed_ratio = 0.5 - (elapsed_time - ACTION_DURATION * 0.3) / (ACTION_DURATION * 0.4) * 0.2
                current_speed = int(self.base_speed * max(0.3, speed_ratio))
            else:
                # 最后30%：降低到20%，准备停车
                speed_ratio = 0.3 - (elapsed_time - ACTION_DURATION * 0.7) / (ACTION_DURATION * 0.3) * 0.1
                current_speed = int(self.base_speed * max(0.2, speed_ratio))
            
            # 执行：一边跑一边转 (画弧线)，但速度已降低
            self.drive(current_speed, turn_angle)
            
            # 打印倒计时（降低频率，避免刷屏）
            if int(elapsed_time * 10) % 5 == 0:  # 每0.5秒打印一次
                self.get_logger().info(f"  -> 正在入库 ({target_dir}): {elapsed_time:.2f}/{ACTION_DURATION}s, 速度: {current_speed}")
            
        else:
            # 5. 时间到，立即停车
            self.stop_car()
            self.ab_parking_stopped = True
            self.get_logger().warn(f"✅ AB区域停车完成！")
    
    def control_callback(self):
        """控制回调 - 根据测试模式执行相应控制逻辑"""
        # 首先检查是否需要轮子回正
        if not self.wheel_aligned:
            if self.wheel_align_start_time is None:
                self.wheel_align_start_time = time.time()
            
            elapsed = time.time() - self.wheel_align_start_time
            if elapsed < self.wheel_align_time:
                # 回正中：速度=0，角度=0（相对于基准值）
                self.drive(0, 0)
                return
            else:
                # 回正完成
                self.wheel_aligned = True
                self.get_logger().info('✅ 轮子回正完成，开始测试')
        
        # 轮子回正后，执行正常的测试逻辑
        if self.test_mode == 'red_light':
            self.control_red_light()
        elif self.test_mode == 'ab_sign':
            self.control_ab_sign()
        elif self.test_mode == 'yellow_line':
            self.control_yellow_line()
        elif self.test_mode == 'ab_parking':
            self.control_ab_parking()
        elif self.test_mode == 'all':
            # 所有功能：优先级：AB车库 > 黄线 > A/B > 红绿灯
            if self.ab_parking_detected:
                self.control_ab_parking()
            elif self.yellow_line_detected:
                self.control_yellow_line()
            elif self.ab_detected is not None:
                self.control_ab_sign()
            elif self.red_light_detected:
                self.control_red_light()
            else:
                self.drive(self.base_speed, 0)
        else:
            # 默认：正常行驶
            self.drive(self.base_speed, 0)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = VisionControlTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

