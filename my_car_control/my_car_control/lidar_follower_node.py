#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # 导入传感器服务质量配置
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LidarFollower(Node):
    def __init__(self):
        super().__init__('lidar_follower_node')
        
        # --- 声明参数 (Tuning Parameters) ---
        # 这些参数需要你根据实际车辆和赛道进行精细调整
        
        # 1. P控制器增益 (Proportional Gain)
        # 决定了车辆对误差的反应速度。越大，转弯越急。
        # 注意：配置文件中的 angular_speed 用作P控制器增益的基础值
        self.declare_parameter('angular_speed', 0.5)
        angular_speed_config = self.get_parameter('angular_speed').get_parameter_value().double_value
        # 将配置文件中的 angular_speed 转换为P控制器增益（可根据实际调整转换系数）
        self.Kp_angular = angular_speed_config * 4.0  # 转换系数，可根据实际调整

        # 2. 车辆行驶速度 (m/s)
        # 规则要求第二圈加速 [cite: 20]，你可以先用一个慢速调第一圈
        self.declare_parameter('linear_speed', 0.2)
        self.forward_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        # 3. 探测角度 (radians)
        # 我们探测车辆前方偏左和偏右的角度
        # 规则中的"S"弯  和圆形区域 [cite: 52] 可能需要调整这些角度
        self.declare_parameter('scan_angle_left_deg', 60.0)
        self.declare_parameter('scan_angle_right_deg', -60.0)
        self.scan_angle_left = np.deg2rad(self.get_parameter('scan_angle_left_deg').get_parameter_value().double_value)
        self.scan_angle_right = np.deg2rad(self.get_parameter('scan_angle_right_deg').get_parameter_value().double_value)

        # 4. 安全停止距离 (m)
        # 如果正前方太近，就停车
        self.declare_parameter('safe_distance', 0.5)
        self.safety_distance = self.get_parameter('safe_distance').get_parameter_value().double_value

        # --- ROS2 订阅与发布 ---
        
        # 订阅激光雷达数据
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',  # 激光雷达的话题名称
            self.scan_callback,
            qos_profile=qos_profile_sensor_data  # 使用传感器专用的QoS
        )
        
        # 发布速度控制指令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # 控制小车运动的话题
            10
        )
        
        self.get_logger().info("Python雷达循迹节点已启动。")
        self.get_logger().info(f"速度: {self.forward_speed} m/s, 角速度: {self.angular_speed} rad/s, P增益: {self.Kp_angular}")

    def scan_callback(self, msg: LaserScan):
        """
        处理 LaserScan 消息的回调函数
        """
        
        # --- 1. 查找探测角度对应的索引 ---
        left_index = self.get_index_from_angle(msg, self.scan_angle_left)
        right_index = self.get_index_from_angle(msg, self.scan_angle_right)
        front_index = self.get_index_from_angle(msg, 0.0) # 正前方

        if left_index is None or right_index is None or front_index is None:
            self.get_logger().warn("探测角度超出了雷达范围, 检查角度设置。")
            return

        # --- 2. 获取距离数据 ---
        dist_left = msg.ranges[left_index]
        dist_right = msg.ranges[right_index]
        dist_front = msg.ranges[front_index]

        # --- 3. 处理无效数据 (inf, nan) ---
        # 如果雷达读数无效 (例如太远或太近)，我们假定一个安全距离 (例如1.5m)
        # 赛道宽度为1.5m [cite: 32]，所以两侧距离不太可能无限远
        max_safe_range = 1.5
        if not np.isfinite(dist_left): dist_left = max_safe_range
        if not np.isfinite(dist_right): dist_right = max_safe_range
        if not np.isfinite(dist_front): dist_front = max_safe_range


        # --- 4. 安全检查 (E-Stop) ---
        # 如果正前方有障碍物 (锥桶或挡板 [cite: 25]) 太近，则紧急停止
        if dist_front < self.safety_distance:
            self.get_logger().warn(f"正前方障碍物过近 ({dist_front:.2f}m)! 停车。")
            self.publish_control(0.0, 0.0)
            return

        # --- 5. 计算P控制器误差 ---
        # 目标：让 dist_left 和 dist_right 相等
        # error > 0: 离右侧近 (dist_left > dist_right)，需要向左转 (角速度为正)
        # error < 0: 离左侧近 (dist_left < dist_right)，需要向右转 (角速度为负)
        error = dist_left - dist_right
        
        # --- 6. 计算控制量 ---
        # P控制器: 角速度 = Kp * 误差
        angular_velocity = self.Kp_angular * error
        
        # 线速度保持恒定 (你也可以根据转弯幅度动态调整线速度)
        linear_velocity = self.forward_speed
        
        # --- 7. 发布控制指令 ---
        self.publish_control(linear_velocity, angular_velocity)
        
        # (可选) 打印调试信息
        # self.get_logger().info(f"L:{dist_left:.2f} R:{dist_right:.2f} | Err:{error:.2f} | AngVel:{angular_velocity:.2f}")


    def get_index_from_angle(self, scan_msg: LaserScan, target_angle_rad: float):
        """
        辅助函数：根据给定的角度(弧度)计算它在 'ranges' 数组中的索引
        """
        index = int((target_angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        if 0 <= index < len(scan_msg.ranges):
            return index
        else:
            return None # 角度超出范围

    def publish_control(self, linear_vel: float, angular_vel: float):
        """辅助函数：发布Twist消息"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist_msg)

    def on_shutdown(self):
        """节点关闭时执行的操作"""
        self.get_logger().info("节点关闭，正在停止小车...")
        self.publish_control(0.0, 0.0) # 发送停止命令

def main(args=None):
    rclpy.init(args=args)
    
    lidar_follower_node = LidarFollower()
    
    try:
        rclpy.spin(lidar_follower_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 在退出前执行清理
        lidar_follower_node.on_shutdown()
        lidar_follower_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()