#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制节点 - 基于NCSC2024源程序racecar_teleop_test.py
- 订阅感知节点的目标点话题
- 执行PID控制逻辑
- 发布车辆驱动指令
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool, String, Int16MultiArray
from std_srvs.srv import Empty
from cartographer_ros_msgs.srv import WriteState, FinishTrajectory
from my_car_control.navigation_manager import NavigationManager
import math
import numpy as np
import time
import os


class ControlNode(Node):
    """控制节点 - 基于源程序racecar_teleop_test.py的Route类"""
    
    def __init__(self):
        super().__init__('control_node')
        
        # --- 声明参数（基于源程序racecar_teleop_test.py）---
        # V7: 黄线弯参数已硬编码到代码中（不再从配置文件读取）
        
        # V8: 新增向右修正转向角
        self.declare_parameter('right_correction_angle', -5.0) # 向右修正（三等分点为空）的固定转向角（-5度，最终角度=75-5=70度）

        # 停车时间参数
        self.declare_parameter('target_stop_duration', 4.0)  # 红灯停车持续时间（秒）

        # 坐标打印参数
        self.declare_parameter('print_position', True)   # 是否打印XY坐标（用于手推车获取停车点坐标）
        self.declare_parameter('position_print_interval', 0.5)  # 坐标打印间隔（秒）

        # XY坐标停车参数（支持多停车点，按圈数配置）
        self.declare_parameter('enable_xy_parking', False)  # 是否启用XY坐标停车
        self.declare_parameter('park_points', [])  # 停车点列表，格式: [[x, y, 容差(米), 停车时间(秒), 圈数], ...]

        # 导航路径点记录参数（基于源程序racecar_teleop_test.py）
        self.declare_parameter('enable_navigation_recording', True)  # 是否启用第一圈路径点记录
        self.declare_parameter('navigation_csv_path', os.path.expanduser('~/navigation_points.csv'))  # 路径点CSV文件路径
        self.declare_parameter('record_interval', 0.5)  # 第一圈记录路径点的时间间隔（秒），基于源程序t=0.5
        self.declare_parameter('navigation_min_distance', 0.8)  # 第二圈到达目标点的最小距离阈值（米），基于源程序minDist=0.8
        self.declare_parameter('enable_navigation_loop', True)  # 是否启用第二圈循环导航

        # 速度控制参数（提高基础速度，保持安全范围）
        self.declare_parameter('base_speed', 25)           # 基础速度档位 (1525)
        self.declare_parameter('circle1_speed', 25)        # 第一圈速度 (1525)
        self.declare_parameter('circle2_speed', 25)        # 第二圈速度 (1525)
        self.declare_parameter('park_speed', 25)           # 停车速度 (1525)
        
        # PID参数（基于源程序的pidList）- 低速降P，高速提P
        self.declare_parameter('pid_params_5', [-45.0, 0.0, 0.3])     # 直角弯后续帧专用 (超低速)
        self.declare_parameter('pid_params_10', [-55.0, 0.0, 0.5])    # 备用
        self.declare_parameter('pid_params_12', [-60.0, 0.0, 0.6])    # 黄线弯专用 (低速)
        self.declare_parameter('pid_params_16', [-65.0, 0.0, 0.8])    # 备用
        self.declare_parameter('pid_params_18', [-70.0, 0.0, 1.0])  # 极低速PID
        self.declare_parameter('pid_params_19', [-75.0, 0.0, 1.2])  # 超低速PID
        self.declare_parameter('pid_params_20', [-80.0, 0.0, 1.0])  # 低速PID
        self.declare_parameter('pid_params_22', [-85.0, 0.0, 1.2])  # 中低速PID
        self.declare_parameter('pid_params_25', [-110.0, 0.0, 3.5])  # 第一圈PID - 大幅提高P值增强响应速度
        self.declare_parameter('pid_params_28', [-115.0, 0.0, 4.0])  # 第二圈PID - 大幅提高P值增强响应速度
        self.declare_parameter('pid_params_30', [-110.0, 0.0, 0.15])  # 备用
        self.declare_parameter('pid_params_35', [-120.0, 0.0, 0.18])  # 备用
        self.declare_parameter('pid_params_45', [-80.0, 0.0, 0.1])
        self.declare_parameter('pid_params_50', [-90.0, 0.0, 0.2])
        self.declare_parameter('pid_params_65', [-90.0, 0.0, 0.1])
        self.declare_parameter('pid_params_70', [-140.0, 0.0, 0.5])
        self.declare_parameter('pid_params_75', [-120.0, 0.0, 20.0])
        self.declare_parameter('pid_params_80', [-145.0, 0.0, 10.0])
        self.declare_parameter('pid_params_85', [-185.0, 0.0, 15.0])
        
        # 实时速度动态PID参数（雷达循迹模式使用，移植自line_follow.py）
        self.declare_parameter('velocity_pid_low_speed_threshold', 1.5)  # 低速阈值（m/s）
        self.declare_parameter('velocity_pid_low_speed_params', [-60.0, 0.0, 30.0])  # 低速PID参数
        self.declare_parameter('velocity_pid_high_speed_params', [-65.0, 0.0, 30.0])  # 高速PID参数
        self.declare_parameter('velocity_pid_max_velocity', 2.5)  # 最大速度限制（m/s）
        
        # 动态PID参数
        self.declare_parameter('pid_k_factor', 0.3)        # 直道PID中P的比例因子
        self.declare_parameter('max_error', 100.0)         # 最大误差（用于积分分离）
        self.declare_parameter('alpha_factor', -0.05)      # 积分分离Alpha因子
        self.declare_parameter('sum_err_threshold', 100.0) # 积分项上限
        self.declare_parameter('min_dist_threshold', 0.3)  # 到达目标点的最小距离
        
        # 转向参数（基于源程序的turnTheta）- 最终优化：增大转向角
        self.declare_parameter('turn_theta_3', [70, 70])  # 直角弯后续帧专用 (1503) → 145°（75+70）
        self.declare_parameter('turn_theta_5', [80, 80])  # 直角弯后续帧备用 (1505) → 155°（75+80，增大角度避免冲出去）
        self.declare_parameter('turn_theta_10', [40, 40])  # 备用 (1510) → 115°（75+40）
        self.declare_parameter('turn_theta_12', [45, 45])  # (1512) → 120°（75+45）
        self.declare_parameter('turn_theta_16', [37, 37])  # 备用 (1516) → 112°（75+37）
        self.declare_parameter('turn_theta_18', [36, 36])  # 极低速转向 (1518) → 111°（加大4°）
        self.declare_parameter('turn_theta_19', [35, 35])  # 超低速转向 (1519) → 110°（加大4°）
        self.declare_parameter('turn_theta_20', [34, 34])  # 低速转向 (1520) → 109°（加大4°）
        self.declare_parameter('turn_theta_22', [33, 33])  # 中低速转向 (1522) → 108°（加大4°）
        self.declare_parameter('turn_theta_25', [34, 34])  # 第一圈正常 (1525) → 109°（加大4°）
        self.declare_parameter('turn_theta_28', [32, 32])  # 第二圈正常 (1528) → 107°（加大4°）
        self.declare_parameter('turn_theta_30', [40, 40])  # 备用转向 (1530)
        self.declare_parameter('turn_theta_35', [42, 42])  # 备用转向 (1535)
        self.declare_parameter('turn_theta_45', [30, 30])
        self.declare_parameter('turn_theta_50', [35, 35])
        self.declare_parameter('turn_theta_65', [36, 36])
        self.declare_parameter('turn_theta_70', [45, 45])
        self.declare_parameter('turn_theta_75', [45, 47])
        self.declare_parameter('turn_theta_80', [55, 60])
        self.declare_parameter('turn_theta_85', [100, 100])
        
        # 获取参数
        # V7: 黄线弯参数（硬编码，基于control_params.yaml第6-8行）
        # 最终角度 = 75 + 27 = 102度 (减小1度：28->27)
        self.yellow_curve_angle = 25.0  # 黄线弯（无锥桶）的固定转向角
        self.yellow_curve_speed = 25    # 黄线弯固定速度档位（改为25档）
        
        # V8: 获取向右修正参数
        self.right_correction_angle = self.get_parameter('right_correction_angle').value

        # 获取停车时间参数
        self.target_stop_duration = self.get_parameter('target_stop_duration').value

        # 获取坐标打印参数
        self.print_position = self.get_parameter('print_position').value
        self.position_print_interval = self.get_parameter('position_print_interval').value
        self.last_position_print_time = 0.0  # 上次打印坐标的时间

        # 获取导航路径点记录参数
        self.enable_navigation_recording = self.get_parameter('enable_navigation_recording').value
        self.navigation_csv_path = self.get_parameter('navigation_csv_path').value
        self.record_interval = self.get_parameter('record_interval').value
        self.navigation_min_distance = self.get_parameter('navigation_min_distance').value
        self.enable_navigation_loop = self.get_parameter('enable_navigation_loop').value

        # 获取XY坐标停车参数
        self.enable_xy_parking = self.get_parameter('enable_xy_parking').value
        park_points_raw = self.get_parameter('park_points').value

        self.base_speed = self.get_parameter('base_speed').value
        self.circle1_speed = self.get_parameter('circle1_speed').value
        self.circle2_speed = self.get_parameter('circle2_speed').value
        self.park_speed = self.get_parameter('park_speed').value
        
        # PID参数字典
        self.pid_params = {
            5: self.get_parameter('pid_params_5').value,
            10: self.get_parameter('pid_params_10').value,
            12: self.get_parameter('pid_params_12').value,
            16: self.get_parameter('pid_params_16').value,
            18: self.get_parameter('pid_params_18').value,
            19: self.get_parameter('pid_params_19').value,
            20: self.get_parameter('pid_params_20').value,
            22: self.get_parameter('pid_params_22').value,
            25: self.get_parameter('pid_params_25').value,
            28: self.get_parameter('pid_params_28').value,
            30: self.get_parameter('pid_params_30').value,
            35: self.get_parameter('pid_params_35').value,
            45: self.get_parameter('pid_params_45').value,
            50: self.get_parameter('pid_params_50').value,
            65: self.get_parameter('pid_params_65').value,
            70: self.get_parameter('pid_params_70').value,
            75: self.get_parameter('pid_params_75').value,
            80: self.get_parameter('pid_params_80').value,
            85: self.get_parameter('pid_params_85').value
        }
        
        # 实时速度动态PID参数（雷达循迹模式使用）
        self.velocity_pid_low_speed_threshold = self.get_parameter('velocity_pid_low_speed_threshold').value
        self.velocity_pid_low_speed_params = self.get_parameter('velocity_pid_low_speed_params').value
        self.velocity_pid_high_speed_params = self.get_parameter('velocity_pid_high_speed_params').value
        self.velocity_pid_max_velocity = self.get_parameter('velocity_pid_max_velocity').value
        
        # 动态PID参数
        self.pid_k_factor = self.get_parameter('pid_k_factor').value
        self.max_error = self.get_parameter('max_error').value
        self.alpha_factor = self.get_parameter('alpha_factor').value
        self.sum_err_threshold = self.get_parameter('sum_err_threshold').value
        self.min_dist_threshold = self.get_parameter('min_dist_threshold').value
        
        # 转向参数字典
        self.turn_theta = {
            3: self.get_parameter('turn_theta_3').value,
            5: self.get_parameter('turn_theta_5').value,
            10: self.get_parameter('turn_theta_10').value,
            12: self.get_parameter('turn_theta_12').value,
            16: self.get_parameter('turn_theta_16').value,
            18: self.get_parameter('turn_theta_18').value,
            19: self.get_parameter('turn_theta_19').value,
            20: self.get_parameter('turn_theta_20').value,
            22: self.get_parameter('turn_theta_22').value,
            25: self.get_parameter('turn_theta_25').value,
            28: self.get_parameter('turn_theta_28').value,
            30: self.get_parameter('turn_theta_30').value,
            35: self.get_parameter('turn_theta_35').value,
            45: self.get_parameter('turn_theta_45').value,
            50: self.get_parameter('turn_theta_50').value,
            65: self.get_parameter('turn_theta_65').value,
            70: self.get_parameter('turn_theta_70').value,
            75: self.get_parameter('turn_theta_75').value,
            80: self.get_parameter('turn_theta_80').value,
            85: self.get_parameter('turn_theta_85').value
        }
        
        # --- 状态变量（基于源程序Route类）---
        self.route_point = ()          # 目标点
        self.absolute_point = ()        # 小车绝对坐标（已归零）
        self.relative_point = ()        # 相对坐标系下的xy坐标
        self.angle = 0.0                # 小车姿态角度，范围[-pi,pi]
        self.last_error = 0.0           # 上次的误差，用于pid微分
        self.sum_error = 0.0            # 误差积分，用于pid积分
        self.origin_p = self.pid_params[self.circle1_speed][0]  # 保存原始P值
        self.last_normal_theta = 0.0    # 最后一次非停车指令的转向角度（用于-40000模式恢复）
        self.last_twist = None          # 最后一次发送的Twist消息（用于检测小车是否停止）
        self.current_velocity = 0.0       # 实时速度变量（m/s），从/odom_combined中提取（移植自line_follow.py）
        
        # 里程计归零相关状态
        self.odom_initialized = False   # 是否已初始化里程计零点
        self.odom_offset_x = 0.0        # 里程计初始X偏移量
        self.odom_offset_y = 0.0        # 里程计初始Y偏移量
        
        # 运行状态
        self.circle = 0                 # 圈数：0为初始状态， 1为第一圈，2为第二圈
        self.start = False              # 是否开始运行
        self.stop = False               # 是否停车
        self.finished_2_circles = False  # 是否已完成两圈（第二圈第二停车区停车后）
        self.map_saved = False          # 是否已保存第一圈地图
        self.map_save_future = None     # 地图保存的异步future（用于后台检查）
        self.trajectory_finished = False # 是否已停止建图（第二圈）
        self.finish_trajectory_future = None  # 停止建图的异步future（用于后台检查）
        self.red = 0                    # 红灯检测
        self.yellow = 0                 # 黄线检测
        self.park = False               # 是否进入停车模式
        self.cooldown = 0               # 红灯冷却时间
        self.A_B = 2                    # 1为A，2为B。预设为B
        
        # 红灯停车状态（用于正常跑圈时的红灯停车）
        self.red_light_stopping = False  # 是否正在红灯停车
        self.red_light_stop_start_time = None  # 红灯停车开始时间
        self.red_light_stop_duration = 3.5  # 红灯停车持续时间（秒）
        self.last_red_light_log_time = 0.0  # 上次打印红灯检测日志的时间（用于节流）
        self.image_detection_received = False  # 是否收到过图像检测消息
        self.image_detection_last_time = time.time()  # 上次收到图像检测消息的时间（初始化为当前时间）
        self.just_switched_left = False # 是否刚切入左转模式（用于抑制切换抖动）
        self.in_yellow_curve_mode = False # V7: 是否在黄线弯模式中（锁定模式，直到识别到有效目标点）
        self.yellow_curve_first_frame = False # V11: 黄线弯首帧标志（用于角度增强）
        
        # V10: 目标区域停止相关状态
        self.in_target_stop_area = False    # 是否在第一个目标停止区域
        self.target_stop_start_time = None  # 停止开始时间
        # self.target_stop_duration 已在上面从参数获取（默认3.0秒）
        self.target_stop_completed = False  # 本次进入第一个区域是否已完成停车（防止重复触发）
        
        # 第二个目标停止区域相关状态
        self.in_target_stop_area2 = False    # 是否在第二个目标停止区域
        self.target_stop_start_time2 = None  # 第二个区域停止开始时间
        self.target_stop_completed2 = False  # 本次进入第二个区域是否已完成停车（防止重复触发）
        self._precise_park_started2 = False  # 是否已开始精确微调（进入检测范围即开始微调）
        
        # 第二个停车区域精确停车控制（停在赛道右半部分）
        self.precise_park_target2_x = -2.40   # 目标停车位置X坐标（可根据实际调整）-1.41
        self.precise_park_target2_y = -0.30    # 目标停车位置Y坐标（正值偏向右侧，可根据实际调整）
        self.precise_park_target2_angle = 0.0  # 目标停车角度（弧度，0表示朝向X轴正方向，可根据实际调整）
        self.precise_park_tolerance = 0.30    # 精确停车位置容差（米）容许误差 
        self.precise_park_angle_tolerance = 0.1  # 精确停车角度容差（弧度，约5.7度）
        self.precise_park_reached2 = False    # 是否已到达精确停车位置和角度
        self.precise_park_last_distance = None  # 上一次到目标点的距离（用于检测距离开始增大）
        
        # XY坐标停车相关状态
        self.park_points = []                # 停车点列表（格式：[{x, y, tolerance, duration, circle, completed}, ...]）
        self.current_park_index = 0          # 当前要到达的停车点索引
        self.xy_park_reached = False        # 是否已到达当前停车点
        self.xy_park_start_time = None      # 停车开始时间
        self.xy_park_completed = False      # 当前停车点是否已完成
        
        # 解析停车点配置
        if self.enable_xy_parking:
            for i, point in enumerate(park_points_raw):
                if len(point) >= 5:
                    self.park_points.append({
                        'x': float(point[0]),
                        'y': float(point[1]),
                        'tolerance': float(point[2]),
                        'duration': float(point[3]),
                        'circle': int(point[4]),  # 1=第一圈，2=第二圈
                        'completed': False
                    })
                    self.get_logger().info(
                        f"停车点{i+1}: ({point[0]}, {point[1]}), "
                        f"容差{point[2]}m, 停车{point[3]}秒, 第{point[4]}圈"
                    )
            self.get_logger().info(f"已配置{len(self.park_points)}个停车点")
        
        # 控制参数
        self.v = self.circle1_speed     # 当前速度档位
        self.last_twist = None          # 上次的控制指令
        
        # --- QoS配置 ---
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- ROS2 订阅与发布 ---
        # 订阅目标点（来自感知节点）
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            '/target',
            self.target_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 订阅里程计（用于位置和姿态）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_combined',  # 使用实际的里程计话题
            self.odom_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 订阅图像检测结果（用于红灯检测）
        self.image_detection_sub = self.create_subscription(
            Int16MultiArray,
            '/image_detection',  # 图像检测话题：[红灯, A/B, 黄线, AB车库方向]
            self.image_detection_callback,
            qos_profile=qos_profile_sensor
        )
        
        # 发布车辆控制指令（使用teleop_cmd_vel话题，与ROS2驱动器匹配）
        self.cmd_vel_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        
        # 发布导航开始信号
        self.navi_pub = self.create_publisher(Bool, '/navi', 10)
        
        # 发布开始信号
        self.start_pub = self.create_publisher(Bool, '/start', 10)
        
        # 发布导航目标点（用于Nav2导航，第二圈循环发点）
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Cartographer服务客户端（用于保存地图和停止建图）
        self.write_state_client = self.create_client(WriteState, '/write_state')
        self.finish_trajectory_client = self.create_client(FinishTrajectory, '/finish_trajectory')
        
        # 地图保存路径（默认保存在home目录下的maps文件夹）
        self.map_save_dir = os.path.expanduser('~/maps')
        os.makedirs(self.map_save_dir, exist_ok=True)
        self.map_save_path = os.path.join(self.map_save_dir, 'first_circle_map.pbstream')
        
        # --- 初始化导航管理器（基于源程序Navigator类）---
        if self.enable_navigation_recording:
            self.navigation_manager = NavigationManager(
                csv_file_path=self.navigation_csv_path,
                logger=self.get_logger(),
                record_interval=self.record_interval,
                min_distance=self.navigation_min_distance
            )
        else:
            self.navigation_manager = None
        
        # 定时器（20Hz控制频率）
        self.timer = self.create_timer(0.05, self.control_callback)
        
        # 第一圈路径点记录定时器（基于源程序：timer = rospy.Timer(rospy.Duration(t), navigator.timer)）
        if self.enable_navigation_recording and self.navigation_manager:
            self.record_timer = self.create_timer(self.record_interval, self.record_navigation_point)
        else:
            self.record_timer = None
        
        # 日志
        self.get_logger().info("=" * 60)
        self.get_logger().info("控制节点已启动（基于NCSC2024源程序）")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"速度档位: 第一圈={self.circle1_speed}, 第二圈={self.circle2_speed}, 停车={self.park_speed}")
        self.get_logger().info(f"实时速度PID参数（雷达循迹模式）:")
        self.get_logger().info(f"  低速阈值: {self.velocity_pid_low_speed_threshold} m/s")
        self.get_logger().info(f"  低速PID: {self.velocity_pid_low_speed_params} (速度 < {self.velocity_pid_low_speed_threshold} m/s)")
        self.get_logger().info(f"  高速PID: {self.velocity_pid_high_speed_params} (速度 >= {self.velocity_pid_low_speed_threshold} m/s)")
        self.get_logger().info(f"  最大速度限制: {self.velocity_pid_max_velocity} m/s")
        self.get_logger().info(f"传统PID参数表（用于active_pid函数）: {self.pid_params}")
        self.get_logger().info(f"动态PID: K因子={self.pid_k_factor}, 积分分离阈值={self.max_error}")
        self.get_logger().info(f"转向参数: {self.turn_theta}")
        self.get_logger().info(f"坐标打印: 启用={self.print_position}, 间隔={self.position_print_interval}秒")
        self.get_logger().info(f"XY坐标停车: 启用={self.enable_xy_parking}, 停车点数量={len(self.park_points)}")
        self.get_logger().info("=" * 60)
    
    def get_pid_params_by_velocity(self, velocity):
        """
        根据实时速度返回对应的PID参数（移植自line_follow.py）
        从配置文件读取参数，支持动态调整
        """
        # 限制速度范围
        velocity = max(0.0, min(velocity, self.velocity_pid_max_velocity))
        
        # 根据速度阈值选择PID参数
        if velocity < self.velocity_pid_low_speed_threshold:
            return self.velocity_pid_low_speed_params
        
        # 速度大于等于阈值时使用高速PID参数
        return self.velocity_pid_high_speed_params
    
    def pid_control(self, error, last_error, sum_error, velocity):
        """
        PID控制（移植自line_follow.py，使用实时速度动态查找PID参数）
        采用积分分离法，对积分系数进行抛物线控制(误差小时积分项控制作用大)
        """
        # 根据实时速度查找PID参数（移植自line_follow.py）
        p, i, d = self.get_pid_params_by_velocity(velocity)
        
        # 积分分离法（基于源程序）
        if abs(error) < self.max_error:
            i = (error - self.max_error) ** 2 / self.max_error ** 2 * self.alpha_factor
        
        output_theta = p * error + i * sum_error + d * (error - last_error)
        return output_theta
    
    def drive(self, v, theta, last=False):
        """
        驱动小车（基于源程序racecar_teleop_test.py的drive函数）
        输入小车的v和舵机角度，驱动小车
        """
        if last:
            try:
                self.cmd_vel_pub.publish(self.last_twist)
            except:
                pass
            return
        
        start_speed = 1500
        start_theta = 75
        
        twist = Twist()
        twist.linear.x = float(start_speed + v)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(start_theta + theta)
        twist.angular.z = float(max(min(twist.angular.z, 180.0), 0.0))
        
        # 调试输出
        if self.absolute_point != ():
            self.get_logger().info(
                f'位置: ({self.absolute_point[0]:.2f}, {self.absolute_point[1]:.2f}) | '
                f'转向: {twist.angular.z:.2f}',
                throttle_duration_sec=0.1
            )
        
        self.cmd_vel_pub.publish(twist)
        self.last_twist = twist
        
        # 如果不是停车指令，记录转向角度（用于-40000模式恢复）
        if v != -500:
            self.last_normal_theta = theta
    
    def save_map_async(self):
        """异步保存Cartographer地图为pbstream文件（完全不阻塞）"""
        if not self.write_state_client.wait_for_service(timeout_sec=0.1):
            # 服务不可用，返回None，不阻塞
            return None
        
        try:
            request = WriteState.Request()
            request.filename = self.map_save_path
            request.include_unfinished_submaps = False  # 只保存完成的子图
            
            future = self.write_state_client.call_async(request)
            # 完全异步，不等待，立即返回future
            return future
        except Exception as e:
            # 异常也不阻塞
            return None
    
    def check_map_save_result(self):
        """检查地图保存结果（在主循环中定期调用）"""
        if self.map_save_future is not None and self.map_save_future.done():
            try:
                response = self.map_save_future.result()
                if response.status.code == 0:  # 0表示成功
                    # 地图保存成功，可以在这里添加日志
                    pass
                else:
                    # 保存失败，可以在这里添加日志
                    pass
            except Exception as e:
                # 异常处理
                pass
            finally:
                self.map_save_future = None  # 清除future
    
    def finish_trajectory_async(self):
        """异步停止Cartographer建图（结束当前轨迹，完全不阻塞）"""
        if not self.finish_trajectory_client.wait_for_service(timeout_sec=0.1):
            # 服务不可用，返回None，不阻塞
            return None
        
        try:
            request = FinishTrajectory.Request()
            request.trajectory_id = 0  # 默认轨迹ID为0
            
            future = self.finish_trajectory_client.call_async(request)
            # 完全异步，不等待，立即返回future
            return future
        except Exception as e:
            # 异常也不阻塞
            return None
    
    def check_finish_trajectory_result(self):
        """检查停止建图结果（在主循环中定期调用）"""
        if self.finish_trajectory_future is not None and self.finish_trajectory_future.done():
            try:
                response = self.finish_trajectory_future.result()
                if response.status.code == 0:  # 0表示成功
                    # 停止建图成功，可以在这里添加日志
                    pass
                else:
                    # 停止建图失败，可以在这里添加日志
                    pass
            except Exception as e:
                # 异常处理
                pass
            finally:
                self.finish_trajectory_future = None  # 清除future
    
    def update_origin_p(self):
        """更新原始P值（基于源程序）"""
        self.origin_p = self.pid_params[self.v][0]
    
    def detect_cooldown(self):
        """检测冷却时间（基于源程序）"""
        if self.cooldown > 0:
            self.red = 0
            self.cooldown -= 1
    
    def set_car_point_and_angle(self, msg):
        """刷新小车绝对坐标与角度（基于源程序）"""
        # 获取原始里程计坐标
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        
        # 第一次接收到里程计数据时，记录初始位置作为零点（与perception_node.py保持一致）
        if not self.odom_initialized:
            self.odom_offset_x = raw_x
            self.odom_offset_y = raw_y
            self.odom_initialized = True
        
        # 使用归零后的坐标（与perception_node.py的停车区域判断一致，与打印坐标一致）
        if self.odom_initialized:
            self.absolute_point = (raw_x - self.odom_offset_x, raw_y - self.odom_offset_y)
        else:
            self.absolute_point = (raw_x, raw_y)
        
        # 从四元数提取yaw角度
        orientation = msg.pose.pose.orientation
        # 使用简化的四元数到欧拉角转换
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.angle = np.round(math.atan2(siny_cosp, cosy_cosp), 6)
    
    def set_goal_point(self, p):
        """设置目标点（基于源程序）"""
        self.route_point = (p[0], p[1])
    
    def set_relative_point(self, p):
        """设置相对点（基于源程序）"""
        self.relative_point = p
    
    def delete_goal_point(self):
        """删除目标点（基于源程序）"""
        if self.route_point != ():
            if (abs(self.absolute_point[0] - self.route_point[0]) + 
                abs(self.absolute_point[1] - self.route_point[1])) < self.min_dist_threshold:
                self.route_point = ()
    
    def get_route_theta(self):
        """返回小车和目标点的夹角（基于源程序）"""
        if self.route_point == ():
            return 0.0
        return math.atan2(self.route_point[1] - self.absolute_point[1], 
                         self.route_point[0] - self.absolute_point[0])
    
    def tf_coordinates(self):
        """根据目标点相对坐标进行坐标变换，并设置绝对坐标下的目标点（基于源程序）"""
        if self.absolute_point == () or self.relative_point == ():
            return
        
        (x0, y0) = self.absolute_point
        (x2, y2) = self.relative_point
        theta1 = self.angle
        theta2 = math.atan2(y2, x2)
        relative_length = math.sqrt(x2 ** 2 + y2 ** 2)
        
        self.set_goal_point((
            x0 + relative_length * math.cos(theta1 + theta2), 
            y0 + relative_length * math.sin(theta1 + theta2)
        ))
    
    def turn_left(self):
        """左转（基于源程序）"""
        if self.v in self.turn_theta:
            turn_angle = self.turn_theta[self.v][self.circle - 1]
            self.drive(self.v, turn_angle)
    
    def turn_left_with_speed(self, speed):
        """左转并指定速度（避免冲到内侧）"""
        # 使用基础速度的转向角度（如果指定速度不存在）
        if speed in self.turn_theta:
            turn_angle = self.turn_theta[speed][self.circle - 1]
        elif self.v in self.turn_theta:
            turn_angle = self.turn_theta[self.v][self.circle - 1]
        else:
            turn_angle = 28  # 默认值（对应103°）
        self.drive(speed, turn_angle)
    
    def start_navi(self):
        """开始导航（基于源程序）"""
        self.navi_pub.publish(Bool(data=False))
        time.sleep(3)
        self.navi_pub.publish(Bool(data=True))
    
    def active_pid(self, if_line):
        """动态PID（基于源程序）"""
        if if_line:  # 进入直道
            self.pid_params[self.v][0] = self.pid_params[self.v][0] * self.pid_k_factor
        else:
            self.pid_params[self.v][0] = self.origin_p
    
    def stop_the_car(self):
        """停车检测（基于源程序）"""
        # 注释掉黄线停车区识别（已改用XY坐标停车）
        # if self.park and self.yellow:
        #     self.stop = True
    
    def car_drive(self):
        """
        车辆驱动逻辑（基于源程序）- V10版（新增目标停止区域模式）
        区分七种模式：
        1. 第一个目标停止区域 (X = -99999) - 红灯识别区，到达指定区域，停车3秒
        2. 第二个目标停止区域 (X = -88888) - (-9.25, -6.86)区域，停车3秒
        3. 正常循迹 (X > -9000)
        4. 直角弯 (X = -20000) - 感知失败
        5. 黄线弯 (X = -10000) - 扇形检测无锥桶（锁定模式，直到识别到有效目标点）
        6. 向右修正 (X = -30000) - 三等分点为空（内外道配对失败或外道点被过滤）
        7. 保持当前状态 (X = -40000) - 内道不足但外道点中等（外道=5或6）
        """
        # 检查地图保存结果和停止建图结果（非阻塞）
        self.check_map_save_result()
        self.check_finish_trajectory_result()
        
        # 检查是否已完成两圈（第二圈第二停车区停车后），如果是则停止车辆（已注释）
        # if self.finished_2_circles:
        #     self.drive(-500, 0)  # 停止车辆
        #     return
        
        # === V10: 最高优先级 - 检查是否到达目标停止区域（已注释） ===
        # 第一个停车区域（红灯识别区）-99999.0
        # if self.relative_point and self.relative_point[0] == -99999.0:
        #     # --- 模式0.1：第一个目标停止区域（最高优先级，停车3秒后继续）---
        #     self.get_logger().warn(f">>> 【识别到红灯】<<<")
        #     
        #     # 如果本次进入已经完成过停车，直接跳过（等待离开区域）
        #     if self.target_stop_completed:
        #         self.get_logger().warn(f">>> 【识别到红灯】已完成停车 <<<")
        #         # 不停车，继续执行后续的正常驾驶逻辑
        #         pass
        #     elif not self.in_target_stop_area:
        #         # 首次进入第一个停止区域（且未完成停车）
        #         # 只要进入停车区域就开始停车，立即增加圈数
        #         old_circle = self.circle
        #         self.circle += 1
        #         
        #         # 第二圈开始时，停止建图（避免覆盖第一圈的地图，异步，不阻塞）
        #         if old_circle == 1 and self.circle == 2 and not self.trajectory_finished:
        #             self.get_logger().warn(f">>> 【第二圈开始】停止建图，避免覆盖第一圈地图 <<<")
        #             # 异步停止建图，不阻塞主循环
        #             self.finish_trajectory_future = self.finish_trajectory_async()
        #             self.trajectory_finished = True  # 标记为已尝试停止，避免重复调用
        #         
        #         self.get_logger().warn(f">>> 【识别到红灯】开始停车！<<<")
        #         self.target_stop_completed = True  # 标记已完成停车，防止重复触发
        #         self.in_target_stop_area = True
        #         self.target_stop_start_time = time.time()
        #         self.drive(-500, 0)
        #         return
        #     else:
        #         # 已经在第一个停止区域中，检查是否已停止足够时间
        #         elapsed_time = time.time() - self.target_stop_start_time
        #         
        #         if elapsed_time < self.target_stop_duration:
        #             # 继续停车，显示剩余时间
        #             remaining_time = self.target_stop_duration - elapsed_time
        #             self.get_logger().info(f">>> 第一个区域停车中... 剩余 {remaining_time:.1f}秒 <<<", throttle_duration_sec=0.5)
        #             self.drive(-500, 0)
        #             return
        #         else:
        #             # 已停车3秒，标记为已完成，继续运行
        #             # 注意：圈数已在进入停车区域时增加，这里不需要再次增加
        #             self.get_logger().warn(">>> 【停车完成】继续运行！ <<<")
        #             self.in_target_stop_area = False
        #             self.target_stop_start_time = None
        #             # 不return，继续执行后续的正常驾驶逻辑
        # else:
        #     # 不在第一个停止区域，重置所有停止状态（准备下次进入）
        #     # 注意：圈数已在进入停车区域时增加，这里不需要检查停车时间
        #     if self.in_target_stop_area or self.target_stop_completed:
        #         # 离开第一个停车区域时重置状态，允许下次进入时再次停车和增加圈数
        #         self.get_logger().warn(f">>> 【离开红绿灯区域】状态已重置！当前圈数={self.circle} <<<")
        #         
        #         # 第一圈结束时（circle==1且刚离开红绿灯区域），保存地图（异步，不阻塞）
        #         if self.circle == 1 and not self.map_saved:
        #             self.get_logger().warn(f">>> 【第一圈完成】开始保存地图... <<<")
        #             # 异步保存地图，不阻塞主循环
        #             self.map_save_future = self.save_map_async()
        #             self.map_saved = True  # 标记为已尝试保存，避免重复调用
        #         
        #         self.in_target_stop_area = False
        #         self.target_stop_start_time = None
        #         self.target_stop_completed = False  # 重置完成标志，允许下次进入时再次停车
        
        # 第二个停车区域（-9.25, -6.86）-88888.0（已注释）
        # 注意：只在第二圈及以后（第一次红灯停车之后，即圈数>=2）才会触发
        # 关键：第一圈进入第二停车区时，忽略-88888.0信号，继续正常循迹
        
        # 如果第一圈进入第二停车区，明确忽略-88888.0信号，不处理，继续正常循迹
        # if self.relative_point and self.relative_point[0] == -88888.0 and self.circle < 2:
        #     # 第一圈进入第二停车区，忽略此信号，继续正常循迹
        #     # 不设置任何状态，让后续的正常循迹逻辑处理
        #     pass
        # 如果已经在精确停车模式中，或者收到-88888.0信号且圈数>=2，执行精确停车逻辑
        # elif self._precise_park_started2 or (self.relative_point and self.relative_point[0] == -88888.0 and self.circle >= 2):
        #     # --- 模式0.2：第二个目标停止区域（最高优先级，精确停车后停车3秒）---
        #     # 只在第二圈时才处理第二个停车区
        #     if not self._precise_park_started2:
        #         # 首次进入，记录日志
        #         self.get_logger().warn(f">>> 【进入停车识别区域】 第二圈执行停车")
        #     # 如果已经在精确停车模式中，继续执行，不记录日志（避免刷屏）
        #     
        #     if self.target_stop_completed2:
        #         # 如果本次进入已经完成过停车，直接跳过（等待离开区域）
        #         self.get_logger().debug(">>> 本次进入第二个区域已完成停车，继续运行... <<<")
        #         # 不停车，继续执行后续的正常驾驶逻辑
        #         pass
        #     elif not self.in_target_stop_area2:
        #         # 首次进入第二个停止区域（且未完成停车）
        #         # 收到-88888.0信号即进入检测范围，开始精确微调模式
        #         self.in_target_stop_area2 = True
        #         self.precise_park_reached2 = False  # 重置精确停车标志
        #         self.precise_park_last_distance = None  # 重置距离记录，开始新的距离跟踪
        #         self._precise_park_started2 = True
        #         self.get_logger().warn(f">>> 识别到【标注B】正在停车")
        #         # 继续执行后续逻辑进行精确停车计算
        #     
        #     # 已经在第二个停止区域中（包括首次进入的后续处理）
        #     # 进入检测范围即开始精确微调模式，慢速进入目标点
        #     if self.in_target_stop_area2 and not self.precise_park_reached2:
        #         if self.absolute_point == ():
        #             # 如果坐标还未准备好，等待并停止
        #             self.get_logger().warn(">>> 【等待坐标数据】精确停车需要坐标信息... <<<")
        #             self.drive(-500, 0)
        #             return
        #         
        #         # absolute_point 已准备好，进行精确停车计算
        #         # 计算到目标位置的距离（仅用于判断是否到达精确位置）
        #         current_x, current_y = self.absolute_point
        #         dx = self.precise_park_target2_x - current_x
        #         dy = self.precise_park_target2_y - current_y
        #         distance = math.sqrt(dx * dx + dy * dy)
        #         
        #         # 检测距离是否开始增大（说明已经过了最近点，开始远离目标点）
        #         if self.precise_park_last_distance is not None:
        #             # 如果当前距离 > 上一次距离，说明开始远离目标点，立即停车
        #             if distance > self.precise_park_last_distance:
        #                 self.precise_park_reached2 = True
        #                 self.target_stop_start_time2 = time.time()
        #                 self.get_logger().warn(
        #                      f">>> 【检测到距离开始增大】上一距离={self.precise_park_last_distance:.3f}m, "
        #                      f"当前距离={distance:.3f}m, 在最近点停车！位置(X={current_x:.3f}, Y={current_y:.3f}) <<<"
        #                  )
        #                 self.precise_park_last_distance = None  # 重置，避免重复触发
        #                 self.drive(-500, 0)
        #                 return
        #         
        #         # 更新上一次的距离
        #         self.precise_park_last_distance = distance
        #         
        #         # 计算角度误差（目标角度 - 当前角度）
        #         angle_error = self.precise_park_target2_angle - self.angle
        #         # 归一化角度误差到[-pi, pi]
        #         while angle_error > math.pi:
        #             angle_error -= 2 * math.pi
        #         while angle_error < -math.pi:
        #             angle_error += 2 * math.pi
        #         
        #         # 检查是否同时满足位置和角度要求
        #         position_ok = distance <= self.precise_park_tolerance
        #         angle_ok = abs(angle_error) <= self.precise_park_angle_tolerance
        #         
        #         if position_ok and angle_ok:
        #             # 已到达精确停车位置和角度
        #             self.precise_park_reached2 = True
        #             self.target_stop_start_time2 = time.time()
        #             self.precise_park_last_distance = None  # 重置距离记录
        #             self.get_logger().warn(
        #                 f">>> 【精确停车位置和角度已到达】(X={current_x:.3f}, Y={current_y:.3f}, "
        #                 f"角度={math.degrees(self.angle):.1f}°)，开始停车3秒！ <<<"
        #             )
        #             self.drive(-500, 0)
        #             return
        #         else:
        #             # 需要继续微调到目标位置和角度
        #             # 策略：直接朝向目标点，根据距离动态调整速度
        #             # 计算朝向目标点的角度
        #             target_move_angle = math.atan2(dy, dx)
        #             move_angle_error = target_move_angle - self.angle
        #             # 归一化角度误差到[-pi, pi]
        #             while move_angle_error > math.pi:
        #                 move_angle_error -= 2 * math.pi
        #             while move_angle_error < -math.pi:
        #                 move_angle_error += 2 * math.pi
        #             
        #             # 根据距离动态调整速度：距离越近速度越慢，确保能精确到达
        #             if distance > 1.0:
        #                 # 距离较远，使用黄线弯速度
        #                 speed = self.yellow_curve_speed
        #             elif distance > 0.5:
        #                 # 距离中等，降低速度
        #                 speed = max(8, self.yellow_curve_speed - 8)
        #             elif distance > 0.3:
        #                 # 距离较近，进一步降低速度
        #                 speed = max(5, self.yellow_curve_speed - 11)
        #             else:
        #                 # 距离很近，极慢速度
        #                 speed = max(3, self.yellow_curve_speed - 14)
        #             
        #             # 转向角度：直接朝向目标点，根据距离调整转向灵敏度
        #             if distance > 0.5:
        #                 # 距离较远，转向角度较大，确保能快速对准目标点
        #                 theta = max(-25, min(25, move_angle_error * 180 / math.pi * 3))
        #             else:
        #                 # 距离较近，转向角度更细致，避免过调
        #                 theta = max(-25, min(25, move_angle_error * 180 / math.pi * 2))
        #             
        #             # 如果位置已达标但角度未达标，主要调整角度
        #             if position_ok and not angle_ok:
        #                 theta = max(-25, min(25, angle_error * 180 / math.pi * 3))
        #             
        #             self.get_logger().warn(
        #                 f">>> 【精确停车微调】位置距离={distance:.3f}m{'✓' if position_ok else '✗'}, "
        #                 f"角度误差={math.degrees(angle_error):.1f}°{'✓' if angle_ok else '✗'}, "
        #                 f"目标方向={math.degrees(target_move_angle):.1f}°, "
        #                 f"速度={speed}, 转向={theta:.1f}° <<<"
        #             )
        #             self.drive(speed, theta)
        #             return
        #     
        #     # 已到达精确位置，开始停车计时
        #     if self.in_target_stop_area2 and self.precise_park_reached2:
        #         elapsed_time = time.time() - self.target_stop_start_time2
        #         
        #         if elapsed_time < self.target_stop_duration:
        #             # 继续停车，显示剩余时间
        #             remaining_time = self.target_stop_duration - elapsed_time
        #             self.get_logger().info(f">>> 第二个区域停车中... 剩余 {remaining_time:.1f}秒 <<<", throttle_duration_sec=0.5)
        #             self.drive(-500, 0)
        #             return
        #         else:
        #             # 已停车3秒，标记为已完成
        #             # 如果是第二圈的第二停车区，标记为已完成两圈，停止车辆
        #             if self.circle == 2:
        #                 self.finished_2_circles = True
        #                 self.get_logger().warn(">>> 【第二圈第二停车区停车3秒完成】已完成两圈，车辆停止！ <<<")
        #                 self.in_target_stop_area2 = False
        #                 self.target_stop_start_time2 = None
        #                 self.precise_park_reached2 = False
        #                 self.precise_park_last_distance = None  # 重置距离记录
        #                 self._precise_park_started2 = False  # 重置微调标志
        #                 self.target_stop_completed2 = True  # 标记本次进入已完成停车
        #                 self.drive(-500, 0)  # 停止车辆
        #                 return
        #             else:
        #                 # 不是第二圈，恢复正常循迹模式
        #                 self.get_logger().warn(">>> 【第二个区域停车3秒完成】恢复正常循迹模式！ <<<")
        #                 self.in_target_stop_area2 = False
        #                 self.target_stop_start_time2 = None
        #                 self.precise_park_reached2 = False
        #                 self.precise_park_last_distance = None  # 重置距离记录
        #                 self._precise_park_started2 = False  # 重置微调标志，恢复正常循迹
        #                 self.target_stop_completed2 = True  # 标记本次进入已完成停车
        #                 # 不return，继续执行后续的正常驾驶逻辑（恢复正常巡锥桶）
        # else:
        #     # 不在第二个停止区域，重置所有停止状态（准备下次进入）
        #     # 注意：只有在精确停车未启动时才重置，避免在精确停车过程中被打断
        #     # 关键：第一圈时，如果收到-88888.0信号但不在第二停车区处理逻辑中，也要重置状态
        #     if not self._precise_park_started2 or self.circle < 2:
        #         if self.in_target_stop_area2 or self.target_stop_completed2 or self._precise_park_started2:
        #             # 第一圈或不在精确停车模式中，重置所有第二停车区状态
        #             if self.circle < 2:
        #                 # 第一圈时，确保清除所有第二停车区相关状态
        #                 pass
        #             self.in_target_stop_area2 = False
        #             self.target_stop_start_time2 = None
        #             self.precise_park_reached2 = False
        #             self.precise_park_last_distance = None  # 重置距离记录
        #             self.target_stop_completed2 = False  # 重置完成标志，允许下次进入时再次停车
        #             # 重置精确微调标志
        #             self._precise_park_started2 = False
        
        # === V7: 优先检查是否有有效目标点，退出黄线弯模式 ===
        # 注意：如果正在精确停车模式中，跳过正常的PID循迹逻辑，避免干扰精确停车（已注释）
        # 关键：第一圈时，即使_precise_park_started2为True，也不拦截正常循迹（可能是状态残留）
        # if self._precise_park_started2 and self.circle >= 2:
        #     # 正在精确停车模式中（且是第二圈），不执行后续的正常驾驶逻辑
        #     return
        
        is_valid_target = (self.route_point != () and self.relative_point != () and 
                          self.relative_point[0] > -9000 and self.relative_point[1] > -9000)
        
        # 调试信息：显示目标点状态（用于诊断为什么小车不动）
        if not is_valid_target:
            self.get_logger().warn(
                f">>> [控制节点调试] is_valid_target=False | "
                f"absolute_point={self.absolute_point}, "
                f"relative_point={self.relative_point}, "
                f"route_point={self.route_point}",
                throttle_duration_sec=1.0
            )
        
        if is_valid_target:
            # 退出黄线弯模式
            if self.in_yellow_curve_mode:
                self.get_logger().info(">>> 退出黄线弯模式 <<<")
                self.in_yellow_curve_mode = False
                self.yellow_curve_first_frame = False
            
            # --- 模式1：PID 循迹（直道、右转）---
            # 触发条件：relative_point[0] > -9000 且 relative_point[1] > -9000（正常目标点）
            if self.relative_point:
                self.get_logger().warn(
                    f">>> 正常循迹模式 | 触发条件: relative_point=({self.relative_point[0]:.2f}, {self.relative_point[1]:.2f}), "
                    f"速度档位={self.v}档, 实时速度={self.current_velocity:.2f}m/s <<<",
                    throttle_duration_sec=1.0
                )
            
            # 计算误差（车身姿态角 - 目标角度）
            error = self.angle - self.get_route_theta()
            
            # 角度误差归一化
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi
            
            # PID控制（移植自line_follow.py：使用实时速度而不是速度档位）
            steering_theta = self.pid_control(error, self.last_error, self.sum_error, self.current_velocity)
            self.last_error = error
            
            # 积分项限幅
            if self.sum_error < self.sum_err_threshold and self.sum_error > -self.sum_err_threshold:
                self.sum_error += error
            
            # 进入循迹分支，清除左转切换标志
            self.just_switched_left = False
            self.drive(self.v, steering_theta)
        
        # === V7: 黄线弯模式锁定逻辑 ===
        elif self.in_yellow_curve_mode:
            # 在黄线弯模式中，不管收到什么信号都继续执行黄线弯逻辑
            # 触发条件：in_yellow_curve_mode==True（锁定模式，直到识别到有效目标点）
            
            # 使用配置的黄线弯速度，角度增强+3度（锁定模式使用后续帧角度，达到105°）
            yellow_curve_speed = self.yellow_curve_speed
            yellow_curve_angle = self.yellow_curve_angle + 3.0  # 锁定模式也增强3度
            
            self.get_logger().warn(
                f">>> 黄线弯模式（锁定） | 触发条件: in_yellow_curve_mode==True（锁定模式）, "
                f"{yellow_curve_speed}档速度，角度{yellow_curve_angle}° <<<",
                throttle_duration_sec=1.0
            )
            
            self.drive(yellow_curve_speed, yellow_curve_angle)
        
        elif self.relative_point and self.relative_point[0] == -10000.0:
            # --- 模式3：黄线弯左转（无锥桶，-10000信号）首次进入 ---
            # 触发条件：relative_point[0] == -10000.0（扇形检测无锥桶）
            
            # 进入黄线弯模式并锁定
            self.in_yellow_curve_mode = True
            
            # 使用配置的黄线弯速度
            yellow_curve_speed = self.yellow_curve_speed
            yellow_curve_angle = self.yellow_curve_angle
            
            # V11: 首帧角度增强（+3度），后续帧角度增强（+3度达到105°）
            if not self.just_switched_left:
                # 首帧：角度增强+3度
                yellow_curve_angle += 3.0  # 首帧增强3度
                self.yellow_curve_first_frame = True
                self.sum_error = 0.0
                self.last_error = 0.0
                self.just_switched_left = True
                self.get_logger().warn(
                    f">>> 黄线弯首帧 | 触发条件: relative_point[0]==-10000.0（扇形检测无锥桶）, "
                    f"{yellow_curve_speed}档速度, 角度{yellow_curve_angle}° <<<",
                    throttle_duration_sec=1.0
                )
            else:
                # 后续帧：角度增强+3度（达到105°）
                yellow_curve_angle += 3.0  # 后续帧增强3度
                self.yellow_curve_first_frame = False
                self.get_logger().warn(
                    f">>> 黄线弯后续帧 | 触发条件: relative_point[0]==-10000.0（扇形检测无锥桶）, "
                    f"{yellow_curve_speed}档速度, 角度{yellow_curve_angle}° <<<",
                    throttle_duration_sec=1.0
                )
            
            self.drive(yellow_curve_speed, yellow_curve_angle)
        
        elif self.relative_point and self.relative_point[0] == -20000.0:
            # --- 模式2：直角左转（有锥桶，感知失败）---
            # 触发条件：relative_point[0] == -20000.0（感知失败）
            # 策略：首帧保持当前速度+65度角度，后续20档速度+60度角度
            self.get_logger().warn(f">>> 直角左转 | 触发条件: relative_point[0]==-20000.0（感知失败）<<<", throttle_duration_sec=1.0)
            
            # 重置PID状态，避免上一帧右打的惯性（D/I残留）
            if not self.just_switched_left:
                # 首帧：保持当前速度，固定65度角度
                # 注意：第一圈和第二圈使用完全相同的参数，不依赖self.circle
                self.sum_error = 0.0
                self.last_error = 0.0
                left_turn_speed = self.v  # 首帧保持当前速度（取消降速）
                # 首帧使用固定65度角度
                boosted_angle = 65  # 首帧固定65度
                self.get_logger().warn(
                    f">>> 直角弯首帧角增强 | 触发条件: relative_point[0]==-20000.0（感知失败）, "
                    f"保持速度{left_turn_speed}档, 角度{boosted_angle}° <<<",
                    throttle_duration_sec=1.0
                )
                self.drive(left_turn_speed, boosted_angle)
                self.just_switched_left = True
                return
            else:
                # 后续帧：固定20档速度，固定60度角度
                # 注意：第一圈和第二圈使用完全相同的参数，不依赖self.circle
                left_turn_speed = 20  # 常规左转（固定20档速度）
                turn_angle = 60  # 后续帧固定60度
                self.get_logger().warn(
                    f">>> 直角弯后续帧 | 触发条件: relative_point[0]==-20000.0（感知失败）, "
                    f"速度={left_turn_speed}档, 角度{turn_angle}° <<<",
                    throttle_duration_sec=1.0
                )
                self.drive(left_turn_speed, turn_angle)
        
        elif self.relative_point and self.relative_point[0] == -30000.0:
            # --- 模式4：向右修正（三等分点为空，内外道配对失败）---
            # 触发条件：relative_point[0] == -30000.0（三等分点为空，内外道配对失败或外道点被过滤）
            # 策略：保持当前速度，向右转向修正
            self.get_logger().warn(
                f">>> 向右修正模式 | 触发条件: relative_point[0]==-30000.0（三等分点为空，内外道配对失败）, "
                f"保持速度{self.v}档，向右转向{self.right_correction_angle}° <<<",
                throttle_duration_sec=1.0
            )
            
            # 重置PID状态，避免残留
            self.sum_error = 0.0
            self.last_error = 0.0
            
            self.drive(self.v, self.right_correction_angle)
        
        elif self.relative_point and self.relative_point[0] == -40000.0:
            # --- 模式5：保持当前状态（内道不足但外道点中等）---
            # 策略：如果小车已停止（例如停车后），恢复正常的循迹速度；否则保持当前状态
            if hasattr(self, 'last_twist') and self.last_twist is not None:
                # 检查小车是否已停止（速度 <= 1000，即 1500 + (-500)）
                current_speed = self.last_twist.linear.x
                if current_speed <= 1000:
                    # 小车已停止，恢复正常的循迹速度
                    self.get_logger().warn(f">>> 保持当前状态模式 (-40000) | 检测到小车已停止，恢复循迹速度{self.v}档 <<<", throttle_duration_sec=1.0)
                    # 使用最后一次非停车的转向角度，如果没有则使用直行（0）
                    self.drive(self.v, self.last_normal_theta)
                else:
                    # 小车正在运行，保持当前状态（不发送新的驱动指令）
                    self.get_logger().warn(f">>> 保持当前状态模式 (-40000) | 保持当前速度和转向角度 <<<", throttle_duration_sec=1.0)
                    pass  # 不执行任何驱动指令，保持当前状态
            else:
                # 没有历史数据，恢复正常的循迹速度
                self.get_logger().warn(f">>> 保持当前状态模式 (-40000) | 无历史数据，恢复循迹速度{self.v}档 <<<", throttle_duration_sec=1.0)
                self.drive(self.v, self.last_normal_theta)
    
    def check_xy_parking(self):
        """
        检查是否到达硬编码的XY停车位置（支持多停车点，按圈数配置）
        返回: True=需要停车, False=继续行驶
        """
        if not self.enable_xy_parking:
            return False
        
        if len(self.park_points) == 0:
            return False
        
        if self.absolute_point == ():
            return False
        
        if self.current_park_index >= len(self.park_points):
            # 所有停车点都已完成
            return False
        
        # 获取当前停车点
        current_point = self.park_points[self.current_park_index]
        
        # 检查是否匹配当前圈数
        if self.circle != current_point['circle']:
            # 不是当前圈数的停车点，跳过
            # 如果所有后续点都不是当前圈数，重置索引
            found_current_circle = False
            for i in range(self.current_park_index, len(self.park_points)):
                if self.park_points[i]['circle'] == self.circle:
                    found_current_circle = True
                    break
            
            if not found_current_circle:
                # 当前圈没有更多停车点，等待下一圈
                return False
            
            # 寻找当前圈数的下一个停车点
            while self.current_park_index < len(self.park_points):
                if self.park_points[self.current_park_index]['circle'] == self.circle:
                    break
                self.current_park_index += 1
            
            if self.current_park_index >= len(self.park_points):
                return False
            
            current_point = self.park_points[self.current_park_index]
        
        # 如果这个点已经完成，跳到下一个
        if current_point['completed']:
            self.current_park_index += 1
            if self.current_park_index >= len(self.park_points):
                return False
            current_point = self.park_points[self.current_park_index]
            # 如果新点不是当前圈数，返回False
            if current_point['circle'] != self.circle:
                return False
        
        # 计算到停车目标的距离
        current_x, current_y = self.absolute_point
        target_x = current_point['x']
        target_y = current_point['y']
        tolerance = current_point['tolerance']
        
        distance = math.sqrt(
            (current_x - target_x) ** 2 + 
            (current_y - target_y) ** 2
        )
        
        # 如果到达停车区域
        if distance <= tolerance:
            if not self.xy_park_reached:
                # 首次到达停车位置
                self.xy_park_reached = True
                self.xy_park_start_time = time.time()
                self.xy_park_completed = False
                self.get_logger().warn(
                    f">>> 【到达停车点{self.current_park_index+1}/{len(self.park_points)}】"
                    f"第{current_point['circle']}圈 | "
                    f"坐标({current_x:.3f}, {current_y:.3f}), "
                    f"目标({target_x:.3f}, {target_y:.3f}), "
                    f"距离{distance:.3f}m, 停车{current_point['duration']}秒 <<<"
                )
                return True
            else:
                # 已在停车区域内，检查是否已停车足够时间
                if not self.xy_park_completed:
                    elapsed = time.time() - self.xy_park_start_time
                    duration = current_point['duration']
                    
                    if elapsed < duration:
                        remaining = duration - elapsed
                        self.get_logger().info(
                            f">>> 停车点{self.current_park_index+1}停车中... "
                            f"剩余 {remaining:.1f}秒 <<<",
                            throttle_duration_sec=0.5
                        )
                        return True
                    else:
                        # 停车时间已到
                        self.xy_park_completed = True
                        current_point['completed'] = True
                        self.get_logger().warn(
                            f">>> 【停车点{self.current_park_index+1}完成】"
                            f"已停车{elapsed:.1f}秒，继续前往下一个点！ <<<"
                        )
                        # 准备下一个停车点
                        self.current_park_index += 1
                        self.xy_park_reached = False
                        self.xy_park_start_time = None
                        self.xy_park_completed = False
                        return False
                else:
                    # 已完成停车，等待离开区域
                    return False
        else:
            # 不在停车区域内
            if self.xy_park_reached:
                # 离开停车区域，重置状态（如果还没完成，可能是误判）
                self.xy_park_reached = False
                self.xy_park_start_time = None
            return False
    
    def run(self):
        """主运行逻辑（基于源程序）"""
        self.stop_the_car()
        self.detect_cooldown()
        
        if self.stop:
            # 停车
            self.get_logger().warn('--------Stop!!!---------')
            self.drive(-500, 0)
            return
        
        # === XY坐标停车检查（最高优先级，支持多停车点，按圈数配置）===
        if self.check_xy_parking():
            # 需要停车
            self.drive(-500, 0)
            return
        
        # === 红灯停车检查（正常跑圈时的红灯停车）===
        # 调试信息：定期打印红灯检测状态（每2秒一次，避免刷屏）
        current_time = time.time()
        if current_time - self.last_red_light_log_time >= 2.0:
            # 检查是否收到过图像检测消息
            time_since_last_msg = current_time - self.image_detection_last_time if self.image_detection_received else float('inf')
            
            if not self.image_detection_received:
                self.get_logger().warn(
                    "⚠️⚠️⚠️ 【警告】从未收到图像检测消息！请检查："
                    "\n  1. image_detector 节点是否正在运行？"
                    "\n  2. /image_detection 话题是否存在？"
                    "\n  3. 运行命令检查: ros2 topic echo /image_detection"
                )
            elif time_since_last_msg > 5.0:
                self.get_logger().warn(
                    f"⚠️⚠️⚠️ 【警告】超过{time_since_last_msg:.1f}秒未收到图像检测消息！"
                    "\n  可能 image_detector 节点已停止或话题连接断开"
                )
            else:
                self.get_logger().info(
                    f">>> 【红灯检测状态】red={self.red}, cooldown={self.cooldown}, "
                    f"stopping={self.red_light_stopping}, "
                    f"上次消息{time_since_last_msg:.1f}秒前 <<<"
                )
            self.last_red_light_log_time = current_time
        
        # === 关键修复：如果正在停车中，优先处理停车逻辑，不管红灯状态如何变化 ===
        if self.red_light_stopping:
            # 正在停车中，检查是否已停车足够时间
            elapsed_time = time.time() - self.red_light_stop_start_time
            if elapsed_time < self.red_light_stop_duration:
                # 继续停车，显示剩余时间（锁定停车状态，即使红灯消失也继续停车）
                remaining_time = self.red_light_stop_duration - elapsed_time
                self.get_logger().warn(
                    f">>> 【红灯停车中】剩余 {remaining_time:.1f}秒 <<<",
                    throttle_duration_sec=0.5
                )
                self.drive(-500, 0)
                return
            else:
                # 停车时间已到，恢复正常运行
                self.get_logger().warn(
                    f">>> 【红灯停车完成】已停车{elapsed_time:.1f}秒，继续运行！ <<<"
                )
                self.red_light_stopping = False
                self.red_light_stop_start_time = None
                self.cooldown = 300  # 设置冷却时间，避免重复触发
                # 不return，继续执行后续的正常驾驶逻辑
        
        # === 检测到红灯且未在停车中，开始停车 ===
        elif self.red and self.cooldown < 1:
            # 首次检测到红灯，开始停车
            self.get_logger().warn(
                "=================>🔴 检测到红灯！停车3.5秒...<=================="
            )
            self.red_light_stopping = True
            self.red_light_stop_start_time = time.time()
            self.drive(-500, 0)
            return
        
        # === 原有的红灯处理逻辑（用于圈数切换，仅在特定位置触发）===
        # 注意：如果需要保留圈数切换功能，可以在这里添加特定位置的判断
        # 例如：只在特定坐标范围内检测到红灯时才增加圈数
        # if self.red and self.cooldown < 1 and self.is_at_specific_location():
        #     self.circle += 1
        #     ...
        
        self.car_drive()
    
    def image_detection_callback(self, msg: Int16MultiArray):
        """图像检测回调函数 - 获取红灯、A/B、黄线检测结果"""
        self.image_detection_received = True
        self.image_detection_last_time = time.time()
        
        if len(msg.data) >= 1:
            # data格式: [红灯, A/B, 黄线, AB车库方向]
            old_red = self.red
            self.red = msg.data[0]  # 红灯检测结果：0=未检测，1=检测到
            
            # 如果红灯状态发生变化，打印日志
            if old_red != self.red:
                if self.red == 1:
                    self.get_logger().warn("🔴🔴🔴 红灯检测状态变化：检测到红灯！ 🔴🔴🔴")
                else:
                    self.get_logger().info("✅ 红灯检测状态变化：红灯消失")
            
            # 定期打印图像检测数据（每5秒一次，用于调试）
            current_time = time.time()
            if current_time - self.last_red_light_log_time >= 5.0:
                self.get_logger().info(
                    f">>> 【图像检测数据】红灯={self.red}, A/B={msg.data[1] if len(msg.data) >= 2 else 'N/A'}, "
                    f"黄线={msg.data[2] if len(msg.data) >= 3 else 'N/A'}, "
                    f"原始数据={list(msg.data)} <<<"
                )
                self.last_red_light_log_time = current_time
            
            if len(msg.data) >= 2:
                self.A_B = msg.data[1]  # A/B标识：0=未检测，1=A，2=B
            if len(msg.data) >= 3:
                self.yellow = msg.data[2]  # 黄线检测：0=未检测，1=检测到
        else:
            self.get_logger().warn("⚠️  图像检测消息数据为空！")
    
    def target_callback(self, msg: Float32MultiArray):
        """目标点回调（基于源程序getTarget函数）"""
        if not self.start:
            self.start_pub.publish(Bool(data=True))
            self.start = True
            # 第一圈开始时启动路径点记录（基于源程序：circle=1时开始记录）
            if self.navigation_manager and self.circle == 0:
                self.circle = 1  # 设置为第一圈
                self.navigation_manager.start_recording()
                self.get_logger().info(">>> 【第一圈开始】开始记录路径点 <<<")
            # 不要return，继续处理这一帧数据
        
        data = msg.data
        if len(data) >= 2:
            self.set_relative_point((data[0], data[1]))
            if self.absolute_point != ():  # 只有在有里程计数据时才进行坐标变换
                    self.tf_coordinates()
    
    def odom_callback(self, msg: Odometry):
        """里程计回调（基于源程序callBack函数）"""
        if not self.start:
            return
        
        # 提取实时速度（m/s）（移植自line_follow.py）
        self.current_velocity = msg.twist.twist.linear.x
        
        self.delete_goal_point()
        self.set_car_point_and_angle(msg)
        
        # 更新导航管理器的姿态信息（用于记录路径点时获取四元数）
        if self.navigation_manager:
            orientation = msg.pose.pose.orientation
            self.navigation_manager.update_orientation(orientation.z, orientation.w)
        
        # 第二圈导航：检查是否到达目标点并发送下一个点
        if self.enable_navigation_loop and self.navigation_manager and self.navigation_manager.is_navigating:
            if self.absolute_point != ():
                current_x, current_y = self.absolute_point
                # 检查是否到达当前目标点
                if self.navigation_manager.check_goal_reached(current_x, current_y):
                    # 到达目标点，移动到下一个
                    self.navigation_manager.move_to_next_goal()
                
                # 获取当前目标点并发布
                goal = self.navigation_manager.get_current_goal(clock=self.get_clock())
                if goal:
                    self.goal_pub.publish(goal)
        
        # 打印XY坐标（用于手推车获取停车点坐标）
        # 注意：打印的坐标就是可以直接用来设置停车区域的坐标（归零后的坐标）
        if self.print_position and self.absolute_point != ():
            current_time = time.time()
            if current_time - self.last_position_print_time >= self.position_print_interval:
                x, y = self.absolute_point  # absolute_point已经是归零后的坐标
                # 从四元数提取角度
                orientation = msg.pose.pose.orientation
                siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
                cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                yaw_deg = math.degrees(yaw)
                
                self.last_position_print_time = current_time
        
        self.run()
    
    def control_callback(self):
        """控制回调（定时器触发）"""
        # 这里可以添加额外的控制逻辑
        pass
    
    def record_navigation_point(self):
        """
        第一圈路径点记录定时器回调（基于源程序racecar_teleop_test.py的timer方法）
        每隔固定时间（record_interval）记录当前车身坐标和姿态
        """
        if not self.navigation_manager:
            return
        
        # 基于源程序：if circle != 1 or not start: return
        if self.circle != 1 or not self.start:
            return
        
        # 基于源程序：if self.route.absolutePoint != (): self.storePoint(...)
        if self.absolute_point != ():
            x, y = self.absolute_point
            self.navigation_manager.record_point(x, y)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = ControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
