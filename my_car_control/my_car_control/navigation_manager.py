#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航管理器 - 基于源程序racecar_teleop_test.py的Navigator类
功能：
1. 第一圈：每隔固定时间记录车身坐标和姿态作为导航目标点
2. 第二圈：从CSV文件读取路径点，循环发送给导航系统
"""

import csv
import os
import math
import time
from typing import List, Tuple, Optional
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class NavigationManager:
    """
    导航管理器类
    基于源程序racecar_teleop_test.py的Navigator类实现
    """
    
    def __init__(self, csv_file_path: str, logger=None, record_interval: float = 0.5, min_distance: float = 1.5):
        """
        初始化导航管理器
        
        Args:
            csv_file_path: CSV文件保存路径（用于存储和读取路径点）
            logger: ROS2 logger对象（可选，如果不提供则使用标准logging）
            record_interval: 第一圈记录路径点的时间间隔（秒），默认0.5秒
            min_distance: 第二圈到达目标点的最小距离阈值（米），默认0.8米
        """
        self.csv_file_path = csv_file_path
        self.record_interval = record_interval
        self.min_distance = min_distance
        self.logger = logger
        
        # 存储路径点的列表（第一圈记录时使用）
        # 格式: [[x, y, z, w], ...] 其中z, w是四元数的z和w分量
        self.navigation_points = []
        
        # 当前读取的路径点列表（第二圈导航时使用）
        self.loaded_points = []
        
        # 当前目标点索引（第二圈导航时使用）
        self.current_goal_index = 0
        
        # 当前姿态（用于记录路径点时获取四元数）
        self.current_orientation_z = 0.0
        self.current_orientation_w = 1.0
        
        # 状态标志
        self.is_recording = False  # 是否正在记录路径点
        self.is_navigating = False  # 是否正在导航（第二圈）
        self.points_saved = False  # 路径点是否已保存
        
        # 确保CSV文件目录存在
        csv_dir = os.path.dirname(csv_file_path)
        if csv_dir and not os.path.exists(csv_dir):
            os.makedirs(csv_dir, exist_ok=True)
    
    def start_recording(self):
        """开始记录路径点（第一圈开始时调用）"""
        self.is_recording = True
        self.navigation_points = []
        self.points_saved = False
        if self.logger:
            self.logger.info("开始记录路径点（第一圈）")
    
    def stop_recording(self):
        """停止记录路径点（第一圈结束时调用）"""
        self.is_recording = False
        if self.logger:
            self.logger.info(f"停止记录路径点，共记录 {len(self.navigation_points)} 个点")
    
    def update_orientation(self, orientation_z: float, orientation_w: float):
        """
        更新当前姿态（四元数）
        在里程计回调中调用，用于记录路径点时获取姿态
        
        Args:
            orientation_z: 四元数z分量
            orientation_w: 四元数w分量
        """
        self.current_orientation_z = orientation_z
        self.current_orientation_w = orientation_w
    
    def record_point(self, x: float, y: float):
        """
        记录一个路径点（第一圈定时调用）
        基于源程序racecar_teleop_test.py的storePoint方法
        
        Args:
            x: 当前X坐标
            y: 当前Y坐标
        """
        if not self.is_recording:
            return
        
        # 存储路径点：x, y, z, w（四元数）
        # 基于源程序：self.navigationMsgs.append([x,y,z,w])
        if self.current_orientation_z is not None and self.current_orientation_w is not None:
            point = [x, y, self.current_orientation_z, self.current_orientation_w]
            self.navigation_points.append(point)
            if self.logger:
                self.logger.debug(f"记录路径点 {len(self.navigation_points)}: ({x:.3f}, {y:.3f}, z={self.current_orientation_z:.3f}, w={self.current_orientation_w:.3f})")
    
    def save_points_to_csv(self):
        """
        保存路径点到CSV文件（第一圈结束时调用）
        基于源程序racecar_teleop_test.py的savePointsToCsv方法
        """
        if len(self.navigation_points) == 0:
            if self.logger:
                self.logger.warn("没有路径点可保存")
            return
        
        try:
            with open(self.csv_file_path, 'w', encoding='utf-8', newline='') as f:
                writer = csv.writer(f)
                for point in self.navigation_points:
                    writer.writerow(point)
            
            self.points_saved = True
            if self.logger:
                self.logger.info(f"成功保存 {len(self.navigation_points)} 个路径点到: {self.csv_file_path}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"保存路径点失败: {str(e)}")
    
    def load_points_from_csv(self) -> bool:
        """
        从CSV文件加载路径点（第二圈开始时调用）
        基于源程序goal_loop.py的读取逻辑
        
        Returns:
            bool: 是否成功加载
        """
        if not os.path.exists(self.csv_file_path):
            if self.logger:
                self.logger.error(f"CSV文件不存在: {self.csv_file_path}")
            return False
        
        try:
            self.loaded_points = []
            with open(self.csv_file_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 4:
                        # 读取: x, y, z, w
                        point = [float(row[0]), float(row[1]), float(row[2]), float(row[3])]
                        self.loaded_points.append(point)
            
            if len(self.loaded_points) == 0:
                if self.logger:
                    self.logger.error("CSV文件中没有路径点")
                return False
            
            self.current_goal_index = 0
            self.is_navigating = True
            if self.logger:
                self.logger.info(f"成功加载 {len(self.loaded_points)} 个路径点")
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"加载路径点失败: {str(e)}")
            return False
    
    def get_current_goal(self, clock=None) -> Optional[PoseStamped]:
        """
        获取当前目标点（第二圈导航时调用）
        基于源程序goal_loop.py的发布逻辑
        
        Args:
            clock: ROS2 Clock对象（用于设置时间戳），如果为None则不设置时间戳
        
        Returns:
            PoseStamped: 当前目标点，如果没有则返回None
        """
        if not self.is_navigating or len(self.loaded_points) == 0:
            return None
        
        if self.current_goal_index >= len(self.loaded_points):
            # 所有点已完成，循环回到第一个点
            self.current_goal_index = 0
        
        point = self.loaded_points[self.current_goal_index]
        goal = PoseStamped()
        goal.header.frame_id = 'map'  # 使用 'map' 框架（全局地图）
        
        # 必须设置有效的时间戳，否则 Nav2 可能拒绝目标
        if clock:
            goal.header.stamp = clock.now().to_msg()
        else:
            # 如果没有提供 clock，使用当前时间
            import rclpy
            from builtin_interfaces.msg import Time
            now = rclpy.clock.Clock().now()
            goal.header.stamp = now.to_msg()
        
        goal.pose.position.x = float(point[0])
        goal.pose.position.y = float(point[1])
        goal.pose.position.z = 0.0
        
        # 直接使用CSV文件中的姿态（z, w），与源程序保持一致
        if len(point) >= 4:
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = float(point[2])
            goal.pose.orientation.w = float(point[3])
        else:
            # 如果没有姿态信息，使用默认朝向（0度）
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
        
        return goal
    
    def check_goal_reached(self, current_x: float, current_y: float) -> bool:
        """
        检查是否到达当前目标点（第二圈导航时调用）
        基于源程序goal_loop.py的statusCB方法
        
        注意：源程序检查的是上一个已发送的目标点（goalId-1），而不是当前目标点
        
        Args:
            current_x: 当前X坐标
            current_y: 当前Y坐标
            
        Returns:
            bool: 是否到达目标点
        """
        if not self.is_navigating or len(self.loaded_points) == 0:
            return False
        
        # 源程序逻辑：检查上一个已发送的目标点（goalId-1）
        # self.gx = self.goalListX[self.goalId-1] if(self.goalId != 0) else self.goalListX[self.goalId]
        # self.gy = self.goalListY[self.goalId-1] if(self.goalId != 0) else self.goalListY[self.goalId]
        # 
        # 源程序中，goalId 是下一个要发送的目标点索引
        # 当 goalId = 1 时，检查 goalId-1 = 0 的目标点（已发送的目标点0）
        # 当 goalId = 0 时，检查 goalId = 0 的目标点（第一个目标点）
        #
        # 在我们的实现中，current_goal_index 是当前要发送的目标点索引
        # 当发送目标点后，current_goal_index 会加1，所以：
        # - 如果 current_goal_index = 0，说明还没发送任何目标点，检查索引0
        # - 如果 current_goal_index > 0，说明已发送了 current_goal_index-1 的目标点，检查它
        if self.current_goal_index == 0:
            # 还没发送任何目标点，检查第一个目标点（索引0）
            goal_idx = 0
        else:
            # 已发送了 current_goal_index-1 的目标点，检查它
            goal_idx = self.current_goal_index - 1
        
        if goal_idx >= len(self.loaded_points):
            return False
        
        # 获取上一个已发送的目标点
        goal_point = self.loaded_points[goal_idx]
        goal_x = goal_point[0]
        goal_y = goal_point[1]
        
        # 计算距离（基于源程序的distance方法）
        distance = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
        
        # 添加调试信息（每10次检查打印一次）
        if not hasattr(self, '_check_counter'):
            self._check_counter = 0
        self._check_counter += 1
        
        if self._check_counter % 10 == 0 and self.logger:
            self.logger.info(
                f"【目标点检查】检查目标点 {goal_idx+1} ({goal_x:.3f}, {goal_y:.3f}), "
                f"当前位置 ({current_x:.3f}, {current_y:.3f}), "
                f"距离: {distance:.3f} 米, "
                f"current_goal_index: {self.current_goal_index}"
            )
        
        # 基于源程序：if self.dist < self.MIN_DISTANCE and self.flag == 1
        # 但是，如果目标点距离太近（小于0.1米），我们需要等待一段时间，
        # 确保小车真的移动了，而不是立即跳到下一个目标点
        if distance < self.min_distance:
            # 如果距离太近（小于0.1米），需要等待至少2秒才认为到达
            # 这样可以避免因为位置没有更新而立即跳到下一个目标点
            if distance < 0.1:
                import time
                if not hasattr(self, '_close_goal_start_time'):
                    self._close_goal_start_time = time.time()
                    if self.logger:
                        self.logger.info(f"目标点 {goal_idx+1} 距离太近 ({distance:.3f} 米)，开始计时...")
                    return False
                elif time.time() - self._close_goal_start_time < 2.0:
                    if self.logger and self._check_counter % 10 == 0:
                        elapsed = time.time() - self._close_goal_start_time
                        self.logger.info(f"目标点 {goal_idx+1} 等待中... ({elapsed:.1f}/2.0 秒)")
                    return False
                else:
                    # 等待时间已到，清除计时器
                    elapsed = time.time() - self._close_goal_start_time
                    if self.logger:
                        self.logger.info(f"目标点 {goal_idx+1} 等待时间已到 ({elapsed:.1f} 秒)，认为到达")
                    delattr(self, '_close_goal_start_time')
                    return True
            else:
                # 距离在0.1米到min_distance之间，直接认为到达
                if hasattr(self, '_close_goal_start_time'):
                    delattr(self, '_close_goal_start_time')
                if self.logger:
                    self.logger.info(f"✅ 到达目标点 {goal_idx+1}，距离: {distance:.3f} 米")
                return True
        
        # 距离大于min_distance，清除计时器（如果存在）
        if hasattr(self, '_close_goal_start_time'):
            delattr(self, '_close_goal_start_time')
        
        return False
    
    def move_to_next_goal(self):
        """
        移动到下一个目标点（到达当前目标点后调用）
        基于源程序goal_loop.py的逻辑
        """
        if not self.is_navigating or len(self.loaded_points) == 0:
            return
        
        self.current_goal_index += 1
        
        # 如果所有点都完成，循环回到第一个点
        if self.current_goal_index >= len(self.loaded_points):
            self.current_goal_index = 0
            if self.logger:
                self.logger.info("所有路径点已完成，循环回到第一个点")
        else:
            if self.logger:
                self.logger.info(f"移动到下一个目标点: {self.current_goal_index + 1}/{len(self.loaded_points)}")
    
    def start_navigation(self):
        """
        开始导航（第二圈开始时调用）
        加载路径点并开始循环发送
        """
        if self.load_points_from_csv():
            if self.logger:
                self.logger.info("开始第二圈导航")
        else:
            if self.logger:
                self.logger.error("无法开始导航：路径点加载失败")
    
    def stop_navigation(self):
        """停止导航"""
        self.is_navigating = False
        if self.logger:
            self.logger.info("停止导航")
    
    def get_points_count(self) -> int:
        """获取已记录的路径点数量"""
        return len(self.navigation_points)
    
    def get_loaded_points_count(self) -> int:
        """获取已加载的路径点数量"""
        return len(self.loaded_points)
    
    def get_current_goal_index(self) -> int:
        """获取当前目标点索引"""
        return self.current_goal_index

