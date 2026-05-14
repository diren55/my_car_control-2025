#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标点话题到Action桥接节点
将 /goal_pose 话题转换为 Nav2 的 /navigate_to_pose Action 调用
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time


class GoalPoseToActionBridge(Node):
    """将 /goal_pose 话题转换为 Nav2 Action 调用"""
    
    def __init__(self):
        super().__init__('goal_pose_to_action_bridge')
        
        # 订阅目标点话题
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Nav2 Action 客户端
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # 当前目标句柄
        self.current_goal_handle = None
        
        # 等待Action服务器就绪的定时器
        self.wait_timer = self.create_timer(2.0, self.check_action_server)
        self.server_ready = False
        self.retry_count = 0
        self.max_retries = 60  # 最多重试60次（120秒，给BT Navigator更多启动时间）
        
        self.get_logger().info("目标点桥接节点已启动")
        self.get_logger().info("等待 Nav2 Action 服务器就绪...")
        self.get_logger().info("提示：如果长时间无法连接，请检查：")
        self.get_logger().info("  1. Nav2 是否已启动: ros2 node list | grep bt_navigator")
        self.get_logger().info("  2. Action 服务是否存在: ros2 action list | grep navigate")
        self.get_logger().info("  3. BT Navigator 是否已激活（可能需要等待 Nav2 完全启动）")
        # 立即尝试一次连接
        self.check_action_server()
    
    def check_action_server(self):
        """检查Action服务器是否就绪"""
        if self.server_ready:
            return
        
        if self.retry_count >= self.max_retries:
            self.get_logger().error(f"❌ 已重试 {self.max_retries} 次（约 {self.max_retries * 2} 秒），Nav2 Action 服务器仍未就绪")
            self.get_logger().error("请检查：")
            self.get_logger().error("  1. Nav2 是否已完全启动（可能需要等待 10-30 秒）")
            self.get_logger().error("  2. 运行: ros2 action info /navigate_to_pose")
            self.get_logger().error("  3. 如果 Action servers: 0，说明 BT Navigator 未激活")
            self.get_logger().error("  4. 可以尝试手动激活: ros2 lifecycle set /bt_navigator configure && ros2 lifecycle set /bt_navigator activate")
            self.wait_timer.cancel()
            return
        
        self.retry_count += 1
        if self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.server_ready = True
            self.wait_timer.cancel()
            self.get_logger().info(f"✅ Nav2 Action 服务器已就绪（重试 {self.retry_count} 次后）")
        else:
            if self.retry_count % 5 == 0:  # 每5次重试打印一次
                self.get_logger().warn(f"Nav2 Action 服务器未就绪，已重试 {self.retry_count} 次...")
    
    def goal_callback(self, msg: PoseStamped):
        """目标点话题回调"""
        if not self.server_ready:
            # 再次尝试连接（给更多时间）
            self.get_logger().warn("Nav2 Action 服务器未就绪，尝试连接...")
            if self.nav_action_client.wait_for_server(timeout_sec=5.0):
                self.server_ready = True
                self.get_logger().info("✅ Nav2 Action 服务器已就绪（在回调中连接成功）")
            else:
                self.get_logger().error("❌ 无法连接到 Nav2 Action 服务器")
                self.get_logger().error("请确保 Nav2 已启动且 BT Navigator 已激活")
                self.get_logger().error("可以运行: ros2 action list | grep navigate")
                return
        
        # 取消之前的goal（如果有）
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal()
                self.get_logger().info("已取消之前的导航目标")
            except Exception as e:
                self.get_logger().warn(f"取消之前目标时出错: {e}")
        
        # 创建Action目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        # 发送Action目标
        self.get_logger().info(
            f"📤 转发目标点到 Nav2: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})"
        )
        
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Action目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ 导航目标被 Nav2 拒绝")
            # 尝试获取拒绝原因
            try:
                status = goal_handle.status
                self.get_logger().warn(f"拒绝状态码: {status}")
            except:
                pass
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info("✅ 导航目标已被 Nav2 接受，开始导航")
        
        # 获取结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Action目标结果回调"""
        try:
            response = future.result()
            status = response.status
            result = response.result
            
            # 检查导航状态
            if status == 4:  # SUCCEEDED
                self.get_logger().info(f"✅ 导航完成（状态: SUCCEEDED）")
            elif status == 2:  # CANCELED
                self.get_logger().warn(f"⚠️  导航被取消（状态: CANCELED）")
            elif status == 3:  # ABORTED
                self.get_logger().error(f"❌ 导航失败（状态: ABORTED）")
            else:
                self.get_logger().warn(f"⚠️  导航完成，状态码: {status}")
        except Exception as e:
            self.get_logger().warn(f"获取导航结果时出错: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = GoalPoseToActionBridge()
    
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

