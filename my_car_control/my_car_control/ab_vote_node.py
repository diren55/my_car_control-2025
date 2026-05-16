#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A/B 牌投票 + combo 锁定后处理节点
================================================================================
迁移自 2022 国一代码（NCSC2024SDACY0Y）main.py 的 A/B 识别后处理逻辑。

设计思路（22 年 main.py 原文逻辑的 Python 翻译）：
  if A_B == 0:                          # 当前帧没识别到
      combo -= 1
      if combo == 0:                    # 锁定期已过
          confidence = [0, 0]           # 清空累积投票
          output = 0
      else:
          output = last_locked          # 锁定期内继续报上次结果
  else:                                  # 当前帧识别到 A 或 B
      combo = combo_max                  # 看到一次就锁 combo_max 帧
      confidence[A_B - 1] += 1            # 累积投票
      output = argmax(confidence)         # 取累积票数最多者

为什么有用：
  - 模板匹配偶尔会闪现错误（光照/角度问题）
  - 累积投票让"一致看到 N 次"才信，避免单帧错检
  - combo 锁定让"短暂看不见"时不立刻清空，避免突发抖动

为什么用独立节点而不是改 image_detector.py：
  - image_detector.py 1550 行，改动风险大
  - 独立节点可以一键开关（launch 文件 remapping 切换）
  - 不启用时完全无侵入，启用时是纯增加
  - control_node.py 一行不改

================================================================================
话题接口
================================================================================
订阅: /image_detection_raw  (std_msgs/Int16MultiArray)
       来自 image_detector，data 字段同 /image_detection 协议
       通过 launch remapping 把 image_detector 的输出改名为 _raw

发布: /image_detection       (std_msgs/Int16MultiArray)
       control_node 订阅这个话题（与原始协议同名）
       data 字段：
         data[0] = 红灯 (透传)
         data[1] = A/B (经过投票+锁定后的结果)
         data[2] = 黄线 (透传)
         data[3] = AB 方向 (透传)
         data[4] = 蓝色锥桶 (透传)

================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


class ABVoteNode(Node):
    """A/B 牌投票 + combo 锁定后处理节点"""

    def __init__(self):
        super().__init__('ab_vote_node')

        # ---- 参数 ----
        self.declare_parameter('combo_max', 10)
        # 看到一次锁定多少帧。值越大越"粘"，错检概率越低但响应也越迟钝。
        # 22 年原值是 10。模板匹配频率约 10-30Hz，10 帧 = 0.3-1 秒。

        self.declare_parameter('require_min_votes', 0)
        # 报告结果前要求至少多少张同方向投票。0 = 立刻报告（与 22 年原版一致）。
        # 设为 2-3 可让首次报告更稳健，代价是首次识别会延迟。

        self.declare_parameter('enable_logging', True)
        # 是否记录投票切换日志（方便上车后调参时看效果）

        self.combo_max = int(self.get_parameter('combo_max').value)
        self.require_min_votes = int(self.get_parameter('require_min_votes').value)
        self.enable_logging = bool(self.get_parameter('enable_logging').value)

        # ---- 内部状态 ----
        self.combo = 0                    # 剩余锁定帧数
        self.confidence = [0, 0]          # [A 票数, B 票数]
        self.last_output = 0              # 上一帧最终输出（用于检测切换并打日志）

        # ---- 话题 ----
        self.sub = self.create_subscription(
            Int16MultiArray, '/image_detection_raw', self.callback, 10)
        self.pub = self.create_publisher(
            Int16MultiArray, '/image_detection', 10)

        # 启动日志
        self.get_logger().info(
            f'[AB_VOTE] 启动: combo_max={self.combo_max}, '
            f'require_min_votes={self.require_min_votes}'
        )
        self.get_logger().info(
            '[AB_VOTE] 订阅 /image_detection_raw, 发布 /image_detection'
        )

    def apply_vote(self, raw_ab):
        """
        核心算法：22 年 main.py 的投票+combo 锁定逻辑

        参数:
            raw_ab: 当前帧 image_detector 的 A/B 检测结果
                    0 = 未检测, 1 = A, 2 = B

        返回:
            经过投票+锁定后的 A/B 结果（0 / 1 / 2）
        """
        if raw_ab == 0:
            # 当前帧没识别到
            self.combo -= 1
            if self.combo <= 0:
                # 锁定期结束，清空状态
                self.combo = 0
                self.confidence = [0, 0]
                return 0
            else:
                # 锁定期内，继续报上次结果
                return self.last_output if self.last_output != 0 else 0

        elif raw_ab in (1, 2):
            # 当前帧识别到 A 或 B
            self.combo = self.combo_max
            self.confidence[raw_ab - 1] += 1

            # 检查最低投票数门槛
            total_votes = self.confidence[0] + self.confidence[1]
            if total_votes < self.require_min_votes:
                # 票数不够，先不报告
                return 0

            # 取累积票数最多者
            if self.confidence[0] > self.confidence[1]:
                return 1
            elif self.confidence[1] > self.confidence[0]:
                return 2
            else:
                # 平票时按当前帧
                return raw_ab
        else:
            # raw_ab 异常值，不动
            return self.last_output

    def callback(self, msg):
        """订阅回调：拷贝 msg、替换 data[1]、重新发布"""
        # 容错：data 长度可能 < 5（旧版 image_detector 只发 4 字段）
        if len(msg.data) < 2:
            # 数据异常，原样转发
            self.pub.publish(msg)
            return

        raw_ab = int(msg.data[1])
        voted_ab = self.apply_vote(raw_ab)

        # 切换日志
        if self.enable_logging and voted_ab != self.last_output:
            label_map = {0: 'NONE', 1: 'A', 2: 'B'}
            self.get_logger().info(
                f'[AB_VOTE] {label_map.get(self.last_output, "?")} → '
                f'{label_map.get(voted_ab, "?")} '
                f'(raw={raw_ab}, conf={self.confidence}, combo={self.combo})'
            )
        self.last_output = voted_ab

        # 重新发布（其它字段透传）
        new_msg = Int16MultiArray()
        new_msg.data = list(msg.data)  # 拷贝
        new_msg.data[1] = voted_ab
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ABVoteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
