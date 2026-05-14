from setuptools import setup
import os
from glob import glob

package_name = 'my_car_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davinci-mini',
    maintainer_email='user@example.com',
    description='自定义小车控制功能包',
    license='Apache-2.0',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            # 在这里注册你的可执行节点
            'lidar_follower_node = my_car_control.lidar_follower_node:main',
            'lidar_sub = my_car_control.lidar_sub:main',  # 激光雷达可视化节点
            'raw_lidar_viewer = my_car_control.raw_lidar_viewer:main',  # 原始激光雷达点云可视化节点（仅矩形滤波）
            'simple_perception_node = my_car_control.simple_perception_node:main',  # 简化感知节点（基于简单聚类和连线）
            'perception_node = my_car_control.perception_node:main',  # 纯感知节点（基于NCSC2024源程序）
            'perception_node_simple_cluster = my_car_control.perception_node_simple_cluster:main',  # 纯感知节点（使用简单聚类替代KMeans）
            'control_node = my_car_control.control_node:main',  # 控制节点（基于NCSC2024源程序）
            'image_detector = my_car_control.image_detector:main',  # 图像检测节点（红灯、A/B标识、黄线）
            'vision_control_test = my_car_control.vision_control_test:main',  # 视觉控制测试节点（独立调试）
            'vision_debug_viewer = my_car_control.vision_debug_viewer:main',  # 视觉调试查看器（实时显示画面+调阈值）
            'map_viewer = my_car_control.map_viewer:main',  # 简易弹窗地图查看器
            'ab_detector_debug = my_car_control.ab_detector_debug:main',  # AB标志识别调试工具（实时显示处理步骤）
            'yolo_ab_detector = my_car_control.yolo_ab_detector:main',  # YOLO AB标志识别节点（使用训练好的YOLO模型）
            'debug_yellow_hsv = my_car_control.debug_yellow_hsv:main',  # 黄线检测HSV调试工具（实时调节参数）
            'debug_red_hsv = my_car_control.debug_red_hsv:main',  # 红灯检测HSV调试工具（实时调节参数）
            'capture_ab_template = my_car_control.capture_ab_template:main',  # AB标志模板拍照工具
            'camera_control_node = my_car_control.camera_control_node:main',  # 摄像头参数实时调整节点
            'npu_ab_detector_node = my_car_control.npu_ab_detector_node:main',  # NPU AB标志检测节点（使用昇腾NPU）
            'npu_ab_detector_node_launcher = my_car_control.npu_ab_detector_node_launcher:main',  # NPU节点启动器（自动选择Python版本）
            'navigation_test_node = my_car_control.navigation_test_node:main',  # 导航测试节点（单独调试导航功能）
            'send_nav_command = my_car_control.scripts.send_nav_command:main',  # 发送导航命令的交互式工具
            'cmd_vel_relay_node = my_car_control.cmd_vel_relay_node:main',  # Nav2速度命令转发节点（已废弃，使用cmd_vel_converter_node）
            'cmd_vel_converter_node = my_car_control.cmd_vel_converter_node:main',  # Nav2速度命令转换节点（m/s转PWM）
            'cmd_vel_pid_fusion_node = my_car_control.cmd_vel_pid_fusion_node:main',  # Nav2速度命令PID融合节点（参考23张原的nav.py）
            'goal_pose_to_action_bridge = my_car_control.goal_pose_to_action_bridge:main',  # 目标点话题到Action桥接节点
            'visualize_navigation = my_car_control.visualize_navigation:main',  # 导航路径可视化节点（实时显示路径规划、记录点、坐标系）
            # bag录制和回放工具
            'record_bag = my_car_control.scripts.record_bag:main',  # 录制bag文件
            'play_bag = my_car_control.scripts.play_bag:main',  # 回放bag文件
            # === 2025 改造：被动观察 + 离线调试基础设施 ===
            'track_memory_recorder = my_car_control.track_memory_recorder:main',  # 被动记录每帧状态到CSV（不影响控制）
            'track_memory_viewer = my_car_control.track_memory_viewer:main',  # 离线重画CSV（迁移自2022 debug方法论）
        ],
    },
)

