from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # 参数：雷达帧与父帧、外参、scan 话题
    parent_a = DeclareLaunchArgument('parent_a', default_value='base_link')
    child_a = DeclareLaunchArgument('child_a', default_value='base_footprint')
    ax = DeclareLaunchArgument('ax', default_value='0.0')
    ay = DeclareLaunchArgument('ay', default_value='0.0')
    az = DeclareLaunchArgument('az', default_value='0.0')
    aroll = DeclareLaunchArgument('aroll', default_value='0.0')
    apitch = DeclareLaunchArgument('apitch', default_value='0.0')
    ayaw = DeclareLaunchArgument('ayaw', default_value='0.0')

    parent_b = DeclareLaunchArgument('parent_b', default_value='base_footprint')
    child_b = DeclareLaunchArgument('child_b', default_value='laser_link')
    bx = DeclareLaunchArgument('bx', default_value='0.07')
    by = DeclareLaunchArgument('by', default_value='0.0')
    bz = DeclareLaunchArgument('bz', default_value='0.0')
    broll = DeclareLaunchArgument('broll', default_value='0.0')
    bpitch = DeclareLaunchArgument('bpitch', default_value='0.0')
    byaw = DeclareLaunchArgument('byaw', default_value='0.0')

    scan_topic = DeclareLaunchArgument('scan_topic', default_value='/scan')

    # 引用同包的双TF launch
    tf_dual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('my_car_control'), '/launch/tf_static_dual.launch.py']
        ),
        launch_arguments={
            'parent_a': LaunchConfiguration('parent_a'),
            'child_a': LaunchConfiguration('child_a'),
            'ax': LaunchConfiguration('ax'),
            'ay': LaunchConfiguration('ay'),
            'az': LaunchConfiguration('az'),
            'aroll': LaunchConfiguration('aroll'),
            'apitch': LaunchConfiguration('apitch'),
            'ayaw': LaunchConfiguration('ayaw'),

            'parent_b': LaunchConfiguration('parent_b'),
            'child_b': LaunchConfiguration('child_b'),
            'bx': LaunchConfiguration('bx'),
            'by': LaunchConfiguration('by'),
            'bz': LaunchConfiguration('bz'),
            'broll': LaunchConfiguration('broll'),
            'bpitch': LaunchConfiguration('bpitch'),
            'byaw': LaunchConfiguration('byaw'),
        }.items(),
    )

    # Cartographer 2D node（使用默认 bringup 的 2D 启动逻辑，透传 scan_topic）
    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('cartographer_bringup'),
            '/launch/cartographer_2d.launch.py',
        ]),
        launch_arguments={
            'scan_topic': LaunchConfiguration('scan_topic'),
        }.items(),
    )

    return LaunchDescription([
        parent_a, child_a, ax, ay, az, aroll, apitch, ayaw,
        parent_b, child_b, bx, by, bz, broll, bpitch, byaw,
        scan_topic,
        tf_dual_launch,
        cartographer_node,
    ])


