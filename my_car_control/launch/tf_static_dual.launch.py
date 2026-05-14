from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 第一条：base_link -> base_footprint（默认零外参，可按需修改）
    parent_a_arg = DeclareLaunchArgument('parent_a', default_value='base_link')
    child_a_arg = DeclareLaunchArgument('child_a', default_value='base_footprint')
    ax_arg = DeclareLaunchArgument('ax', default_value='0.0')
    ay_arg = DeclareLaunchArgument('ay', default_value='0.0')
    az_arg = DeclareLaunchArgument('az', default_value='0.0')
    aroll_arg = DeclareLaunchArgument('aroll', default_value='0.0')
    apitch_arg = DeclareLaunchArgument('apitch', default_value='0.0')
    ayaw_arg = DeclareLaunchArgument('ayaw', default_value='0.0')

    # 第二条：base_footprint -> laser_link（与你的雷达外参保持一致，默认给出示例）
    parent_b_arg = DeclareLaunchArgument('parent_b', default_value='base_footprint')
    child_b_arg = DeclareLaunchArgument('child_b', default_value='laser_link')
    bx_arg = DeclareLaunchArgument('bx', default_value='0.07')
    by_arg = DeclareLaunchArgument('by', default_value='0.0')
    bz_arg = DeclareLaunchArgument('bz', default_value='0.0')
    broll_arg = DeclareLaunchArgument('broll', default_value='0.0')
    bpitch_arg = DeclareLaunchArgument('bpitch', default_value='0.0')
    byaw_arg = DeclareLaunchArgument('byaw', default_value='0.0')

    static_tf_a = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_base_footprint',
        output='screen',
        arguments=[
            LaunchConfiguration('ax'),
            LaunchConfiguration('ay'),
            LaunchConfiguration('az'),
            LaunchConfiguration('aroll'),
            LaunchConfiguration('apitch'),
            LaunchConfiguration('ayaw'),
            LaunchConfiguration('parent_a'),
            LaunchConfiguration('child_a'),
        ],
    )

    static_tf_b = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_laser',
        output='screen',
        arguments=[
            LaunchConfiguration('bx'),
            LaunchConfiguration('by'),
            LaunchConfiguration('bz'),
            LaunchConfiguration('broll'),
            LaunchConfiguration('bpitch'),
            LaunchConfiguration('byaw'),
            LaunchConfiguration('parent_b'),
            LaunchConfiguration('child_b'),
        ],
    )

    return LaunchDescription([
        parent_a_arg, child_a_arg, ax_arg, ay_arg, az_arg, aroll_arg, apitch_arg, ayaw_arg,
        parent_b_arg, child_b_arg, bx_arg, by_arg, bz_arg, broll_arg, bpitch_arg, byaw_arg,
        static_tf_a,
        static_tf_b,
    ])


