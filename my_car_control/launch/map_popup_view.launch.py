from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 可选：如果没发布 /map，可一并启动 occupancy_grid_node
    enable_occupancy = DeclareLaunchArgument('start_occupancy', default_value='true')
    resolution = DeclareLaunchArgument('resolution', default_value='0.05')
    period = DeclareLaunchArgument('publish_period_sec', default_value='1.0')

    occupancy_node = Node(
        condition=None,
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='carto_occupancy_grid',
        output='screen',
        parameters=[
            {'resolution': LaunchConfiguration('resolution')},
            {'publish_period_sec': LaunchConfiguration('publish_period_sec')},
        ],
    )

    viewer_node = Node(
        package='my_car_control',
        executable='map_viewer',
        name='map_viewer',
        output='screen',
    )

    return LaunchDescription([
        enable_occupancy, resolution, period,
        occupancy_node,
        viewer_node,
    ])


