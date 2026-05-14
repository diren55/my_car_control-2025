from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # 参数
    scan_topic = DeclareLaunchArgument('scan_topic', default_value='/scan')

    # 双TF
    tf_dual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('my_car_control'), '/launch/tf_static_dual.launch.py']
        )
    )

    # Cartographer 2D
    carto_2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('cartographer_bringup'), '/launch/cartographer_2d.launch.py'
        ]),
        launch_arguments={
            'scan_topic': LaunchConfiguration('scan_topic'),
        }.items()
    )

    # 占据栅格发布节点（若未发布 /map，可启用）
    occ_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05, 'publish_period_sec': 1.0}],
    )

    # 弹窗地图查看
    viewer = Node(
        package='my_car_control',
        executable='map_viewer',
        name='map_viewer',
        output='screen',
        parameters=[{'map_topic': '/map'}],
    )

    return LaunchDescription([
        scan_topic,
        tf_dual,
        carto_2d,
        occ_grid,
        viewer,
    ])


