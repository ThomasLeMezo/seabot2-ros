import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_observer = os.path.join(
        get_package_share_directory('seabot2'),
        'config',  # Directory where yaml are
        'observer.yaml'  # Name of the file
    )
    config_physics = os.path.join(
        get_package_share_directory('seabot2'),
        'config',  # Directory where yaml are
        'physics.yaml'  # Name of the file
    )

    depth_filter_node = Node(
        package='seabot2_depth_filter',
        executable='depth_pose_node',
        namespace='observer',
        name='depth_pose_node',
        parameters=[config_observer, config_physics]
    )

    internal_sensor_filter = Node(
        package='seabot2_internal_sensor_filter',
        executable='filter_internal_sensor_node',
        namespace='observer',
        name='filter_internal_sensor_node',
        parameters=[config_observer]
    )

    kalmann_node = Node(
        package='seabot2_kalmann',
        executable='kalmann_node',
        namespace='observer',
        name='kalmann_node',
        parameters=[config_observer]
    )

    lambert_node = Node(
        package='seabot2_lambert',
        executable='lambert_node',
        namespace='observer',
        name='lambert_node',
        parameters=[config_observer]
    )

    power_filter_node = Node(
        package='seabot2_power_filter',
        executable='filter_power_node',
        namespace='observer',
        name='filter_power_node',
        parameters=[config_observer]
    )

    return LaunchDescription([
        depth_filter_node,
        internal_sensor_filter,
        kalmann_node,
        lambert_node,
        power_filter_node
    ])
