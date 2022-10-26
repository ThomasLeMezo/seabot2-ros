import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    home_path = os.path.expanduser('~')
    parameters_file_list = []

    config_observer = os.path.join(
        home_path,
        'config/default/',  # Directory where yaml are
        'observer.yaml'  # Name of the file
    )
    if os.path.exists(config_observer):
        parameters_file_list.append(config_observer)
        
    config_physics = os.path.join(
        get_package_share_directory('seabot2'),
        'config/default/',  # Directory where yaml are
        'physics.yaml'  # Name of the file
    )
    if os.path.exists(config_physics):
        parameters_file_list.append(config_physics)

    depth_filter_node = Node(
        package='seabot2_depth_filter',
        executable='depth_pose_node',
        namespace='observer',
        name='depth_pose_node',
        parameters=parameters_file_list,
        respawn=True
    )

    internal_sensor_filter = Node(
        package='seabot2_internal_sensor_filter',
        executable='filter_internal_sensor_node',
        namespace='observer',
        name='filter_internal_sensor_node',
        parameters=parameters_file_list,
        respawn=True
    )

    temperature_filter = Node(
        package='seabot2_temperature_filter',
        executable='filter_temperature_node',
        namespace='observer',
        name='filter_temperature_node',
        parameters=parameters_file_list,
        respawn=True
    )

    kalman_node = Node(
        package='seabot2_kalman',
        executable='kalman_node',
        namespace='observer',
        name='kalman_node',
        parameters=parameters_file_list,
        respawn=True
    )

    lambert_node = Node(
        package='seabot2_lambert',
        executable='lambert_node',
        namespace='observer',
        name='lambert_node',
        parameters=parameters_file_list,
        respawn=True
    )

    power_filter_node = Node(
        package='seabot2_power_filter',
        executable='filter_power_node',
        namespace='observer',
        name='filter_power_node',
        parameters=parameters_file_list,
        respawn=True
    )

    density_node = Node(
        package='seabot2_density',
        executable='density_node',
        namespace='observer',
        name='density_node',
        parameters=parameters_file_list,
        respawn=True
    )

    return LaunchDescription([
        depth_filter_node,
        internal_sensor_filter,
        temperature_filter,
        kalman_node,
        lambert_node,
        power_filter_node,
        density_node
    ])
