import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    mission = Node(
        package='seabot2_mission',
        executable='mission_node',
        namespace='mission',
        name='mission_node',
        parameters=[config_observer, config_physics],
        respawn=True
    )

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('seabot2'), 'launch'),
            '/driver_launch.py'])
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('seabot2'), 'launch'),
            '/control_launch.py'])
    )

    observer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('seabot2'), 'launch'),
            '/observer_launch.py'])
    )

    return LaunchDescription([
        drivers,
        control,
        observer,
        mission
    ])