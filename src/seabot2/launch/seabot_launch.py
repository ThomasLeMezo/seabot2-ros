import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_control = os.path.join(
        get_package_share_directory('seabot2'),
        'config',  # Directory where yaml are
        'control.yaml'  # Name of the file
    )
    config_physics = os.path.join(
        get_package_share_directory('seabot2'),
        'config',  # Directory where yaml are
        'physics.yaml'  # Name of the file
    )

    mission_path = os.path.join(
        get_package_share_directory('seabot2'),
        'mission',  # Directory where yaml are
    )

    seabot2_mission = Node(
        package='seabot2_mission',
        executable='mission_node',
        namespace='mission',
        name='mission_node',
        parameters=[config_control, {"mission_path": mission_path}],
        respawn=True
    )

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
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
        bag,
        drivers,
        control,
        observer,
        seabot2_mission
    ])