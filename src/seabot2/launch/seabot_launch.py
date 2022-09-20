import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
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

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('seabot2'), 'launch'),
            '/mission_launch.py'])
    )

    return LaunchDescription([
        bag,
        drivers,
        control,
        observer,
        mission
    ])