import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_mission = os.path.join(
        get_package_share_directory('seabot2'),
        'config',  # Directory where yaml are
        'mission.yaml'  # Name of the file
    )

    config_safety = os.path.join(
        get_package_share_directory('seabot2'),
        'config',  # Directory where yaml are
        'safety.yaml'  # Name of the file
    )

    mission_path = os.path.expanduser('~')

    seabot2_mission = Node(
        package='seabot2_mission',
        executable='mission_node',
        namespace='mission',
        name='mission_node',
        parameters=[config_mission, {"mission_path": mission_path}]
        # respawn=True
    )

    seabot2_safety = Node(
        package='seabot2_safety',
        executable='safety_node',
        namespace='safety',
        name='safety_node',
        parameters=[config_safety]
        # respawn=True
    )

    return LaunchDescription([
        seabot2_mission
    ])
