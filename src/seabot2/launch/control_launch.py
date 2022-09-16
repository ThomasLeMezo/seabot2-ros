import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

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

    seabot2_depth_control = Node(
        package='seabot2_depth_control',
        executable='depth_control_node',
        namespace='control',
        name='depth_control_node',
        parameters=[config_control, config_physics],
        respawn=True
    )

    return LaunchDescription([
        seabot2_depth_control
    ])
