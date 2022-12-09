import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    home_path = os.path.expanduser('~')
    parameters_file_list = []

    config_control = os.path.join(
        home_path,
        'config/default/',  # Directory where yaml are
        'control.yaml'  # Name of the file
    )
    if os.path.exists(config_control):
        parameters_file_list.append(config_control)

    config_physics = os.path.join(
        home_path,
        'config/default/',  # Directory where yaml are
        'physics.yaml'  # Name of the file
    )
    if os.path.exists(config_physics):
        parameters_file_list.append(config_control)

    seabot2_depth_control = Node(
        package='seabot2_depth_control',
        executable='depth_control_node',
        namespace='control',
        name='depth_control_node',
        parameters=parameters_file_list
    )

    return LaunchDescription([
        seabot2_depth_control
    ])
