import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys

def generate_launch_description():
    home_path = os.path.expanduser('~')
    parameters_file_list = []

    print(sys.argv)

    config_simulation = os.path.join(
        home_path,
        'config/',  # Directory where yaml are
        'simulation.yaml'  # Name of the file
    )
    if os.path.exists(config_simulation):
        parameters_file_list.append(config_simulation)

    seabot2_simulator = Node(
        package='seabot2_simulator',
        executable='simulator_node',
        namespace='',
        name='simulation_node',
        parameters=parameters_file_list
    )

    return LaunchDescription([
        seabot2_simulator
    ])
