import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import yaml
import numpy as np

def generate_launch_description():
    home_path = os.path.expanduser('~')
    parameters_file_list = []

    print(sys.argv)

    # Load config simulation
    config_simulation = os.path.join(
        home_path,
        'config/',  # Directory where yaml are
        'simulation.yaml'  # Name of the file
    )
    if os.path.exists(config_simulation):
        parameters_file_list.append(config_simulation)

        # Load temperature profile
        stream = open(config_simulation, 'r')
        yaml_file = yaml.load(stream)
        try:
            file_temp_profile = home_path + "/" + yaml_file['simulation_node']['ros__parameters']['temperature_profile_file']

            if os.path.exists(file_temp_profile):
                data = np.loadtxt(file_temp_profile)
                parameters_file_list.append({'temperature_profile_depth': data[0].tolist()})
                parameters_file_list.append({'temperature_profile_temp': data[1].tolist()})
                #print(parameters_file_list)
        except:
            parameters_file_list.append({'temperature_profile_depth': [0.]})
            parameters_file_list.append({'temperature_profile_temp': [0.]})
            print("No temperature profile file found")
            pass

    # Load alpha values
    file_alpha = home_path + "/config/default/alpha_values.txt"
    if os.path.exists(file_alpha):
        data = np.loadtxt(file_alpha)
        parameters_file_list.append({'solver_alpha': data[0].tolist()})
        parameters_file_list.append({'solver_velocity': data[1].tolist()})

    seabot2_simulator = Node(
        package='seabot2_simulator',
        executable='simulator_node',
        namespace='',
        name='simulation_node',
        parameters=parameters_file_list,
        output='screen',
    )

    return LaunchDescription([
        seabot2_simulator
    ])
