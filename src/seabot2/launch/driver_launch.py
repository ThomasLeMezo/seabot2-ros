import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    home_path = os.path.expanduser('~')
    parameters_file_list = []

    config_driver = os.path.join(
        home_path,
        'config',  # Directory where yaml are
        'driver.yaml'  # Name of the file
    )
    if os.path.exists(config_driver):
        parameters_file_list.append(config_driver)

    gpsd_node = Node(
        package='gpsd_client',
        executable='gpsd_node',
        namespace='driver',
        name='gpsd_node',
        parameters=parameters_file_list,
        respawn=True
    )

    bme280_node = Node(
        package='pressure_bme280_driver',
        executable='bme280_node',
        namespace='driver',
        name='bme280_node',
        parameters=parameters_file_list,
        respawn=True
    )

    ms5803_node = Node(
        package='pressure_ms5803_driver',
        executable='pressure_ms5803_node',
        namespace='driver',
        name='pressure_ms5803_node',
        parameters=parameters_file_list,
        respawn=True
    )

    light_node = Node(
        package='seabot2_light_driver',
        executable='light_node',
        namespace='driver',
        name='light_node',
        parameters=parameters_file_list,
        respawn=True
    )

    piston_node = Node(
        package='seabot2_piston_driver',
        executable='piston_node',
        namespace='driver',
        name='piston_node',
        parameters=parameters_file_list,
        respawn=True
    )

    power_node = Node(
        package='seabot2_power_driver',
        executable='power_node',
        namespace='driver',
        name='power_node',
        parameters=parameters_file_list,
        respawn=True
    )

    screen_node = Node(
        package='seabot2_screen_driver',
        executable='screen_node',
        namespace='driver',
        name='screen_node',
        parameters=parameters_file_list,
        respawn=True
    )

    thruster_node = Node(
        package='seabot2_thruster_driver',
        executable='thruster_node',
        namespace='driver',
        name='thruster_node',
        parameters=parameters_file_list,
        respawn=True
    )

    ping_node = Node(
        package='bluerobotics_ping_driver',
        executable='bluerobotics_ping_node',
        namespace='driver',
        name='ping_node',
        parameters=parameters_file_list,
        respawn=True
    )

    temperature_node = Node(
        package='temperature_tsys01_driver',
        executable='temperature_tsys01_node',
        namespace='driver',
        name='temperature_node',
        parameters=parameters_file_list,
        respawn=True
    )

    return LaunchDescription([
        gpsd_node,
        bme280_node,
        ms5803_node,
        light_node,
        piston_node,
        power_node,
        screen_node,
        thruster_node,
        temperature_node
    ])
