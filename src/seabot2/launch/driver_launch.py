import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_driver = os.path.join(
        get_package_share_directory('seabot2'),
        'config',  # Directory where yaml are
        'driver.yaml'  # Name of the file
    )

    gpsd_node = Node(
        package='gpsd_client',
        executable='gpsd_node',
        namespace='driver',
        name='gpsd_node',
        parameters=[config_driver]
    )

    bme280_node = Node(
        package='pressure_bme280_driver',
        executable='bme280_node',
        namespace='driver',
        name='bme280_node',
        parameters=[config_driver]
    )

    ms5803_node = Node(
        package='pressure_ms5803_driver',
        executable='pressure_ms5803_node',
        namespace='driver',
        name='pressure_ms5803_node',
        parameters=[config_driver]
    )

    light_node = Node(
        package='seabot2_light_driver',
        executable='light_node',
        namespace='driver',
        name='light_node',
        parameters=[config_driver]
    )

    piston_node = Node(
        package='seabot2_piston_driver',
        executable='piston_node',
        namespace='driver',
        name='piston_node',
        parameters=[config_driver]
    )

    power_node = Node(
        package='seabot2_power_driver',
        executable='power_node',
        namespace='driver',
        name='power_node',
        parameters=[config_driver]
    )

    screen_node = Node(
        package='seabot2_screen_driver',
        executable='screen_node',
        namespace='driver',
        name='screen_node',
        parameters=[config_driver]
    )

    thruster_node = Node(
        package='seabot2_thruster_driver',
        executable='thruster_node',
        namespace='driver',
        name='thruster_node',
        parameters=[config_driver]
    )

    ping_node = Node(
        package='bluerobotics_ping_driver',
        executable='bluerobotics_ping_node',
        namespace='driver',
        name='ping_node',
        parameters=[config_driver]
    )

    return LaunchDescription([
        gpsd_node,
        bme280_node,
        ms5803_node,
        light_node,
        piston_node,
        power_node,
        screen_node,
        thruster_node
    ])
