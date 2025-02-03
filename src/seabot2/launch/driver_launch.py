import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import socket
from enum import Enum, IntEnum

class PressureExternalSensor(Enum):
    none = 0
    ms5803 = 1
    ms5837 = 2

def test_enable_pressure(yaml_data, sensor_node_name):
    if sensor_node_name in yaml_data['driver']:
        if 'ros__parameters' in yaml_data['driver'][sensor_node_name]:
            if 'enable' in yaml_data['driver'][sensor_node_name]['ros__parameters']:
                return yaml_data['driver'][sensor_node_name]['ros__parameters']['enable']
    return False
def generate_launch_description():
    home_path = os.path.expanduser('~')
    hostname = socket.gethostname()
    parameters_file_list = []

    config_driver = os.path.join(
        home_path,
        'config/default/',  # Directory where yaml are
        'driver.yaml'  # Name of the file
    )
    if os.path.exists(config_driver):
        parameters_file_list.append(config_driver)

    config_driver_custom = os.path.join(
        home_path,
        'config/' + hostname + '/',  # Directory where yaml are
        'driver.yaml'  # Name of the file
    )

    pressure_external = PressureExternalSensor.none
    if os.path.exists(config_driver_custom):
        parameters_file_list.append(config_driver_custom)

        # read yaml file
        with open(config_driver_custom, "r") as stream:
            try:
                yaml_data = yaml.safe_load(stream)
                if test_enable_pressure(yaml_data, 'pressure_ms5803_node'):
                    pressure_external = PressureExternalSensor.ms5803
                elif test_enable_pressure(yaml_data, 'pressure_ms5837_node'):
                    pressure_external = PressureExternalSensor.ms5837
            except yaml.YAMLError as exc:
                print(exc)

    # Node list
    list_node = []

    gpsd_node = Node(
        package='gpsd_client',
        executable='gpsd_node',
        namespace='driver',
        name='gpsd_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(gpsd_node)

    bme280_node = Node(
        package='pressure_bme280_driver',
        executable='bme280_node',
        namespace='driver',
        name='bme280_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(bme280_node)

    ms5803_node = Node(
        package='pressure_ms5803_driver',
        executable='pressure_ms5803_node',
        namespace='driver',
        name='pressure_ms5803_node',
        output='screen',
        parameters=parameters_file_list
    )
    if pressure_external == PressureExternalSensor.ms5803:
        list_node.append(ms5803_node)

    ms5837_node = Node(
        package='pressure_ms5837_driver',
        executable='pressure_ms5837_node',
        namespace='driver',
        name='pressure_ms5837_node',
        output='screen',
        parameters=parameters_file_list
    )
    if pressure_external == PressureExternalSensor.ms5837:
        list_node.append(ms5837_node)

    light_node = Node(
        package='seabot2_light_driver',
        executable='light_node',
        namespace='driver',
        name='light_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(light_node)

    piston_node = Node(
        package='seabot2_piston_driver',
        executable='piston_node',
        namespace='driver',
        name='piston_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(piston_node)

    power_node = Node(
        package='seabot2_power_driver',
        executable='power_node',
        namespace='driver',
        name='power_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(power_node)

    screen_node = Node(
        package='seabot2_screen_driver',
        executable='screen_node',
        namespace='driver',
        name='screen_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(screen_node)

    thruster_node = Node(
        package='seabot2_thruster_driver',
        executable='thruster_node',
        namespace='driver',
        name='thruster_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(thruster_node)

    ping_node = Node(
        package='bluerobotics_ping_driver',
        executable='bluerobotics_ping_node',
        namespace='driver',
        name='ping_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(ping_node)

    temperature_node = Node(
        package='temperature_tsys01_driver',
        executable='temperature_tsys01_node',
        namespace='driver',
        name='temperature_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(temperature_node)

    xbee_node = Node(
        package='xbee_driver',
        executable='xbee',
        namespace='driver',
        name='xbee_node',
        output='screen',
        parameters=parameters_file_list
    )
    # list_node.append(xbee_node)

    icm20948_node = Node(
        package='icm20948_driver',
        executable='icm20948',
        namespace='driver',
        name='icm20948_node',
        output='screen',
        parameters=parameters_file_list
    )
    list_node.append(icm20948_node)

    audio_recorder = Node(
        package='seabot2_audio_recorder',
        executable='audio_recorder',
        namespace='driver',
        output='screen',
        name='audio_recorder',
        parameters=parameters_file_list
    )
    list_node.append(audio_recorder)

    return LaunchDescription(list_node)
