import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from datetime import datetime
import socket

def generate_launch_description():
    home_path = os.path.expanduser('~')
    hostname = socket.gethostname()

    time_now = datetime.utcnow()
    bag_path = home_path + '/' + hostname + "_" + str(time_now.year) + "_" + str(time_now.month) + "_" + str(time_now.day) + "-" + str(time_now.hour) + "_" + str(time_now.minute) + "_" + str(time_now.second)

    print(bag_path)

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_path],
        output='screen'
    )

    # drivers = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('seabot2'), 'launch'),
    #         '/driver_launch.py'])
    # )

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
        ## drivers,
        control,
        observer,
        mission
    ])