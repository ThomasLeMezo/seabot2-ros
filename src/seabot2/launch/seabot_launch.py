import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    home_path = os.path.expanduser('~')

    bag_path = home_path + "/log/"
    if not os.path.exists(bag_path):
        os.makedirs(bag_path)
    os.chdir(bag_path)

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen',
        respawn=True
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
       control,
       observer,
       mission
    ])