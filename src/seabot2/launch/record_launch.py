import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                  LaunchConfiguration, LocalSubstitution,
                                  PythonExpression)

import sys

def generate_launch_description():
    home_path = "/home/pi"

    bag_path = home_path + "/log/"
    if not os.path.exists(bag_path):
        os.makedirs(bag_path)
    os.chdir(bag_path)

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='both',
        shell=True
    )

    return LaunchDescription([
        bag,
    ])
#
# if __name__ == '__main__':
#     global ls
#     ls = LaunchService(argv=sys.argv)
#     ls.include_launch_description(generate_launch_description())
#     sys.exit(ls.run())
