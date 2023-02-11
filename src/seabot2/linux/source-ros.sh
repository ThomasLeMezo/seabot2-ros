# ROS2
source /opt/ros/rolling/setup.zsh
export ROS_DOMAIN_ID=$HOST[-1]
export ROS_LOCALHOST_ONLY=1
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

# seabot2-ros
source /home/pi/seabot2-ros/install/local_setup.zsh
export ROS_HOME=/home/pi/.ros/
export ROS_LOG_DIR=$ROS_HOME/log/