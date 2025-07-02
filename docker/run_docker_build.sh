#!/bin/bash

mkdir seabot2-ros -p
# shellcheck disable=SC2164
cd /seabot2-ros
cp -r -p /home/src .

# clean install directory
#rm -r install

colcon build #--packages-select seabot2_msgs seabot2_latlon_control

echo "All done building"
