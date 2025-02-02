#!/bin/bash

mkdir seabot2-ros -p
# shellcheck disable=SC2164
cd /seabot2-ros
cp -r -p /home/src .

# clean install directory
#rm -r install

colcon build

echo "All done building"
