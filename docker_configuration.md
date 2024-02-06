

## Run docker container
```
mkdir -p ~/seabot2/seabot2-ros/.arm64
sudo docker run --platform linux/arm64 -v ~/seabot2/seabot2-ros:/home -v ~/seabot2/seabot2-ros/.arm64:/seabot2-ros -it arm64v8/ros:rolling-seafoil-v4
```

ToDo
* sudo apt install -y libi2c-dev libi2c-dev python3-pip libboost-all-dev
* pip3 install jinja2

###
```
mkdir /seabot2-ros -p && cd /seabot2-ros && rsync -r /home/src . && colcon build

rsync -r --info=progress2 install pi@192.168.0.102:~/seabot2-ros/
```