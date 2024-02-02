

## Run docker container
```
mkdir -p ~/seabot2/seabot2-ros/.arm64
sudo docker run --platform linux/arm64 -v ~/seabot2/seabot2-ros:/home -v ~/seabot2/seabot2-ros/.arm64:/seabot2-ros -it arm64v8/ros:rolling-seafoil-v4
```

ToDo
* libi2c-dev libi2c-dev python3-pip libboost-all-dev
* 

###
```
mkdir /seabot2-ros -p && cd /seabot2-ros && cp -r -p /home/src . && colcon build
```