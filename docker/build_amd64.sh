#!/bin/zsh

mkdir -p ~/seabot2/seabot2-ros/.amd64

# Build the software in the amd64 container
docker run --platform linux/amd64 \
  -v ~/seabot2/seabot2-ros:/home \
  -v ~/seabot2/seabot2-ros/.amd64:/seabot2-ros \
  --rm tristanlefloch/seabot2-ros-linux \
  bash -c "/home/docker/run_docker_build.sh"

# Go to the directory
cd ~/seabot2/seabot2-ros/.amd64

## Create a tar.gz archive of the install directory
#tar -czf install.tar.gz install