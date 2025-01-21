#!/bin/zsh

# Build the software in the arm64 container
docker run --platform linux/arm64 \
  -v ~/seabot2/seabot2-ros:/home \
  -v ~/seabot2/seabot2-ros/.arm64:/seabot2-ros \
  --rm thomaslemezo/ros-arm64-robot \
  bash -c "/home/docker/run_docker_build.sh"

# Go to the directory
cd ../.arm64

## Create a tar.gz archive of the install directory
#tar -czf install.tar.gz install