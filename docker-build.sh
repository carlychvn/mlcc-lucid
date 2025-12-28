#!/bin/bash

# Script to build the MLCC project inside Docker

set -e

echo "Building Docker image..."
docker-compose build

echo ""
echo "Building MLCC project with catkin_make..."
docker-compose run --rm mlcc bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"

echo ""
echo "Build complete! Use ./docker-run.sh to start a container."
