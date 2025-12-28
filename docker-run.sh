#!/bin/bash

# Script to run an interactive shell in the MLCC Docker container

echo "Starting MLCC Docker container..."
echo "The project is mounted at: /root/catkin_ws/src/mlcc"
echo ""
echo "Useful commands inside the container:"
echo "  catkin_make                              - Build the project"
echo "  source devel/setup.bash                  - Source the workspace"
echo "  roslaunch mlcc pose_refine.launch        - Run pose refinement"
echo "  roslaunch mlcc extrinsic_refine.launch   - Run extrinsic refinement"
echo "  roslaunch mlcc global.launch             - Run global refinement"
echo "  roslaunch mlcc calib_camera.launch       - Run camera calibration"
echo "  roslaunch mlcc calib_single_camera.launch - Run single camera calibration"
echo ""

docker-compose run --rm mlcc
