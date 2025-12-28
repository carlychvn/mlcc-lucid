# MLCC Docker Setup Guide

This guide explains how to run the MLCC (Multi-LiDAR-Camera Calibration) project using Docker on macOS.

## Prerequisites

- Docker Desktop for Mac (already installed)
- Approximately 2GB of disk space for the Docker image

## Quick Start

### 1. Build the Docker Environment (One-time Setup)

Run the build script to create the Docker image with ROS Noetic and all dependencies:

```bash
./docker-build.sh
```

This will:
- Build a Docker image with Ubuntu 20.04 and ROS Noetic
- Install all required dependencies (PCL, OpenCV, Eigen, Ceres, etc.)
- Compile the MLCC project

**Note:** This process takes 5-10 minutes on first run.

### 2. Run the Container

Start an interactive shell in the Docker container:

```bash
./docker-run.sh
```

You'll be dropped into a bash shell inside the container with ROS Noetic fully configured.

## Working Inside the Container

Once inside the container, the project is located at `/root/catkin_ws/src/mlcc`.

### Building the Project

If you make code changes, rebuild with:

```bash
cd /root/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### Running the Calibration Tools

#### Multi-LiDAR Extrinsic Calibration (3 steps)

**Step 1: Base LiDAR pose optimization**
```bash
roslaunch mlcc pose_refine.launch
```

**Step 2: LiDAR extrinsic optimization**
```bash
roslaunch mlcc extrinsic_refine.launch
```

**Step 3: Pose and extrinsic joint optimization**
```bash
roslaunch mlcc global.launch
```

#### Multi-LiDAR-Camera Extrinsic Calibration
```bash
roslaunch mlcc calib_camera.launch
```

#### Single LiDAR-Camera Calibration
```bash
roslaunch mlcc calib_single_camera.launch
```

## Configuration

Before running the calibration, you need to configure parameters in the launch files located at:
- `/root/catkin_ws/src/mlcc/launch/`

Key parameters to adjust:
- `base_lidar`: AVIA or MID
- `test_scene`: scene-1 or scene-2
- `adaptive_voxel_size`: Voxel size for adaptive voxelization
- `voxel_size`: Voxel size for calibration
- `feat_eigen_limit`: Feature eigenvalue ratio threshold
- `downsmp_sz_base`: Downsampling size for base LiDAR

Configuration files are in:
- `/root/catkin_ws/src/mlcc/config/`

## Data

The project includes two test scenes:
- `scene1/`: First calibration scene
- `scene2/`: Second calibration scene

Original rosbag files are referenced in the README:
- [scene-1 rosbag](https://drive.google.com/file/d/1LTJSKkwGJX0P8MjpqKcM7L3s6aS46D-r/view?usp=sharing)
- [scene-2 rosbag](https://drive.google.com/file/d/1yzuPrCLGk1bUDjMc9WdLFVbBfBSRTc3F/view?usp=sharing)

To use your own data, save LiDAR point clouds in `.pcd` format in the respective scene directories.

## Useful Commands

### Docker Container Management

```bash
# Start a new container
./docker-run.sh

# Connect to an already running container
./docker-shell.sh

# Stop all running containers
docker-compose down

# Remove all build artifacts (start fresh)
docker-compose down -v
```

### Accessing Files

All files in the `/Users/carly/Desktop/mlcc` directory on your Mac are automatically mounted to `/root/catkin_ws/src/mlcc` in the container. Any changes you make inside the container will be reflected on your Mac and vice versa.

### Viewing Results

Results are saved in the `result/` directory, which is accessible both inside the container and on your Mac at `/Users/carly/Desktop/mlcc/result/`.

## Troubleshooting

### Build Errors

If you encounter build errors, try rebuilding from scratch:

```bash
docker-compose down -v
./docker-build.sh
```

### ROS Not Found

If ROS commands aren't working inside the container, source the setup file:

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
```

### File Permissions

If you have permission issues with files created inside the container, they may be owned by root. You can change ownership on your Mac:

```bash
sudo chown -R $(whoami) /Users/carly/Desktop/mlcc
```

## Visualization with RViz

RViz visualization is included but requires X11 forwarding to display on macOS. To enable:

1. Install XQuartz: `brew install --cask xquartz`
2. Launch XQuartz and allow network connections
3. Run: `xhost +localhost`
4. Restart the Docker container

Then inside the container, you can run:
```bash
rosrun rviz rviz -d /root/catkin_ws/src/mlcc/rviz_cfg/show_calib.rviz
```

## Additional Resources

- [Paper on IEEE TIM](https://ieeexplore.ieee.org/document/9779777)
- [Experiment Video (YouTube)](https://www.youtube.com/watch?v=mRNXQEijtmI)
- [Experiment Video (Bilibili)](https://www.bilibili.com/video/BV1YL411J7v7/)

## Notes

- The container uses `network_mode: host` to allow ROS communication
- Build artifacts are persisted in Docker volumes for faster rebuilds
- The working directory inside the container is `/root/catkin_ws`
