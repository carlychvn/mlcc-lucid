# MLCC Quick Start Guide

## Setup Complete! ✅

Your MLCC calibration environment is ready to use.

## Quick Commands

### Start Docker Container
```bash
./docker-run.sh
```

### Inside Container - Run Calibration

**Multi-LiDAR Calibration (3 steps):**
```bash
# Step 1: Optimize base LiDAR poses
roslaunch mlcc pose_refine.launch rviz:=false

# Step 2: (Optional) Refine individual LiDAR extrinsics
roslaunch mlcc extrinsic_refine.launch rviz:=false

# Step 3: Global joint optimization
timeout 120 roslaunch mlcc global.launch rviz:=false
```

**Camera Calibration:**
```bash
# Multi-camera calibration
timeout 180 roslaunch mlcc calib_camera.launch

# Single camera calibration (needs data path config)
timeout 180 roslaunch mlcc calib_single_camera.launch
```

## Results Location

All results are saved in:
- `/Users/carly/Desktop/mlcc/result/` (on your Mac)
- `/root/catkin_ws/src/mlcc/result/` (inside Docker)

## What Was Accomplished

✅ Built Docker environment with ROS Noetic on macOS
✅ Compiled all 5 MLCC calibration executables
✅ Ran base LiDAR pose optimization (Step 1)
✅ Ran global joint optimization (Step 3)
✅ Generated calibration results for scene2

## Output Files

- `scene2/pose.json` - Optimized LiDAR poses (21 frames)
- `scene2/ref.json` - Refined multi-LiDAR extrinsic parameters
- `result/extrinsic.txt` - Camera extrinsic matrices

## Configuration Files

Edit these before running on your own data:
- `launch/*.launch` - Processing parameters
- `config/*.yaml` - Camera and scene configuration

## Documentation

- `DOCKER_SETUP.md` - Comprehensive Docker setup guide
- `CALIBRATION_RESULTS.md` - Detailed results from calibration runs
- `README.md` - Original project documentation

## Troubleshooting

**Out of Memory Errors:**
- Increase Docker memory: Docker Desktop → Settings → Resources
- Recommended: 8GB+ for camera calibration

**Path Errors:**
- Update `config/scene_config.yaml` DataPath for your data
- Ensure .pcd files exist at specified locations

**Container Issues:**
```bash
# Clean restart
docker-compose down -v
./docker-build.sh
```

## Support

- GitHub Issues: https://github.com/hku-mars/mlcc/issues
- Authors: xliuaa@connect.hku.hk, xy19980205@outlook.com
