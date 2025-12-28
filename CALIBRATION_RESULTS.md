# MLCC Calibration Results Summary

This document summarizes the calibration runs performed on the MLCC example data.

**Date:** December 23, 2025
**Scene:** scene2
**Base LiDAR:** AVIA (ID: 3)
**Reference LiDARs:** MID-100 units (IDs: 0, 1, 2)

---

## Calibration Workflow Completed

### ✅ Step 1: Base LiDAR Pose Optimization

**Command:** `roslaunch mlcc pose_refine.launch`

**Status:** COMPLETED SUCCESSFULLY

**Performance:**
- Total iterations: 10
- Average iteration time: 0.352 seconds
- Total processing time: ~3.5 seconds

**Output:**
- Updated pose file: `scene2/pose.json` (21 poses)
- Initial pose loaded from: `scene2/original_pose/3.json`

**Configuration:**
- Voxel size: 4.0
- Downsample base: 0.4
- Eigen threshold: 10.0

**Result:** Successfully optimized the base AVIA LiDAR poses across all frames.

---

### ⚠️ Step 2: LiDAR Extrinsic Optimization (Individual)

**Command:** `roslaunch mlcc extrinsic_refine.launch`

**Status:** ENCOUNTERED NUMERICAL ISSUE

**Issue:** Assertion failure `!std::isnan(residual2)` in optimization
- The individual extrinsic refinement encountered a NaN (Not a Number) error
- This step calibrates one reference LiDAR at a time
- Issue may be related to initial extrinsic values or numerical stability

**Note:** This step can be skipped when initial extrinsic values are available, proceeding directly to global optimization.

---

### ✅ Step 3: Joint Pose and Extrinsic Global Optimization

**Command:** `roslaunch mlcc global.launch`

**Status:** COMPLETED SUCCESSFULLY

**Performance:**
- Total iterations: 10
- Average iteration time: 1.343 seconds
- Total processing time: ~13.4 seconds

**Configuration:**
- Base LiDAR: 3 (AVIA)
- Reference LiDARs: 0, 1, 2 (all MID-100 units)
- Voxel size: 4.0
- Downsample base: 0.4
- Downsample ref: 0.2
- Eigen threshold: 20.0

**Output:**
- Updated extrinsic parameters in: `scene2/ref.json`

**Refined Extrinsic Parameters (tx ty tz qw qx qy qz):**
```
LiDAR 0: -0.152559  0.0249413  -0.127399  -0.00372219  0.000950952  -0.00748395  0.999965
LiDAR 1: -0.151981  0.00961512 -0.126382   0.00395626 -0.000382073   0.00713794 -0.999966
LiDAR 2: -0.152596  0.0249367  -0.127399  -0.00372226  0.000950898  -0.00748401  0.999965
```

**Result:** Successfully performed joint optimization of all LiDAR poses and extrinsic parameters.

---

### ⚠️ Multi-LiDAR-Camera Extrinsic Calibration

**Command:** `roslaunch mlcc calib_camera.launch`

**Status:** PROCESS TERMINATED (OOM)

**Issue:** Process killed with exit code -9 (likely out of memory)
- Loaded camera configurations successfully for 2 cameras
- Detected 3 LiDAR extrinsic parameters
- Process terminated during heavy computation phase

**Camera Configurations Loaded:**

**Camera 0:**
- Focal length: (863.59, 863.10)
- Principal point: (621.67, 533.97)
- Distortion: [-0.0944, 0.0947, -0.0081, 0.0001, 0.0001]

**Camera 1:**
- Focal length: (863.08, 862.56)
- Principal point: (628.94, 533.00)
- Distortion: [-0.0944, 0.0983, -0.0125, 0.0005, -0.0001]

**Recommendations:**
- Increase Docker container memory allocation
- Process images in smaller batches
- Use downsampled images if available

**Partial Output:**
Some camera extrinsic calibration results were written to: `result/extrinsic.txt`

```
Camera 0 Extrinsic Matrix:
 0.00138404  -0.999995   0.00282654   0.0202171
-0.0190134   -0.00285234 -0.999815    0.110485
 0.999818     0.00133004 -0.0190172  -0.0218314
 0            0           0            1

Camera 1 Extrinsic Matrix:
-0.00094278  -0.999901   -0.0140524  -0.0892047
-0.0194254    0.0140681  -0.999712    0.13136
 0.999811    -0.000669535 -0.0194367   0.0151308
 0            0            0            1
```

---

### ⚠️ Single LiDAR-Camera Calibration

**Command:** `roslaunch mlcc calib_single_camera.launch`

**Status:** CONFIGURATION ERROR

**Issue:** Hardcoded data path in configuration file
- `config/scene_config.yaml` points to: `/home/sam/Desktop/test/`
- This is the original author's development path
- No data found at this location

**Error Messages:**
- `Could not find file '/home/sam/Desktop/test/2.pcd'`
- `No image data!`
- `total point size 0`

**Required Fix:**
Update `config/scene_config.yaml` line 5:
```yaml
# Change from:
DataPath: "/home/sam/Desktop/test/"

# To (example):
DataPath: "/root/catkin_ws/src/mlcc/scene1/"  # or appropriate path
```

**Note:** The single camera calibration requires:
- Point cloud files (.pcd format) at specified indices
- Corresponding image files
- Proper path configuration in scene_config.yaml

---

## Summary

### Successfully Completed:
- ✅ Base LiDAR pose optimization (Step 1)
- ✅ Global pose and extrinsic joint optimization (Step 3)
- ✅ Partial camera calibration output generated

### Needs Attention:
- ⚠️ Individual extrinsic refinement (Step 2) - numerical stability issue
- ⚠️ Multi-LiDAR-camera calibration - memory constraints
- ⚠️ Single LiDAR-camera calibration - path configuration

### Key Achievements:
1. Successfully set up ROS Noetic environment in Docker on macOS
2. Built all MLCC executables successfully
3. Ran multi-LiDAR calibration pipeline on example data
4. Generated refined extrinsic parameters for 3 LiDAR sensors
5. Optimized 21 base LiDAR poses

---

## Files Modified/Generated:

### Input Data:
- `scene2/original_pose/3.json` - Initial AVIA poses
- `scene2/ref.json` - Initial extrinsic parameters

### Output Data:
- `scene2/pose.json` - Optimized base LiDAR poses (21 frames)
- `scene2/ref.json` - Refined multi-LiDAR extrinsic parameters
- `result/extrinsic.txt` - Camera extrinsic calibration (partial)

---

## Next Steps:

1. **For production use:**
   - Increase Docker memory allocation: Edit Docker Desktop → Resources → Memory
   - Or process camera calibration on a system with more RAM

2. **For single camera calibration:**
   - Download fisheye camera data from: https://drive.google.com/file/d/1TUQJOPmA_j0M_6qgbXZKjKlRo6AwZs4Z/view?usp=sharing
   - Update `config/scene_config.yaml` with correct data path
   - Re-run: `roslaunch mlcc calib_single_camera.launch`

3. **For complete multi-camera calibration:**
   - Consider processing scenes individually
   - Reduce image resolution if possible
   - Monitor Docker container resources during processing

---

## Technical Notes:

- **Processing Environment:** ROS Noetic in Docker on macOS (ARM64)
- **Build System:** catkin_make with C++17
- **Dependencies:** PCL 1.10, OpenCV 4.2, Eigen 3.3.7, Ceres 1.14
- **Adaptive Voxelization:** Enabled for all calibration steps
- **Visualization:** RViz disabled for headless operation

---

## References:

- Paper: "Targetless Extrinsic Calibration of Multiple Small FoV LiDARs and Cameras using Adaptive Voxelization"
- IEEE TIM: https://ieeexplore.ieee.org/document/9779777
- Repository: https://github.com/hku-mars/mlcc
