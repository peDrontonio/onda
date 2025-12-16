# ROS 2 Humble Migration Guide

## Overview

This guide documents the migration of the Braço robot package from ROS 1 (Catkin) to ROS 2 Humble (Colcon).

## Changes Made

### 1. Package Structure

#### package.xml
**Changes:**
- Updated format from `2` to `3`
- Changed package name: `Braço_description` → `braco_description` (removed special character)
- Replaced `catkin` with `ament_cmake`
- Updated dependencies:
  - `rospy` → removed (not needed for description package)
  - Added `urdf`, `xacro`
  - Added ROS 2 visualization packages
  - Added ROS 2 control packages

#### CMakeLists.txt
**Changes:**
- Replaced `catkin` with `ament_cmake`
- Added proper install rules for ROS 2
- Updated C++ standard to C++14
- Added compiler warnings
- Proper directory installation

### 2. URDF Files

**File:** `urdf/Braço.xacro`

**Changes:**
- Updated robot name: `Braço` → `Braco`
- Updated all package references: `Braço_description` → `braco_description`
- Fixed mesh paths to use new package name
- All `$(find Braço_description)` → `$(find braco_description)`

**Files updated:**
- `urdf/Braço.xacro` ✓
- `urdf/Braço.gazebo` (needs manual update if used)
- `urdf/Braço.trans` (needs manual update if used)

### 3. Launch Files

Created new Python launch files for ROS 2:

#### display.launch.py
- Replaces `display.launch` (XML format)
- Uses Python launch API
- Properly configures robot_state_publisher
- Launches joint_state_publisher_gui
- Launches RViz2

#### gazebo.launch.py
- Replaces `gazebo.launch` (XML format)
- Uses Python launch API
- Integrates with Gazebo ROS 2
- Spawns robot entity properly

**Note:** Old `.launch` files are kept for reference but won't work in ROS 2.

### 4. Swift Visualization Fix

Created `peter_corke/scripts/braco_swift_visualization.py`:
- Handles URDF loading gracefully
- Falls back to DH parameters if URDF fails
- Provides clear error messages
- Works independently of ROS version

## Building in ROS 2

### Prerequisites

1. **Install ROS 2 Humble:**
```bash
# Already included in Docker image
```

2. **Install Dependencies:**
```bash
sudo apt update
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers
```

### Building the Package

#### In Docker (Recommended):

```bash
# Build Docker image
./docker/scripts/build.sh

# Run container
./docker/scripts/run.sh

# Inside container
cd ros_ws
colcon build --packages-select braco_description --symlink-install
source install/setup.bash
```

#### On Host (if ROS 2 installed):

```bash
cd ros_ws
colcon build --packages-select braco_description --symlink-install
source install/setup.bash
```

## Running the Package

### 1. RViz Visualization

```bash
# Source the workspace
source ros_ws/install/setup.bash

# Launch RViz with robot
ros2 launch braco_description display.launch.py
```

### 2. Gazebo Simulation

```bash
# Source the workspace
source ros_ws/install/setup.bash

# Launch Gazebo
ros2 launch braco_description gazebo.launch.py
```

### 3. Swift Visualization (Python/ROS-independent)

```bash
# Activate conda environment
conda activate onda_env

# Run Swift visualization
cd peter_corke/scripts
python braco_swift_visualization.py
```

## Troubleshooting

### Issue 1: Package Not Found

**Problem:** `Package 'braco_description' not found`

**Solution:**
```bash
cd ros_ws
colcon build --packages-select braco_description
source install/setup.bash
```

### Issue 2: Mesh Files Not Loading

**Problem:** Meshes not displaying in RViz/Gazebo

**Solution:**
- Check that package name in URDF matches: `braco_description`
- Verify mesh files exist in `meshes/` directory
- Check file permissions

### Issue 3: Launch File Not Found

**Problem:** `Unable to find launch file`

**Solution:**
- Use `.launch.py` extension for ROS 2
- Ensure files are installed: check `CMakeLists.txt` install rules
- Rebuild: `colcon build --packages-select braco_description`

### Issue 4: Swift URDF Loading Fails

**Problem:** Swift can't load URDF

**Solution:**
- Use the helper script: `braco_swift_visualization.py`
- It automatically falls back to DH parameters
- Swift works independently of ROS installation

### Issue 5: colcon Not Found

**Problem:** `colcon: command not found`

**Solution:**
```bash
# Install colcon
sudo apt install python3-colcon-common-extensions

# Or in Docker, rebuild the image
./docker/scripts/build.sh
```

## Backward Compatibility

### ROS 1 Files (Kept for Reference)

The following ROS 1 files are kept but won't work in ROS 2:
- `launch/*.launch` (XML format)
- Any catkin-specific scripts

### Migration from ROS 1 Projects

If you have code using the old package:

1. **Update package name imports:**
   ```python
   # Old
   from Braço_description import ...
   
   # New
   from braco_description import ...
   ```

2. **Update launch commands:**
   ```bash
   # Old (ROS 1)
   roslaunch Braço_description display.launch
   
   # New (ROS 2)
   ros2 launch braco_description display.launch.py
   ```

3. **Update URDF paths:**
   ```xml
   <!-- Old -->
   <mesh filename="package://Braço_description/meshes/..."/>
   
   <!-- New -->
   <mesh filename="package://braco_description/meshes/..."/>
   ```

## Testing Checklist

After migration, verify:

- [x] Package builds successfully with colcon
- [ ] URDF loads in RViz2
- [ ] Robot displays correctly in RViz2
- [ ] Joint state publisher GUI works
- [ ] Gazebo simulation loads
- [ ] Robot spawns in Gazebo
- [ ] Meshes display correctly
- [ ] Swift visualization works (optional)
- [ ] No console errors or warnings

## Docker Integration

The Docker image (`docker/Dockerfile.roshumble`) includes:
- ✅ ROS 2 Humble Desktop Full
- ✅ All required ROS 2 packages
- ✅ Python Robotics Toolbox
- ✅ colcon build tool
- ✅ Gazebo support
- ✅ Visualization tools

## Next Steps

1. **Test in Docker:**
   ```bash
   ./docker/scripts/build.sh
   ./docker/scripts/run.sh
   # Inside container:
   cd ros_ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch braco_description display.launch.py
   ```

2. **Update Other Packages:**
   - `manipulator_disturbance_control` (if used with this robot)
   - Any custom controllers
   - Any custom nodes

3. **Test Swift Visualization:**
   ```bash
   conda activate onda_env
   cd peter_corke/scripts
   python braco_swift_visualization.py
   ```

## Summary of Files Changed

### Modified:
- `ros_ws/Braço_description/package.xml` - ROS 2 format 3
- `ros_ws/Braço_description/CMakeLists.txt` - ament_cmake
- `ros_ws/Braço_description/urdf/Braço.xacro` - Updated package references

### Created:
- `ros_ws/Braço_description/launch/display.launch.py` - ROS 2 Python launch
- `ros_ws/Braço_description/launch/gazebo.launch.py` - ROS 2 Python launch
- `peter_corke/scripts/braco_swift_visualization.py` - Swift helper
- `docs/ROS2_MIGRATION_GUIDE.md` - This document

### Kept (deprecated but not deleted):
- `ros_ws/Braço_description/launch/*.launch` - ROS 1 XML launches (for reference)

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- [ament_cmake Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [Python Launch Files](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)

---

**Last Updated:** December 2025  
**ROS 2 Version:** Humble Hawksbill  
**Status:** Migration Complete - Testing Required
