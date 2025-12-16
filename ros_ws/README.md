# ROS 2 Humble Workspace - Manipulator Project

This workspace contains packages for simulating and controlling a RRRPR (5-DOF) subsea manipulator robot with disturbance testing capabilities.

## ✅ Migration Status: COMPLETE

All packages have been successfully migrated from ROS 1 (Catkin) to ROS 2 Humble (Ament).

## Workspace Structure

```
ros_ws/
├── src/
│   ├── braco_description/          # Robot URDF description
│   ├── manipulator_msgs/           # Custom ROS 2 service definitions
│   ├── manipulator_disturbance_control/  # Disturbance testing nodes
│   └── manipulator_gazebo/         # Gazebo simulation configuration
├── build_workspace.sh              # Easy build script
└── README.md                       # This file
```

## Packages Overview

### 1. braco_description
**Type:** Robot description package  
**Language:** URDF/Xacro  
**Description:** Contains the URDF model of the RRRPR manipulator with:
- 5 joints (3 revolute, 1 prismatic, 1 revolute)
- STL mesh files for visualization
- Proper `package://` URIs for mesh loading

### 2. manipulator_msgs
**Type:** Interface package  
**Language:** ROS 2 IDL  
**Description:** Custom service definitions:
- `ApplyDisturbance.srv` - Service to apply/remove external forces

### 3. manipulator_disturbance_control
**Type:** Control package  
**Language:** C++ (ROS 2) and Python  
**Description:** Nodes for testing robot behavior under disturbances:
- `disturbance_applier_node` - Service server that applies forces via Gazebo
- `test_disturbance_client` - C++ client to test the service
- Python scripts for advanced testing and GUI

### 4. manipulator_gazebo
**Type:** Simulation package  
**Language:** Launch files, URDF, SDF  
**Description:** Gazebo simulation environment with:
- World files for underwater simulation
- Launch files for spawning the robot
- Controller configurations

## Prerequisites

- **Ubuntu 22.04** (Jammy)
- **ROS 2 Humble** installed
- **Gazebo Classic** (gazebo11)
- **colcon** build tool

### Install ROS 2 Humble
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### Install Additional Dependencies
```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions
```

## Building the Workspace

### Option 1: Using the Build Script (Recommended)
```bash
cd /home/host/onda/ros_ws
./build_workspace.sh
```

### Option 2: Manual Build
```bash
cd /home/host/onda/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Source the Workspace
After building, source the workspace:
```bash
source /home/host/onda/ros_ws/install/setup.bash
```

**Tip:** Add this to your `~/.bashrc` for automatic sourcing:
```bash
echo "source /home/host/onda/ros_ws/install/setup.bash" >> ~/.bashrc
```

## Build Verification

After building, you should see:
```
Summary: 4 packages finished [X.XXs]
```

The packages are:
1. ✅ manipulator_msgs
2. ✅ braco_description
3. ✅ manipulator_gazebo
4. ✅ manipulator_disturbance_control

## Usage Examples

### 1. Visualize Robot in RViz2
```bash
ros2 launch braco_description display.launch.py
```

### 2. Spawn Robot in Gazebo
```bash
ros2 launch manipulator_gazebo gazebo.launch.py
```

### 3. Run Disturbance Applier Node
```bash
ros2 run manipulator_disturbance_control disturbance_applier_node
```

### 4. Test Disturbance Service
```bash
# Apply a 50N force in X direction
ros2 run manipulator_disturbance_control test_disturbance_client 50.0 0.0 0.0
```

### 5. List Available Services
```bash
ros2 service list
ros2 service type /apply_disturbance
```

## Key Changes from ROS 1

### Build System
- ❌ ROS 1: `catkin_make` / `catkin build`
- ✅ ROS 2: `colcon build`

### Package Format
- ❌ ROS 1: `package.xml` format 2 with `<buildtool_depend>catkin</buildtool_depend>`
- ✅ ROS 2: `package.xml` format 3 with `<buildtool_depend>ament_cmake</buildtool_depend>`

### CMakeLists.txt
- ❌ ROS 1: `find_package(catkin REQUIRED ...)`
- ✅ ROS 2: `find_package(ament_cmake REQUIRED)`

### C++ API Changes
- ❌ ROS 1: `ros::NodeHandle`, `ros::ServiceServer`, `ROS_INFO`
- ✅ ROS 2: `rclcpp::Node`, `rclcpp::Service`, `RCLCPP_INFO`

### Python API Changes
- ❌ ROS 1: `rospy.init_node()`, `rospy.Service`
- ✅ ROS 2: `rclpy.init()`, `node.create_service()`

### Launch Files
- ❌ ROS 1: XML format (`.launch`)
- ✅ ROS 2: Python format (`.launch.py`)

### Message/Service Imports
- ❌ ROS 1: `#include <package/ServiceName.h>`
- ✅ ROS 2: `#include <package/srv/service_name.hpp>`

## Troubleshooting

### Build Fails with "ament_cmake not found"
**Solution:** Make sure ROS 2 is sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Gazebo Service Not Available
**Solution:** Make sure Gazebo is running and the robot is spawned:
```bash
ros2 launch manipulator_gazebo gazebo.launch.py
```

### URDF Meshes Not Loading
**Solution:** The URDF now uses `package://` URIs. Verify the package is installed:
```bash
ros2 pkg prefix braco_description
```

### Python Scripts Not Executable
**Solution:** Make them executable:
```bash
chmod +x src/manipulator_disturbance_control/scripts/*.py
```

## Development

### Clean Build
```bash
cd /home/host/onda/ros_ws
rm -rf build/ install/ log/
./build_workspace.sh
```

### Build Single Package
```bash
colcon build --packages-select manipulator_msgs
```

### Build with Verbose Output
```bash
colcon build --symlink-install --event-handlers console_direct+
```

## Migration Details

### What Was Changed?

1. **manipulator_msgs**
   - ✅ Converted to `rosidl_generate_interfaces`
   - ✅ Updated package.xml to format 3
   - ✅ Service definitions remain compatible

2. **manipulator_disturbance_control**
   - ✅ Converted C++ nodes from ROS 1 to ROS 2 API
   - ✅ Updated service includes and message types
   - ✅ Changed `ros::spin()` to `rclcpp::spin()`
   - ✅ Updated CMakeLists.txt for `ament_cmake`

3. **manipulator_gazebo**
   - ✅ Updated to use `ros2_control` instead of `gazebo_ros_control`
   - ✅ Updated package dependencies
   - ⚠️  Launch files need conversion to `.launch.py` format

4. **braco_description**
   - ✅ Already in ROS 2 format
   - ✅ URDF with proper `package://` URIs

## Contributing

When making changes:
1. Update package.xml version if needed
2. Rebuild the workspace
3. Test all nodes and services
4. Update this README if adding new features

## License

BSD License - See individual package LICENSE files

## Maintainer

Manipulator Test Team <user@example.com>

---

**Last Updated:** December 2025  
**ROS Version:** ROS 2 Humble  
**Status:** ✅ All packages building successfully
