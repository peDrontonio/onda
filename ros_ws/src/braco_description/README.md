# Braco RRRPR Manipulator - URDF Package

This package contains the URDF description for the Braco subsea RRRPR manipulator arm for use with ROS 2 Humble and roboticstoolbox-python.

## Package Structure

```
braco_description/
├── urdf/
│   ├── Braço.xacro          # Main xacro file (source)
│   ├── braco.urdf           # Generated URDF file (use this!)
│   ├── Braço.trans          # Transmission definitions
│   ├── Braço.gazebo         # Gazebo-specific properties
│   └── materials.xacro      # Material definitions
├── meshes/
│   ├── base_link.stl
│   ├── rot1_1.stl
│   ├── rot2_1.stl
│   ├── rot3_1.stl
│   ├── prism1_1.stl
│   └── rot4_1.stl
├── launch/                  # ROS 2 launch files
├── scripts/
│   ├── convert_xacro_manual.py  # Script to regenerate URDF from xacro
│   └── convert_xacro_proper.py  # Alternative conversion script
└── README.md
```

## Robot Configuration

**Kinematic Structure:** RRRPR (5 DOF)
- Joint 1 (base_rot1): Revolute - Base rotation (0° to 360°)
- Joint 2 (rot1_rot2): Revolute - Shoulder (0° to 360°)
- Joint 3 (rot2_rot3): Revolute - Elbow (0° to 360°)
- Joint 4 (rot3_prism1): Prismatic - Linear extension (0 to 0.1m)
- Joint 5 (prism1_rot4): Revolute - Wrist (-60° to 60°)

## Fixed URDF Issues

### Problem
The original URDF file had **absolute file paths** instead of package URIs:
- ❌ `/home/host/ros_ws/braco_description/meshes/base_link.stl`

This caused the Swift visualization to fail because roboticstoolbox couldn't find the mesh files.

### Solution
The URDF now uses proper **package URIs**:
- ✅ `package://braco_description/meshes/base_link.stl`

These URIs are resolved at runtime to the correct absolute paths.

## Regenerating the URDF

If you modify the xacro files, regenerate the URDF using:

```bash
cd /home/host/onda/ros_ws/braco_description
python3 scripts/convert_xacro_manual.py
```

This script:
1. Processes the xacro file and includes
2. Resolves xacro properties
3. **Preserves package:// URIs** for mesh files
4. Generates a clean URDF file

## Using with roboticstoolbox-python and Swift

### Installation

Make sure you have the required dependencies:

```bash
pip3 install roboticstoolbox-python
pip3 install swift-sim
pip3 install 'numpy>=1.17.3,<1.25.0'  # Compatible version
```

### Loading the URDF

```python
import roboticstoolbox as rtb
import os

# Path to the URDF package
package_path = "/home/host/onda/ros_ws/braco_description"
urdf_file = os.path.join(package_path, "urdf", "braco.urdf")

# Read and resolve package:// URIs
with open(urdf_file, 'r') as f:
    urdf_string = f.read()

# Replace package:// with file:// for roboticstoolbox
urdf_string = urdf_string.replace(
    'package://braco_description',
    f'file://{package_path}'
)

# Save temporary URDF
temp_urdf = "/tmp/braco_resolved.urdf"
with open(temp_urdf, 'w') as f:
    f.write(urdf_string)

# Load the robot
arm = rtb.Robot.URDF(temp_urdf)
print(f"Loaded robot: {arm.name} with {arm.n} joints")
```

### Example Scripts

1. **Test URDF Loading:**
   ```bash
   cd /home/host/onda/peter_corke/scripts
   python3 test_urdf_load.py
   ```

2. **Swift Visualization with GUI:**
   ```bash
   cd /home/host/onda/peter_corke/scripts
   python3 swift_test_urdf.py
   ```

## Using with ROS 2

### Visualize in RViz2

```bash
ros2 launch braco_description display.launch.py
```

### Spawn in Gazebo

```bash
ros2 launch braco_description gazebo.launch.py
```

## Verification

To verify the URDF is correctly configured:

```bash
# Check that meshes use package:// URIs
grep "package://" /home/host/onda/ros_ws/braco_description/urdf/braco.urdf

# Run the test script
python3 /home/host/onda/peter_corke/scripts/test_urdf_load.py
```

Expected output:
- ✓ Successfully loaded robot: Braco
- ✓ Number of joints: 5
- ✓ All links have mesh geometry
- ✓ Forward kinematics working
- ✓ Jacobian computation working

## Troubleshooting

### NumPy Version Error
If you see `ValueError: numpy.dtype size changed`, install a compatible version:
```bash
pip3 install 'numpy>=1.17.3,<1.25.0'
```

### Meshes Not Loading
1. Verify URDF uses `package://` URIs (not absolute paths)
2. Check that mesh files exist in `meshes/` directory
3. Ensure package path is correctly resolved in your script

### Swift Websocket Error
The Swift visualization may show a websocket error but the robot model will still load and display correctly. This is a known issue with the Swift server initialization.

## License

MIT License - See LICENSE file for details

## Maintainer

Pedro Antonio <pedrinho@todo.todo>
