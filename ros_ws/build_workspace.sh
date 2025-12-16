#!/bin/bash
# Build script for ROS 2 Humble workspace

echo "=========================================="
echo "Building ROS 2 Humble Workspace"
echo "=========================================="

# Source ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS 2 Humble"
else
    echo "✗ ROS 2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Navigate to workspace
cd "$(dirname "$0")"

echo ""
echo "Cleaning previous build..."
rm -rf build/ install/ log/

echo ""
echo "Building workspace..."
colcon build --symlink-install

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build completed successfully!"
    echo "=========================================="
    echo ""
    echo "To use the workspace, run:"
    echo "  source install/setup.bash"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "✗ Build failed!"
    echo "=========================================="
    exit 1
fi
