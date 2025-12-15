#!/bin/bash
# Test script for Braco ROS 2 setup
# This script verifies the ROS 2 migration

set -e

echo "=========================================="
echo "  Braco ROS 2 Setup Test"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Function to test
test_item() {
    local name="$1"
    local command="$2"
    
    echo -n "Testing: $name... "
    
    if eval "$command" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ PASS${NC}"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC}"
        ((TESTS_FAILED++))
        return 1
    fi
}

echo "1. Checking ROS 2 Installation"
echo "------------------------------------------"
test_item "ROS 2 Humble" "test -d /opt/ros/humble"
test_item "colcon" "which colcon"
test_item "ROS 2 sourced" "test -n \"\$ROS_DISTRO\""
echo ""

echo "2. Checking Package Files"
echo "------------------------------------------"
test_item "package.xml" "test -f Braço_description/package.xml"
test_item "CMakeLists.txt" "test -f Braço_description/CMakeLists.txt"
test_item "URDF file" "test -f Braço_description/urdf/Braço.xacro"
test_item "Display launch" "test -f Braço_description/launch/display.launch.py"
test_item "Gazebo launch" "test -f Braço_description/launch/gazebo.launch.py"
echo ""

echo "3. Checking Mesh Files"
echo "------------------------------------------"
test_item "base_link.stl" "test -f Braço_description/meshes/base_link.stl"
test_item "rot1_1.stl" "test -f Braço_description/meshes/rot1_1.stl"
test_item "rot2_1.stl" "test -f Braço_description/meshes/rot2_1.stl"
test_item "rot3_1.stl" "test -f Braço_description/meshes/rot3_1.stl"
test_item "prism1_1.stl" "test -f Braço_description/meshes/prism1_1.stl"
test_item "rot4_1.stl" "test -f Braço_description/meshes/rot4_1.stl"
echo ""

echo "4. Checking Package Name in Files"
echo "------------------------------------------"
if grep -q "braco_description" Braço_description/package.xml 2>/dev/null; then
    echo -e "Package name in package.xml: ${GREEN}✓ braco_description${NC}"
    ((TESTS_PASSED++))
else
    echo -e "Package name in package.xml: ${RED}✗ Still Braço_description${NC}"
    ((TESTS_FAILED++))
fi

if grep -q "braco_description" Braço_description/urdf/Braço.xacro 2>/dev/null; then
    echo -e "Package name in URDF: ${GREEN}✓ braco_description${NC}"
    ((TESTS_PASSED++))
else
    echo -e "Package name in URDF: ${RED}✗ Still Braço_description${NC}"
    ((TESTS_FAILED++))
fi
echo ""

echo "5. Build System Check"
echo "------------------------------------------"
if grep -q "ament_cmake" Braço_description/package.xml 2>/dev/null; then
    echo -e "Build system: ${GREEN}✓ ament_cmake (ROS 2)${NC}"
    ((TESTS_PASSED++))
elif grep -q "catkin" Braço_description/package.xml 2>/dev/null; then
    echo -e "Build system: ${RED}✗ catkin (ROS 1)${NC}"
    ((TESTS_FAILED++))
else
    echo -e "Build system: ${YELLOW}? Unknown${NC}"
    ((TESTS_FAILED++))
fi
echo ""

echo "=========================================="
echo "  Test Summary"
echo "=========================================="
echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Build the package:"
    echo "     cd /home/host/ros_ws"
    echo "     colcon build --packages-select braco_description --symlink-install"
    echo ""
    echo "  2. Source the workspace:"
    echo "     source install/setup.bash"
    echo ""
    echo "  3. Launch RViz:"
    echo "     ros2 launch braco_description display.launch.py"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please check the issues above.${NC}"
    echo ""
    echo "Common fixes:"
    echo "  - Make sure you've run the migration (files should be updated)"
    echo "  - Check that ROS 2 Humble is installed"
    echo "  - Verify all files are in place"
    echo ""
    exit 1
fi
