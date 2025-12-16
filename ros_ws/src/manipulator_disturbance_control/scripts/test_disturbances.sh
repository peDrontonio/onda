#!/bin/bash
# Quick test script for applying disturbances

echo "============================================"
echo "Manipulator Disturbance Test Script"
echo "============================================"
echo ""

# Check if service is available
echo "Checking if /apply_disturbance service is available..."
if ! rosservice list | grep -q "/apply_disturbance"; then
    echo "ERROR: Service /apply_disturbance not found!"
    echo "Make sure the disturbance_applier_node is running."
    exit 1
fi

echo "Service found!"
echo ""

# Test 1: Apply force in X direction
echo "Test 1: Applying 50N force in +X direction"
rosservice call /apply_disturbance "force: {x: 50.0, y: 0.0, z: 0.0}"
echo "Waiting 3 seconds..."
sleep 3

# Test 2: Remove force (toggle)
echo ""
echo "Test 2: Removing force (toggle off)"
rosservice call /apply_disturbance "force: {x: 50.0, y: 0.0, z: 0.0}"
echo "Waiting 3 seconds..."
sleep 3

# Test 3: Apply force in Y direction
echo ""
echo "Test 3: Applying 30N force in +Y direction"
rosservice call /apply_disturbance "force: {x: 0.0, y: 30.0, z: 0.0}"
echo "Waiting 3 seconds..."
sleep 3

# Test 4: Remove force
echo ""
echo "Test 4: Removing force"
rosservice call /apply_disturbance "force: {x: 0.0, y: 30.0, z: 0.0}"
echo "Waiting 2 seconds..."
sleep 2

# Test 5: Apply force in Z direction (upward)
echo ""
echo "Test 5: Applying 40N force in +Z direction (upward)"
rosservice call /apply_disturbance "force: {x: 0.0, y: 0.0, z: 40.0}"
echo "Waiting 3 seconds..."
sleep 3

# Test 6: Remove force
echo ""
echo "Test 6: Removing force"
rosservice call /apply_disturbance "force: {x: 0.0, y: 0.0, z: 40.0}"

echo ""
echo "============================================"
echo "Test sequence completed!"
echo "============================================"
