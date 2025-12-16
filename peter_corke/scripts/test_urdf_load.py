#!/usr/bin/env python3
"""
Simple test to verify URDF loads correctly with meshes
"""
import numpy as np
import roboticstoolbox as rtb
import os

# Set up paths for the URDF package
package_path = "/home/host/onda/ros_ws/braco_description"
urdf_file = os.path.join(package_path, "urdf", "braco.urdf")

print("=" * 60)
print("Testing URDF Load for Braco Manipulator")
print("=" * 60)
print(f"\nURDF file: {urdf_file}")

# Load the URDF model
try:
    # Read URDF and replace package:// with file:// paths
    with open(urdf_file, 'r') as f:
        urdf_string = f.read()
    
    # Replace package:// URIs with file:// URIs
    urdf_string = urdf_string.replace(
        'package://braco_description',
        f'file://{package_path}'
    )
    
    # Save temporary URDF with resolved paths
    temp_urdf = "/tmp/braco_resolved.urdf"
    with open(temp_urdf, 'w') as f:
        f.write(urdf_string)
    
    print("\n✓ URDF paths resolved")
    
    # Load the robot model
    arm = rtb.Robot.URDF(temp_urdf)
    
    print(f"\n✓ Successfully loaded robot: {arm.name}")
    print(f"✓ Number of joints: {arm.n}")
    print(f"✓ Number of links: {len(arm.links)}")
    
    print("\nJoint Information:")
    print("-" * 60)
    for i, link in enumerate(arm.links):
        if link.isjoint:
            joint_type = "Prismatic" if link.isprismatic else "Revolute"
            limits = link.qlim if link.qlim is not None else "No limits"
            print(f"  Joint {i}: {link.name}")
            print(f"    Type: {joint_type}")
            print(f"    Limits: {limits}")
    
    print("\nLink Geometry Information:")
    print("-" * 60)
    for i, link in enumerate(arm.links):
        print(f"  Link {i}: {link.name}")
        if hasattr(link, 'geometry') and link.geometry:
            print(f"    Has geometry: {len(link.geometry)} objects")
            for j, geom in enumerate(link.geometry):
                print(f"      Geometry {j}: {type(geom).__name__}")
        else:
            print(f"    No geometry")
    
    # Test forward kinematics
    print("\nTesting Forward Kinematics:")
    print("-" * 60)
    q_test = np.zeros(arm.n)
    T = arm.fkine(q_test)
    print(f"  Initial pose (all joints at 0):")
    print(f"  Position: {T.t}")
    print(f"  ✓ Forward kinematics working")
    
    # Test Jacobian
    print("\nTesting Jacobian:")
    print("-" * 60)
    J = arm.jacob0(q_test)
    print(f"  Jacobian shape: {J.shape}")
    print(f"  ✓ Jacobian computation working")
    
    print("\n" + "=" * 60)
    print("✓ ALL TESTS PASSED!")
    print("✓ URDF package is correctly configured")
    print("=" * 60)
    
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
    exit(1)
