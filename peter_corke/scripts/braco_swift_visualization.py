#!/usr/bin/env python3
"""
Braco Robot - Swift Visualization
Load and visualize the Braco robot URDF in Swift simulator
"""

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import os
import sys

def load_braco_from_urdf(urdf_path=None):
    """
    Load Braco robot from URDF file
    
    Args:
        urdf_path: Path to URDF file (optional)
    
    Returns:
        robot: Robot model
        success: Boolean indicating if load was successful
    """
    if urdf_path is None:
        # Try to find URDF in ROS workspace
        possible_paths = [
            "/home/pedrinho/onda/ros_ws/Braço_description/urdf/Braço.xacro",
            "/home/pedrinho/onda/Braço_description/urdf/Braço.xacro",
            os.path.join(os.path.dirname(__file__), "../../ros_ws/Braço_description/urdf/Braço.xacro")
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                urdf_path = path
                break
    
    if urdf_path is None or not os.path.exists(urdf_path):
        print(f"ERROR: URDF file not found at {urdf_path}")
        print("Please provide the correct path to Braço.xacro")
        return None, False
    
    try:
        print(f"Loading URDF from: {urdf_path}")
        
        # Try loading with roboticstoolbox
        # Note: xacro files need to be processed first
        # For now, we'll use the DH parameters directly
        from braco_forward_kinematics import build_braco_robot
        
        robot = build_braco_robot()
        print("✓ Robot loaded successfully using DH parameters")
        return robot, True
        
    except Exception as e:
        print(f"ERROR loading URDF: {e}")
        print("\nTrying alternative method with DH parameters...")
        
        try:
            from braco_forward_kinematics import build_braco_robot
            robot = build_braco_robot()
            print("✓ Robot loaded successfully using DH parameters")
            return robot, True
        except Exception as e2:
            print(f"ERROR: {e2}")
            return None, False

def visualize_in_swift(robot, show_gui=True):
    """
    Visualize robot in Swift simulator
    
    Args:
        robot: Robot model
        show_gui: Show the Swift GUI (default: True)
    """
    try:
        # Import Swift
        import swift
        
        # Create Swift environment
        env = swift.Swift()
        env.launch(realtime=True)
        
        # Add robot to environment
        env.add(robot)
        
        print("\n" + "="*70)
        print("SWIFT VISUALIZATION")
        print("="*70)
        print("\nRobot successfully loaded in Swift simulator!")
        print("\nControls:")
        print("  - Use mouse to rotate view")
        print("  - Scroll to zoom")
        print("  - Update joint values with robot.q")
        print("\nExample: robot.q = [0.5, 0.5, 0.5, 0.05, 0.0]")
        print("\nPress Ctrl+C to exit")
        print("="*70 + "\n")
        
        # Set initial configuration
        robot.q = robot.qz
        
        # Keep simulation running
        if show_gui:
            try:
                while True:
                    env.step(0.05)
            except KeyboardInterrupt:
                print("\nExiting...")
        
        return env
        
    except ImportError as e:
        print(f"\nWARNING: Swift not available: {e}")
        print("Swift visualization requires:")
        print("  pip install swift-sim")
        print("\nYou can still use the robot model for kinematics calculations.")
        return None
    except Exception as e:
        print(f"\nERROR in Swift visualization: {e}")
        print("Continuing without visualization...")
        return None

def test_robot_kinematics(robot):
    """
    Test robot forward kinematics with sample configurations
    """
    print("\n" + "="*70)
    print("TESTING ROBOT KINEMATICS")
    print("="*70)
    
    # Test configurations
    configs = [
        ("Home", np.array([0.0, 0.0, 0.0, 0.0, 0.0])),
        ("Extended", np.array([np.pi/4, np.pi/6, np.pi/6, 0.05, 0.2])),
        ("Reach", np.array([np.pi/2, np.pi/3, np.pi/4, 0.08, -0.3])),
    ]
    
    for name, q in configs:
        T = robot.fkine(q)
        pos = T.t
        
        print(f"\n{name} Configuration:")
        print(f"  Joints: {np.rad2deg(q[:3])} (deg), {q[3]:.3f}m, {np.rad2deg(q[4]):.1f}°")
        print(f"  End-effector: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")
    
    print("\n" + "="*70 + "\n")

def main():
    """Main function"""
    print("\n" + "="*70)
    print(" " * 20 + "BRACO ROBOT - SWIFT VISUALIZATION")
    print("="*70 + "\n")
    
    # Load robot
    robot, success = load_braco_from_urdf()
    
    if not success:
        print("\nFailed to load robot. Exiting.")
        return 1
    
    # Test kinematics
    test_robot_kinematics(robot)
    
    # Try Swift visualization
    print("Attempting to launch Swift visualization...")
    env = visualize_in_swift(robot, show_gui=True)
    
    if env is None:
        print("\nSwift visualization not available.")
        print("Robot model is still loaded and can be used for:")
        print("  - Forward kinematics: robot.fkine(q)")
        print("  - Inverse kinematics: robot.ikine_LM(T)")
        print("  - Jacobian: robot.jacob0(q)")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
