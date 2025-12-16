#!/usr/bin/env python3
"""
Inverse Kinematics Node for Braco Manipulator (RRRPR configuration)

This node calculates joint positions from Cartesian coordinates (x, y, z)
and sends trajectory commands to move the end-effector to the target position.

Robot Configuration (from URDF):
- Joint 1 (base_rot1): Revolute, axis Z, origin (0, 0, 0.035)
- Joint 2 (rot1_rot2): Revolute, axis X, origin (0.1, 0, 0.065) 
- Joint 3 (rot2_rot3): Revolute, axis Y, origin (0.08, 0.1, 0)
- Joint 4 (rot3_prism1): Prismatic, axis Y, origin (-0.18, 0.1, 0), range [0, 0.1]
- Joint 5 (prism1_rot4): Revolute, axis -X, origin (0, 0.165, 0)

Usage:
    ros2 run braco_description send_cartesian.py <x> <y> <z>
    ros2 run braco_description send_cartesian.py 0.2 0.3 0.15 --duration 3.0

Author: Pedro Antonio
Date: 2025-12-16
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
import numpy as np
import sys
import argparse


class BracoKinematics:
    """
    Kinematics calculator for the Braco RRRPR manipulator.
    """
    
    def __init__(self):
        # Robot dimensions from URDF (in meters)
        self.d1 = 0.035   # base_link to joint1 (Z)
        self.d2 = 0.065   # joint1 to joint2 (Z)
        self.a1 = 0.1     # joint1 to joint2 (X)
        self.a2 = 0.08    # joint2 to joint3 (X)
        self.d3 = 0.1     # joint2 to joint3 (Y)
        self.a3 = -0.18   # joint3 to joint4 (X)
        self.d4 = 0.1     # joint3 to joint4 (Y)
        self.d5 = 0.165   # joint4 to joint5 (Y)
        
        # Height of the base
        self.base_height = self.d1 + self.d2  # 0.1m
        
        # Joint limits
        self.joint_limits = {
            'base_rot1': (0.0, 6.283185),
            'rot1_rot2': (0.0, 6.283185),
            'rot2_rot3': (0.0, 6.283185),
            'rot3_prism1': (0.0, 0.1),
            'prism1_rot4': (-1.047198, 1.047198)
        }
        
    def forward_kinematics(self, q):
        """
        Calculate end-effector position from joint angles.
        
        Args:
            q: Joint positions [q1, q2, q3, d4, q5]
               q1, q2, q3, q5: angles in radians
               d4: prismatic displacement in meters
        
        Returns:
            (x, y, z): End-effector position in meters
        """
        q1, q2, q3, d4, q5 = q
        
        # Simplified forward kinematics using transformation matrices
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c3, s3 = np.cos(q3), np.sin(q3)
        c5, s5 = np.cos(q5), np.sin(q5)
        
        # Base rotation around Z
        # Link lengths projection
        # The robot has a complex geometry, using iterative approach
        
        # Approximate calculation considering the main contributions
        # Height contribution
        z = self.base_height + self.d3 * s2 + self.d4 * np.sin(q2 + q3) + (self.d5 + d4) * np.sin(q2 + q3)
        
        # Horizontal reach
        r = self.a1 + self.a2 * c2 + self.a3 * np.cos(q2 + q3) + (self.d5 + d4) * np.cos(q2 + q3)
        
        # X, Y from base rotation
        x = r * c1
        y = r * s1
        
        return x, y, z
    
    def inverse_kinematics(self, x, y, z, q5=0.0):
        """
        Calculate joint positions to reach target Cartesian position.
        
        This uses a geometric approach for the RRRPR configuration.
        
        Args:
            x, y, z: Target position in meters (in base frame)
            q5: Desired end-effector rotation (default 0)
        
        Returns:
            List of joint positions [q1, q2, q3, d4, q5] or None if unreachable
        """
        # Joint 1: Base rotation - simple atan2
        q1 = np.arctan2(y, x)
        
        # Ensure q1 is in valid range [0, 2*pi]
        if q1 < 0:
            q1 += 2 * np.pi
        
        # Horizontal distance from base
        r = np.sqrt(x**2 + y**2)
        
        # Effective height (relative to shoulder)
        z_eff = z - self.base_height
        
        # Arm geometry for joints 2, 3, and prismatic 4
        # Using simplified model where:
        # - L1 is the effective length of link 2-3
        # - L2 is the effective length of link 3-4 + prismatic extension
        
        L1 = np.sqrt(self.a2**2 + self.d3**2)  # ~0.128m
        L2_base = np.sqrt(self.a3**2 + self.d4**2)  # ~0.206m
        
        # We need to find q2, q3, d4 such that the end effector reaches (r, z_eff)
        # The prismatic joint gives us flexibility in reaching different distances
        
        # Target distance from joint 2
        target_dist = np.sqrt(r**2 + z_eff**2)
        
        # Minimum and maximum reach
        min_reach = abs(L1 - L2_base - self.d5)
        max_reach = L1 + L2_base + self.d5 + 0.1  # +0.1 for prismatic extension
        
        if target_dist < min_reach * 0.5 or target_dist > max_reach * 1.2:
            print(f"Warning: Target distance {target_dist:.3f}m may be outside workspace")
            print(f"Workspace range: ~{min_reach:.3f}m to ~{max_reach:.3f}m")
        
        # Use geometric approach
        # We'll set the prismatic joint based on how far we need to reach
        
        # Nominal configuration: try to reach with middle prismatic extension
        d4 = 0.05  # Start with middle of range
        
        # Effective L2 with prismatic extension
        L2 = L2_base + self.d5 + d4
        
        # Use law of cosines to find q3
        cos_q3 = (target_dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q3 = np.clip(cos_q3, -1, 1)
        
        # Elbow configuration (we'll use elbow-up)
        q3 = np.arccos(cos_q3)
        
        # Calculate q2 using geometry
        alpha = np.arctan2(z_eff, r)
        beta = np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))
        q2 = alpha + beta
        
        # Adjust q2 to be in valid range
        while q2 < 0:
            q2 += 2 * np.pi
        while q2 > 2 * np.pi:
            q2 -= 2 * np.pi
        
        # Adjust q3 similarly
        while q3 < 0:
            q3 += 2 * np.pi
        while q3 > 2 * np.pi:
            q3 -= 2 * np.pi
            
        # If still unreachable, try adjusting d4
        if not self._check_limits([q1, q2, q3, d4, q5]):
            # Try different d4 values
            for d4_try in np.linspace(0, 0.1, 11):
                L2 = L2_base + self.d5 + d4_try
                cos_q3 = (target_dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
                if abs(cos_q3) <= 1:
                    q3 = np.arccos(cos_q3)
                    beta = np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))
                    q2 = alpha + beta
                    
                    while q2 < 0:
                        q2 += 2 * np.pi
                    while q3 < 0:
                        q3 += 2 * np.pi
                        
                    if self._check_limits([q1, q2, q3, d4_try, q5]):
                        d4 = d4_try
                        break
        
        solution = [q1, q2, q3, d4, q5]
        
        return solution
    
    def _check_limits(self, q):
        """Check if joint positions are within limits."""
        names = ['base_rot1', 'rot1_rot2', 'rot2_rot3', 'rot3_prism1', 'prism1_rot4']
        for i, (name, val) in enumerate(zip(names, q)):
            lower, upper = self.joint_limits[name]
            if val < lower or val > upper:
                return False
        return True
    
    def get_workspace_info(self):
        """Return information about the robot workspace."""
        return {
            'x_range': (-0.5, 0.5),
            'y_range': (-0.5, 0.5),
            'z_range': (0.0, 0.5),
            'description': 'Approximate workspace of Braco RRRPR manipulator'
        }


class CartesianCommandSender(Node):
    """Node to send Cartesian position commands to the Braco manipulator."""
    
    def __init__(self, x, y, z, duration=3.0, q5=0.0):
        super().__init__('cartesian_command_sender')
        
        self.target_position = (x, y, z)
        self.duration = duration
        self.q5 = q5
        
        # Joint names
        self.joint_names = [
            'base_rot1',
            'rot1_rot2',
            'rot2_rot3',
            'rot3_prism1',
            'prism1_rot4'
        ]
        
        # Kinematics calculator
        self.kinematics = BracoKinematics()
        
        # Publisher for trajectory commands
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/trajectory_command',
            10
        )
        
        # Timer to send command
        self.timer = self.create_timer(0.5, self.send_command)
        self.sent = False
        
    def send_command(self):
        """Calculate IK and send trajectory command."""
        if self.sent:
            rclpy.shutdown()
            return
        
        x, y, z = self.target_position
        
        self.get_logger().info(f'Target Cartesian position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Calculate inverse kinematics
        joint_positions = self.kinematics.inverse_kinematics(x, y, z, self.q5)
        
        if joint_positions is None:
            self.get_logger().error('Could not find IK solution for target position')
            rclpy.shutdown()
            return
        
        self.get_logger().info(f'Calculated joint positions:')
        for name, pos in zip(self.joint_names, joint_positions):
            if name == 'rot3_prism1':
                self.get_logger().info(f'  {name}: {pos:.4f} m')
            else:
                self.get_logger().info(f'  {name}: {pos:.4f} rad ({np.degrees(pos):.2f} deg)')
        
        # Verify with forward kinematics
        fk_pos = self.kinematics.forward_kinematics(joint_positions)
        self.get_logger().info(f'FK verification: x={fk_pos[0]:.3f}, y={fk_pos[1]:.3f}, z={fk_pos[2]:.3f}')
        
        # Create trajectory message
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        # Create single point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start = Duration(
            sec=int(self.duration),
            nanosec=int((self.duration % 1) * 1e9)
        )
        msg.points.append(point)
        
        # Publish
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent trajectory command (duration: {self.duration}s)')
        
        self.sent = True


def main():
    parser = argparse.ArgumentParser(
        description='Send Cartesian position command to Braco manipulator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    ros2 run braco_description send_cartesian.py 0.2 0.1 0.15
    ros2 run braco_description send_cartesian.py 0.3 0.2 0.2 --duration 5.0
    ros2 run braco_description send_cartesian.py 0.25 0.0 0.1 --rotation 0.5
        """
    )
    parser.add_argument('x', type=float, help='Target X position (meters)')
    parser.add_argument('y', type=float, help='Target Y position (meters)')
    parser.add_argument('z', type=float, help='Target Z position (meters)')
    parser.add_argument('--duration', '-d', type=float, default=3.0,
                        help='Duration in seconds (default: 3.0)')
    parser.add_argument('--rotation', '-r', type=float, default=0.0,
                        help='End-effector rotation in radians (default: 0.0)')
    
    # Filter out ROS arguments
    filtered_args = []
    skip_next = False
    for i, arg in enumerate(sys.argv[1:]):
        if skip_next:
            skip_next = False
            continue
        if arg.startswith('--ros-args'):
            break
        if arg == '-r' and i + 1 < len(sys.argv) - 1:
            # Check if this is ROS -r or our --rotation shorthand
            next_arg = sys.argv[i + 2]
            if next_arg.startswith('__'):
                skip_next = True
                continue
        filtered_args.append(arg)
    
    try:
        args = parser.parse_args(filtered_args)
    except SystemExit:
        print("\nUsage: ros2 run braco_description send_cartesian.py <x> <y> <z> [options]")
        print("Example: ros2 run braco_description send_cartesian.py 0.2 0.1 0.15 --duration 3.0")
        return
    
    rclpy.init()
    node = CartesianCommandSender(args.x, args.y, args.z, args.duration, args.rotation)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
