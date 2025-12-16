#!/usr/bin/env python3
"""
Trajectory Command Sender for Braco Manipulator

A simple node to send trajectory commands to the Braco robot.
Can be used to test different positions and trajectories.

Usage:
    ros2 run braco_description send_trajectory.py 1.0 0.5 0.5 0.05 0.0
    
Or with duration:
    ros2 run braco_description send_trajectory.py 1.0 0.5 0.5 0.05 0.0 --duration 3.0

Author: Pedro Antonio
Date: 2025-12-16
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import argparse


class TrajectoryCommandSender(Node):
    """Simple node to send single trajectory commands."""
    
    def __init__(self, positions, duration=3.0):
        super().__init__('trajectory_command_sender')
        
        # Joint names
        self.joint_names = [
            'base_rot1',
            'rot1_rot2',
            'rot2_rot3',
            'rot3_prism1',
            'prism1_rot4'
        ]
        
        self.positions = positions
        self.duration = duration
        
        # Publisher for trajectory commands
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/trajectory_command',
            10
        )
        
        # Timer to send command after initialization
        self.timer = self.create_timer(0.5, self.send_command)
        self.sent = False
        
    def send_command(self):
        """Send trajectory command."""
        if self.sent:
            rclpy.shutdown()
            return
            
        if len(self.positions) != len(self.joint_names):
            self.get_logger().error(
                f'Expected {len(self.joint_names)} positions, got {len(self.positions)}'
            )
            rclpy.shutdown()
            return
        
        # Create trajectory message
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        # Create single point
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.velocities = [0.0] * len(self.positions)
        point.time_from_start = Duration(
            sec=int(self.duration),
            nanosec=int((self.duration % 1) * 1e9)
        )
        msg.points.append(point)
        
        # Publish
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent trajectory to positions: {self.positions}')
        self.get_logger().info(f'Duration: {self.duration} seconds')
        
        self.sent = True


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Send trajectory command to Braco manipulator')
    parser.add_argument('positions', nargs=5, type=float, 
                        help='Joint positions: base_rot1 rot1_rot2 rot2_rot3 rot3_prism1 prism1_rot4')
    parser.add_argument('--duration', '-d', type=float, default=3.0,
                        help='Duration in seconds (default: 3.0)')
    
    # Filter out ROS arguments
    args_to_parse = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args') and arg != '-r']
    
    try:
        args = parser.parse_args(args_to_parse)
    except SystemExit:
        print("\nUsage: ros2 run braco_description send_trajectory.py <j1> <j2> <j3> <j4> <j5> [--duration D]")
        print("Example: ros2 run braco_description send_trajectory.py 1.0 0.5 0.5 0.05 0.0 --duration 3.0")
        return
    
    rclpy.init()
    node = TrajectoryCommandSender(args.positions, args.duration)
    
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
