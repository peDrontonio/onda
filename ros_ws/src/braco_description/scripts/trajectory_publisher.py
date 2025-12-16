#!/usr/bin/env python3
"""
Trajectory Publisher Node for Braco Manipulator

This node publishes joint trajectory commands to move the robot arm.
It can be used to:
1. Send single trajectory goals
2. Execute predefined trajectory sequences
3. Receive trajectory commands via a topic

Usage:
    ros2 run braco_description trajectory_publisher

Topics:
    - Publishes to: /joint_trajectory_controller/joint_trajectory
    - Subscribes to: /trajectory_command (optional custom commands)

Author: Pedro Antonio
Date: 2025-12-16
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import numpy as np


class TrajectoryPublisher(Node):
    """
    Node for publishing joint trajectories to the Braco manipulator.
    """
    
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Joint names for the Braco manipulator
        self.joint_names = [
            'base_rot1',
            'rot1_rot2',
            'rot2_rot3',
            'rot3_prism1',
            'prism1_rot4'
        ]
        
        # Joint limits (from URDF)
        self.joint_limits = {
            'base_rot1': {'lower': 0.0, 'upper': 6.283185},
            'rot1_rot2': {'lower': 0.0, 'upper': 6.283185},
            'rot2_rot3': {'lower': 0.0, 'upper': 6.283185},
            'rot3_prism1': {'lower': 0.0, 'upper': 0.1},  # Prismatic joint in meters
            'prism1_rot4': {'lower': -1.047198, 'upper': 1.047198}
        }
        
        # Current joint positions (initialized to zeros)
        self.current_positions = [0.0] * len(self.joint_names)
        
        # Publisher for JointTrajectory messages
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Publisher for JointState (for visualization without ros2_control)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Action client for FollowJointTrajectory (for use with ros2_control)
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Subscriber for receiving trajectory commands
        self.trajectory_cmd_sub = self.create_subscription(
            JointTrajectory,
            '/trajectory_command',
            self.trajectory_command_callback,
            10
        )
        
        # Timer for publishing joint states (when in standalone mode)
        self.standalone_mode = True
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50 Hz
        
        self.get_logger().info('Trajectory Publisher initialized')
        self.get_logger().info(f'Joints: {self.joint_names}')
        self.get_logger().info('Ready to receive trajectory commands on /trajectory_command')
        self.get_logger().info('Or call send_trajectory() to send predefined trajectories')
        
    def publish_joint_states(self):
        """Publish current joint states for visualization."""
        if self.standalone_mode:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = self.current_positions
            msg.velocity = [0.0] * len(self.joint_names)
            msg.effort = [0.0] * len(self.joint_names)
            self.joint_state_pub.publish(msg)
    
    def trajectory_command_callback(self, msg: JointTrajectory):
        """Handle incoming trajectory commands."""
        self.get_logger().info('Received trajectory command')
        
        # Extract final positions from trajectory and set as target
        if msg.points:
            final_point = msg.points[-1]
            if len(final_point.positions) == len(self.joint_names):
                # Update target positions for interpolation
                if hasattr(self, 'target_positions'):
                    self.target_positions = list(final_point.positions)
                else:
                    self.current_positions = list(final_point.positions)
                self.get_logger().info(f'Target positions set to: {list(final_point.positions)}')
        
        # Also publish to trajectory controller (for ros2_control if available)
        self.trajectory_pub.publish(msg)
    
    def create_trajectory_point(self, positions: list, time_from_start_sec: float) -> JointTrajectoryPoint:
        """Create a JointTrajectoryPoint from positions and time."""
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(time_from_start_sec),
            nanosec=int((time_from_start_sec % 1) * 1e9)
        )
        return point
    
    def send_trajectory(self, waypoints: list, duration_per_point: float = 2.0):
        """
        Send a trajectory with multiple waypoints.
        
        Args:
            waypoints: List of joint positions lists, e.g., [[0,0,0,0,0], [1,0,0,0,0]]
            duration_per_point: Time in seconds between each waypoint
        """
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        for i, positions in enumerate(waypoints):
            if len(positions) != len(self.joint_names):
                self.get_logger().error(
                    f'Waypoint {i} has {len(positions)} positions, expected {len(self.joint_names)}'
                )
                return
            
            # Validate joint limits
            for j, (pos, name) in enumerate(zip(positions, self.joint_names)):
                limits = self.joint_limits[name]
                if pos < limits['lower'] or pos > limits['upper']:
                    self.get_logger().warn(
                        f'Position {pos} for {name} is outside limits [{limits["lower"]}, {limits["upper"]}]'
                    )
            
            time_from_start = (i + 1) * duration_per_point
            point = self.create_trajectory_point(positions, time_from_start)
            msg.points.append(point)
        
        self.get_logger().info(f'Sending trajectory with {len(waypoints)} waypoints')
        self.trajectory_pub.publish(msg)
        
        # Update current positions for visualization
        if waypoints:
            self.current_positions = list(waypoints[-1])
    
    def send_trajectory_action(self, waypoints: list, duration_per_point: float = 2.0):
        """
        Send trajectory via action (for ros2_control integration).
        
        Args:
            waypoints: List of joint positions lists
            duration_per_point: Time in seconds between each waypoint
        """
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, using topic instead')
            self.send_trajectory(waypoints, duration_per_point)
            return
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = self.joint_names
        
        for i, positions in enumerate(waypoints):
            time_from_start = (i + 1) * duration_per_point
            point = self.create_trajectory_point(positions, time_from_start)
            goal.trajectory.points.append(point)
        
        self.get_logger().info(f'Sending trajectory action with {len(waypoints)} waypoints')
        self._action_client.send_goal_async(goal)
    
    def go_to_position(self, positions: list, duration: float = 3.0):
        """
        Move to a single position.
        
        Args:
            positions: List of joint positions [j1, j2, j3, j4, j5]
            duration: Time to reach the position in seconds
        """
        self.send_trajectory([positions], duration)
    
    def go_home(self, duration: float = 3.0):
        """Move all joints to home position (zeros)."""
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('Moving to home position')
        self.go_to_position(home_position, duration)
    
    def execute_demo_trajectory(self):
        """Execute a demonstration trajectory."""
        self.get_logger().info('Executing demo trajectory...')
        
        waypoints = [
            [0.0, 0.0, 0.0, 0.0, 0.0],           # Home
            [1.0, 0.5, 0.5, 0.05, 0.0],          # Position 1
            [2.0, 1.0, 1.0, 0.08, 0.5],          # Position 2
            [3.0, 0.5, 1.5, 0.03, -0.5],         # Position 3
            [1.5, 0.8, 0.8, 0.05, 0.0],          # Position 4
            [0.0, 0.0, 0.0, 0.0, 0.0],           # Back to home
        ]
        
        self.send_trajectory(waypoints, duration_per_point=2.0)


class InteractiveTrajectoryPublisher(TrajectoryPublisher):
    """
    Extended trajectory publisher with interactive capabilities.
    Interpolates smoothly to target positions for visualization.
    """
    
    def __init__(self):
        super().__init__()
        
        # Target positions for interpolation
        self.target_positions = [0.0] * len(self.joint_names)
        
        # Interpolation parameters
        self.interpolation_speed = 0.5  # radians/meters per second
        
        # Override timer for smoother interpolation
        self.timer.cancel()
        self.timer = self.create_timer(0.02, self.update_and_publish)  # 50 Hz
        
    def update_and_publish(self):
        """Update positions via interpolation and publish."""
        dt = 0.02
        
        # Interpolate each joint towards target
        for i in range(len(self.joint_names)):
            diff = self.target_positions[i] - self.current_positions[i]
            if abs(diff) > 0.001:
                step = np.sign(diff) * min(abs(diff), self.interpolation_speed * dt)
                self.current_positions[i] += step
        
        # Publish joint states
        self.publish_joint_states()
    
    def set_target_position(self, positions: list):
        """Set target positions for smooth interpolation."""
        if len(positions) != len(self.joint_names):
            self.get_logger().error(f'Expected {len(self.joint_names)} positions, got {len(positions)}')
            return
        
        self.target_positions = list(positions)
        self.get_logger().info(f'Target set to: {positions}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create the trajectory publisher node
    node = InteractiveTrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
