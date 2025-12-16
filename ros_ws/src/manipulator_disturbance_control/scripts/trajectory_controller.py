#!/usr/bin/env python3
"""
trajectory_controller.py

Node that moves the manipulator through predefined trajectories
Can be used in combination with disturbance testing

Author: Manipulator Test Team
Date: November 2025
"""

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading


class TrajectoryController:
    def __init__(self):
        rospy.init_node('trajectory_controller', anonymous=True)
        
        # Joint names
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        # Publishers for each joint
        self.joint_pubs = {}
        for joint in self.joint_names:
            topic = f'/manipulator/{joint}_position_controller/command'
            self.joint_pubs[joint] = rospy.Publisher(topic, Float64, queue_size=10)
        
        # Subscriber to joint states
        self.current_joint_states = None
        self.joint_state_lock = threading.Lock()
        rospy.Subscriber('/manipulator/joint_states', JointState, self.joint_state_callback)
        
        # Parameters
        self.rate_hz = rospy.get_param('~rate', 50)  # Control rate
        self.rate = rospy.Rate(self.rate_hz)
        
        # Home position (all joints at 0)
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        rospy.loginfo("Trajectory Controller initialized")
        rospy.loginfo(f"Control rate: {self.rate_hz} Hz")
        
    def joint_state_callback(self, msg):
        """Callback to store current joint states"""
        with self.joint_state_lock:
            self.current_joint_states = msg
    
    def get_current_positions(self):
        """Get current joint positions"""
        with self.joint_state_lock:
            if self.current_joint_states is None:
                return self.home_position
            
            positions = []
            for joint_name in self.joint_names:
                try:
                    idx = self.current_joint_states.name.index(joint_name)
                    positions.append(self.current_joint_states.position[idx])
                except ValueError:
                    positions.append(0.0)
            return positions
    
    def publish_joint_positions(self, positions):
        """Publish position commands to all joints"""
        for i, joint_name in enumerate(self.joint_names):
            msg = Float64()
            msg.data = positions[i]
            self.joint_pubs[joint_name].publish(msg)
    
    def interpolate_trajectory(self, start_pos, end_pos, duration, steps=None):
        """
        Generate interpolated trajectory between two positions
        
        Args:
            start_pos: Starting joint positions
            end_pos: Ending joint positions
            duration: Duration in seconds
            steps: Number of steps (if None, calculated from rate and duration)
        
        Returns:
            List of waypoints
        """
        if steps is None:
            steps = int(duration * self.rate_hz)
        
        waypoints = []
        for i in range(steps + 1):
            t = i / float(steps)  # Normalized time [0, 1]
            
            # Use smooth interpolation (cubic)
            t_smooth = 3*t**2 - 2*t**3  # Smoothstep function
            
            waypoint = []
            for j in range(len(start_pos)):
                pos = start_pos[j] + t_smooth * (end_pos[j] - start_pos[j])
                waypoint.append(pos)
            
            waypoints.append(waypoint)
        
        return waypoints
    
    def execute_trajectory(self, waypoints, wait=True):
        """
        Execute a trajectory given as a list of waypoints
        
        Args:
            waypoints: List of joint positions
            wait: If True, blocks until trajectory completes
        """
        rospy.loginfo(f"Executing trajectory with {len(waypoints)} waypoints")
        
        for i, waypoint in enumerate(waypoints):
            if rospy.is_shutdown():
                break
            
            self.publish_joint_positions(waypoint)
            
            if wait:
                self.rate.sleep()
        
        rospy.loginfo("Trajectory execution completed")
    
    def move_to_position(self, target_positions, duration=3.0):
        """
        Move to a specific joint configuration
        
        Args:
            target_positions: Target joint positions [j1, j2, j3, j4, j5, j6]
            duration: Time to reach position (seconds)
        """
        current_pos = self.get_current_positions()
        waypoints = self.interpolate_trajectory(current_pos, target_positions, duration)
        self.execute_trajectory(waypoints)
    
    def move_home(self, duration=3.0):
        """Move to home position (all joints at 0)"""
        rospy.loginfo("Moving to home position")
        self.move_to_position(self.home_position, duration)
    
    def execute_predefined_trajectory(self, trajectory_name):
        """
        Execute a predefined trajectory
        
        Available trajectories:
        - 'square': Square pattern in joint space
        - 'circle': Circular pattern
        - 'zigzag': Zig-zag pattern
        - 'wave': Wave pattern
        """
        rospy.loginfo(f"Executing predefined trajectory: {trajectory_name}")
        
        if trajectory_name == 'square':
            self._execute_square_trajectory()
        elif trajectory_name == 'circle':
            self._execute_circle_trajectory()
        elif trajectory_name == 'zigzag':
            self._execute_zigzag_trajectory()
        elif trajectory_name == 'wave':
            self._execute_wave_trajectory()
        else:
            rospy.logerr(f"Unknown trajectory: {trajectory_name}")
    
    def _execute_square_trajectory(self):
        """Execute a square trajectory (4 corners)"""
        rospy.loginfo("Executing SQUARE trajectory")
        
        corners = [
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],   # Corner 1
            [0.5, -0.5, 0.5, 0.0, 0.0, 0.0],   # Corner 2
            [0.5, -0.5, 1.0, 0.0, 0.0, 0.0],   # Corner 3
            [0.0, -0.5, 1.0, 0.0, 0.0, 0.0],   # Corner 4
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],   # Back to corner 1
        ]
        
        for corner in corners:
            self.move_to_position(corner, duration=2.0)
            rospy.sleep(0.5)  # Pause at each corner
    
    def _execute_circle_trajectory(self):
        """Execute a circular trajectory"""
        rospy.loginfo("Executing CIRCLE trajectory")
        
        n_points = 20
        radius = 0.4
        duration_per_point = 0.5
        
        for i in range(n_points + 1):
            angle = 2 * np.pi * i / n_points
            
            j1 = radius * np.cos(angle)
            j2 = -0.5 + radius * np.sin(angle) * 0.5
            j3 = 0.75
            
            position = [j1, j2, j3, 0.0, 0.0, 0.0]
            self.move_to_position(position, duration=duration_per_point)
    
    def _execute_zigzag_trajectory(self):
        """Execute a zig-zag trajectory"""
        rospy.loginfo("Executing ZIG-ZAG trajectory")
        
        points = [
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
            [0.5, -0.3, 0.8, 0.0, 0.0, 0.0],
            [-0.5, -0.3, 0.8, 0.0, 0.0, 0.0],
            [0.5, -0.7, 1.0, 0.0, 0.0, 0.0],
            [-0.5, -0.7, 1.0, 0.0, 0.0, 0.0],
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
        ]
        
        for point in points:
            self.move_to_position(point, duration=1.5)
            rospy.sleep(0.3)
    
    def _execute_wave_trajectory(self):
        """Execute a wave trajectory"""
        rospy.loginfo("Executing WAVE trajectory")
        
        n_points = 30
        duration_per_point = 0.3
        
        for i in range(n_points):
            t = i / float(n_points) * 4 * np.pi  # 2 full waves
            
            j1 = 0.5 * np.sin(t)
            j2 = -0.5 + 0.2 * np.cos(2*t)
            j3 = 0.75 + 0.3 * np.sin(t)
            
            position = [j1, j2, j3, 0.0, 0.0, 0.0]
            self.move_to_position(position, duration=duration_per_point)
    
    def continuous_loop(self, trajectory_name, num_loops=None):
        """
        Execute a trajectory continuously in a loop
        
        Args:
            trajectory_name: Name of the predefined trajectory
            num_loops: Number of loops (None for infinite)
        """
        loop_count = 0
        
        while not rospy.is_shutdown():
            if num_loops is not None and loop_count >= num_loops:
                break
            
            rospy.loginfo(f"Loop {loop_count + 1}/{num_loops if num_loops else 'inf'}")
            self.execute_predefined_trajectory(trajectory_name)
            
            loop_count += 1
            rospy.sleep(1.0)  # Pause between loops
        
        rospy.loginfo("Continuous loop finished")


def main():
    try:
        controller = TrajectoryController()
        
        # Wait for joint states
        rospy.loginfo("Waiting for joint states...")
        rospy.sleep(2.0)
        
        # Get parameters
        trajectory = rospy.get_param('~trajectory', 'square')
        num_loops = rospy.get_param('~loops', 1)
        continuous = rospy.get_param('~continuous', False)
        
        # Move to home first
        rospy.loginfo("Moving to home position...")
        controller.move_home(duration=3.0)
        rospy.sleep(1.0)
        
        # Execute trajectory
        if continuous or num_loops > 1:
            controller.continuous_loop(trajectory, num_loops if not continuous else None)
        else:
            controller.execute_predefined_trajectory(trajectory)
        
        rospy.loginfo("Trajectory execution completed. Node staying alive...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
