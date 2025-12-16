#!/usr/bin/env python3
"""
trajectory_with_disturbance.py

Execute trajectory while applying disturbances at specific moments
Tests the manipulator's robustness under external forces

Author: Manipulator Test Team
Date: November 2025
"""

import rospy
import threading
import time
from manipulator_msgs.srv import ApplyDisturbance, ApplyDisturbanceRequest
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np


class TrajectoryWithDisturbance:
    def __init__(self):
        rospy.init_node('trajectory_with_disturbance', anonymous=True)
        
        # Service client for disturbances
        self.disturbance_service = '/apply_disturbance'
        rospy.loginfo(f"Waiting for {self.disturbance_service} service...")
        rospy.wait_for_service(self.disturbance_service)
        self.disturbance_client = rospy.ServiceProxy(self.disturbance_service, ApplyDisturbance)
        
        # Joint publishers
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_pubs = {}
        for joint in self.joint_names:
            topic = f'/manipulator/{joint}_position_controller/command'
            self.joint_pubs[joint] = rospy.Publisher(topic, Float64, queue_size=10)
        
        # Joint state subscriber
        self.current_positions = [0.0] * 6
        self.state_lock = threading.Lock()
        rospy.Subscriber('/manipulator/joint_states', JointState, self.joint_state_callback)
        
        # Disturbance state
        self.disturbance_active = False
        
        rospy.loginfo("Trajectory with Disturbance node initialized")
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        with self.state_lock:
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_positions[i] = msg.position[idx]
    
    def publish_positions(self, positions):
        """Publish joint positions"""
        for i, joint_name in enumerate(self.joint_names):
            msg = Float64()
            msg.data = positions[i]
            self.joint_pubs[joint_name].publish(msg)
    
    def apply_disturbance(self, fx, fy, fz):
        """Apply disturbance force"""
        try:
            req = ApplyDisturbanceRequest()
            req.force.x = fx
            req.force.y = fy
            req.force.z = fz
            
            response = self.disturbance_client(req)
            
            if response.success:
                self.disturbance_active = not self.disturbance_active
                status = "APPLIED" if self.disturbance_active else "REMOVED"
                rospy.loginfo(f"Disturbance {status}: [{fx:.1f}, {fy:.1f}, {fz:.1f}] N")
                return True
            else:
                rospy.logerr(f"Failed to apply disturbance: {response.message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def interpolate(self, start, end, steps):
        """Generate smooth interpolation between positions"""
        trajectory = []
        for i in range(steps + 1):
            t = i / float(steps)
            t_smooth = 3*t**2 - 2*t**3  # Smoothstep
            
            waypoint = []
            for j in range(len(start)):
                pos = start[j] + t_smooth * (end[j] - start[j])
                waypoint.append(pos)
            trajectory.append(waypoint)
        
        return trajectory
    
    def move_to(self, target, duration, rate_hz=50):
        """Move to target position"""
        steps = int(duration * rate_hz)
        with self.state_lock:
            current = list(self.current_positions)
        
        trajectory = self.interpolate(current, target, steps)
        
        rate = rospy.Rate(rate_hz)
        for waypoint in trajectory:
            if rospy.is_shutdown():
                break
            self.publish_positions(waypoint)
            rate.sleep()
    
    def scenario_1_single_disturbance(self):
        """
        Scenario 1: Apply disturbance midway through movement
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("SCENARIO 1: Single Disturbance During Movement")
        rospy.loginfo("=" * 60)
        
        # Define positions
        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target = [0.8, -0.5, 0.8, 0.0, 0.0, 0.0]
        
        # Go to home
        rospy.loginfo("Moving to home position...")
        self.move_to(home, duration=3.0)
        rospy.sleep(1.0)
        
        # Start movement
        rospy.loginfo("Starting movement to target...")
        
        # Start moving in a thread
        move_thread = threading.Thread(target=lambda: self.move_to(target, duration=6.0))
        move_thread.start()
        
        # Apply disturbance after 3 seconds (midway)
        rospy.sleep(3.0)
        rospy.loginfo(">>> APPLYING DISTURBANCE <<<")
        self.apply_disturbance(80.0, 0.0, 0.0)
        
        # Wait for movement to complete
        move_thread.join()
        rospy.sleep(2.0)
        
        # Remove disturbance
        rospy.loginfo(">>> REMOVING DISTURBANCE <<<")
        self.apply_disturbance(80.0, 0.0, 0.0)
        
        rospy.loginfo("Scenario 1 completed\n")
    
    def scenario_2_multiple_disturbances(self):
        """
        Scenario 2: Multiple disturbances during circular trajectory
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("SCENARIO 2: Multiple Disturbances on Circular Path")
        rospy.loginfo("=" * 60)
        
        # Go to starting position
        start_pos = [0.4, -0.5, 0.75, 0.0, 0.0, 0.0]
        rospy.loginfo("Moving to start position...")
        self.move_to(start_pos, duration=3.0)
        rospy.sleep(1.0)
        
        # Execute circular trajectory with disturbances
        n_points = 16
        radius = 0.4
        
        for i in range(n_points + 1):
            if rospy.is_shutdown():
                break
            
            angle = 2 * np.pi * i / n_points
            
            j1 = radius * np.cos(angle)
            j2 = -0.5 + radius * np.sin(angle) * 0.5
            j3 = 0.75
            
            target = [j1, j2, j3, 0.0, 0.0, 0.0]
            
            # Move to next point
            self.move_to(target, duration=1.0)
            
            # Apply disturbance at specific points
            if i in [4, 8, 12]:  # 3 disturbances during circle
                force_direction = [60.0 * np.cos(angle), 60.0 * np.sin(angle), 0.0]
                rospy.loginfo(f">>> DISTURBANCE at point {i} <<<")
                self.apply_disturbance(*force_direction)
                rospy.sleep(1.5)
                self.apply_disturbance(*force_direction)  # Remove
        
        rospy.loginfo("Scenario 2 completed\n")
    
    def scenario_3_continuous_disturbance(self):
        """
        Scenario 3: Trajectory with continuous varying disturbance
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("SCENARIO 3: Continuous Varying Disturbance")
        rospy.loginfo("=" * 60)
        
        # Square trajectory waypoints
        waypoints = [
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
            [0.6, -0.5, 0.5, 0.0, 0.0, 0.0],
            [0.6, -0.5, 1.0, 0.0, 0.0, 0.0],
            [0.0, -0.5, 1.0, 0.0, 0.0, 0.0],
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
        ]
        
        # Apply initial disturbance
        rospy.loginfo(">>> APPLYING INITIAL DISTURBANCE <<<")
        self.apply_disturbance(50.0, 0.0, 0.0)
        rospy.sleep(1.0)
        
        # Execute square with disturbance on
        for i, waypoint in enumerate(waypoints):
            rospy.loginfo(f"Moving to corner {i+1}/5")
            self.move_to(waypoint, duration=2.5)
            rospy.sleep(0.5)
            
            # Change disturbance direction at each corner
            if i < len(waypoints) - 1:
                # Remove old
                self.apply_disturbance(50.0, 0.0, 0.0)
                rospy.sleep(0.2)
                
                # Apply new direction
                forces = [
                    [0.0, 50.0, 0.0],   # +Y
                    [0.0, 0.0, 40.0],   # +Z
                    [-50.0, 0.0, 0.0],  # -X
                    [0.0, -50.0, 0.0],  # -Y
                ]
                rospy.loginfo(f">>> CHANGING DISTURBANCE DIRECTION <<<")
                self.apply_disturbance(*forces[i])
                rospy.sleep(0.3)
        
        # Remove final disturbance
        rospy.loginfo(">>> REMOVING FINAL DISTURBANCE <<<")
        self.apply_disturbance(0.0, -50.0, 0.0)
        
        rospy.loginfo("Scenario 3 completed\n")
    
    def scenario_4_random_disturbances(self):
        """
        Scenario 4: Random disturbances during wave trajectory
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("SCENARIO 4: Random Disturbances on Wave Path")
        rospy.loginfo("=" * 60)
        
        n_points = 24
        
        for i in range(n_points):
            if rospy.is_shutdown():
                break
            
            # Generate wave trajectory
            t = i / float(n_points) * 4 * np.pi
            j1 = 0.5 * np.sin(t)
            j2 = -0.5 + 0.2 * np.cos(2*t)
            j3 = 0.75 + 0.3 * np.sin(t)
            
            target = [j1, j2, j3, 0.0, 0.0, 0.0]
            self.move_to(target, duration=0.8)
            
            # Random chance of disturbance
            if np.random.random() < 0.3:  # 30% chance
                fx = np.random.uniform(-60, 60)
                fy = np.random.uniform(-60, 60)
                fz = np.random.uniform(-30, 30)
                
                rospy.loginfo(f">>> RANDOM DISTURBANCE: [{fx:.1f}, {fy:.1f}, {fz:.1f}] N <<<")
                self.apply_disturbance(fx, fy, fz)
                rospy.sleep(0.8)
                self.apply_disturbance(fx, fy, fz)  # Remove
        
        rospy.loginfo("Scenario 4 completed\n")
    
    def run_all_scenarios(self):
        """Run all test scenarios"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("STARTING COMPLETE DISTURBANCE TEST SUITE")
        rospy.loginfo("=" * 60 + "\n")
        
        rospy.sleep(2.0)
        
        try:
            self.scenario_1_single_disturbance()
            rospy.sleep(3.0)
            
            self.scenario_2_multiple_disturbances()
            rospy.sleep(3.0)
            
            self.scenario_3_continuous_disturbance()
            rospy.sleep(3.0)
            
            self.scenario_4_random_disturbances()
            
        except Exception as e:
            rospy.logerr(f"Error during scenario execution: {e}")
        
        # Ensure disturbance is off and return home
        if self.disturbance_active:
            rospy.loginfo("Ensuring disturbance is removed...")
            self.apply_disturbance(0.0, 0.0, 0.0)
        
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("ALL SCENARIOS COMPLETED!")
        rospy.loginfo("=" * 60 + "\n")


def main():
    try:
        controller = TrajectoryWithDisturbance()
        
        # Wait for everything to initialize
        rospy.loginfo("Waiting for system initialization...")
        rospy.sleep(3.0)
        
        # Get parameters
        scenario = rospy.get_param('~scenario', 'all')
        
        if scenario == 'all':
            controller.run_all_scenarios()
        elif scenario == '1':
            controller.scenario_1_single_disturbance()
        elif scenario == '2':
            controller.scenario_2_multiple_disturbances()
        elif scenario == '3':
            controller.scenario_3_continuous_disturbance()
        elif scenario == '4':
            controller.scenario_4_random_disturbances()
        else:
            rospy.logerr(f"Unknown scenario: {scenario}")
        
        rospy.loginfo("Test completed. Node staying alive...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
