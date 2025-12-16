#!/usr/bin/env python3
"""
Initialize manipulator to a safe home position
Publishes initial joint positions to prevent self-collision
"""

import rospy
from std_msgs.msg import Float64
import time

def initialize_manipulator():
    """Send initial position commands to all joints"""
    rospy.init_node('position_initializer', anonymous=True)
    
    # Wait for controllers to be ready
    rospy.loginfo("Waiting 3 seconds for controllers to initialize...")
    rospy.sleep(3.0)
    
    # Create publishers for each joint
    publishers = {
        'joint_1': rospy.Publisher('/manipulator/joint_1_position_controller/command', Float64, queue_size=10),
        'joint_2': rospy.Publisher('/manipulator/joint_2_position_controller/command', Float64, queue_size=10),
        'joint_3': rospy.Publisher('/manipulator/joint_3_position_controller/command', Float64, queue_size=10),
        'joint_4': rospy.Publisher('/manipulator/joint_4_position_controller/command', Float64, queue_size=10),
        'joint_5': rospy.Publisher('/manipulator/joint_5_position_controller/command', Float64, queue_size=10),
        'joint_6': rospy.Publisher('/manipulator/joint_6_position_controller/command', Float64, queue_size=10),
    }
    
    # Wait for publishers to establish connections
    rospy.sleep(1.0)
    
    # Safe home position (all joints at 0.0 radians)
    home_position = {
        'joint_1': 0.0,
        'joint_2': 0.0,
        'joint_3': 0.0,
        'joint_4': 0.0,
        'joint_5': 0.0,
        'joint_6': 0.0,
    }
    
    rospy.loginfo("Sending home position commands...")
    
    # Publish home position multiple times to ensure it's received
    for _ in range(10):
        for joint_name, position in home_position.items():
            publishers[joint_name].publish(Float64(position))
        rospy.sleep(0.1)
    
    rospy.loginfo("✓ Manipulator initialized to home position (all zeros)")
    rospy.loginfo("Ready for trajectory commands!")

if __name__ == '__main__':
    try:
        initialize_manipulator()
    except rospy.ROSInterruptException:
        pass
