#!/usr/bin/env python3
"""
test_disturbance_client.py

Python client to test the disturbance service
Usage: rosrun manipulator_disturbance_control test_disturbance_client.py [fx] [fy] [fz]

Author: Manipulator Test Team
"""

import rospy
import sys
from manipulator_msgs.srv import ApplyDisturbance, ApplyDisturbanceRequest

def main():
    rospy.init_node('test_disturbance_client_py')
    
    # Parse command line arguments
    if len(sys.argv) >= 4:
        fx = float(sys.argv[1])
        fy = float(sys.argv[2])
        fz = float(sys.argv[3])
    else:
        rospy.loginfo("Usage: rosrun manipulator_disturbance_control test_disturbance_client.py <fx> <fy> <fz>")
        rospy.loginfo("Using default values: fx=50.0 fy=0.0 fz=0.0")
        fx, fy, fz = 50.0, 0.0, 0.0
    
    # Wait for service
    service_name = '/apply_disturbance'
    rospy.loginfo(f"Waiting for {service_name} service...")
    rospy.wait_for_service(service_name)
    
    try:
        # Create service proxy
        apply_disturbance = rospy.ServiceProxy(service_name, ApplyDisturbance)
        
        # Create request
        req = ApplyDisturbanceRequest()
        req.force.x = fx
        req.force.y = fy
        req.force.z = fz
        
        rospy.loginfo(f"Calling service with force: [{fx:.2f}, {fy:.2f}, {fz:.2f}] N")
        
        # Call service
        response = apply_disturbance(req)
        
        if response.success:
            rospy.loginfo(f"SUCCESS: {response.message}")
        else:
            rospy.logerr(f"FAILED: {response.message}")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
