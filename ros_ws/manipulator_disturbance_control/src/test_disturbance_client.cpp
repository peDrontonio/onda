/*
 * test_disturbance_client.cpp
 * 
 * Simple client to test the disturbance service
 * Usage: rosrun manipulator_disturbance_control test_disturbance_client <fx> <fy> <fz>
 * 
 * Author: Manipulator Test Team
 */

#include <ros/ros.h>
#include <manipulator_msgs/ApplyDisturbance.h>
#include <sstream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_disturbance_client");
  ros::NodeHandle nh;
  
  // Parse command line arguments
  double fx = 0.0, fy = 0.0, fz = 0.0;
  
  if (argc >= 4) {
    fx = std::atof(argv[1]);
    fy = std::atof(argv[2]);
    fz = std::atof(argv[3]);
  } else {
    ROS_INFO("Usage: rosrun manipulator_disturbance_control test_disturbance_client <fx> <fy> <fz>");
    ROS_INFO("Using default values: fx=50.0 fy=0.0 fz=0.0");
    fx = 50.0;
  }
  
  // Create service client
  ros::ServiceClient client = nh.serviceClient<manipulator_msgs::ApplyDisturbance>(
    "/apply_disturbance"
  );
  
  ROS_INFO("Waiting for /apply_disturbance service...");
  client.waitForExistence();
  
  // Prepare service request
  manipulator_msgs::ApplyDisturbance srv;
  srv.request.force.x = fx;
  srv.request.force.y = fy;
  srv.request.force.z = fz;
  
  ROS_INFO("Calling service with force: [%.2f, %.2f, %.2f] N", fx, fy, fz);
  
  // Call service
  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("SUCCESS: %s", srv.response.message.c_str());
    } else {
      ROS_ERROR("FAILED: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("Failed to call service /apply_disturbance");
    return 1;
  }
  
  return 0;
}
