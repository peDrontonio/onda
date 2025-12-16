/*
 * test_disturbance_client.cpp
 * 
 * Simple client to test the disturbance service
 * Usage: ros2 run manipulator_disturbance_control test_disturbance_client <fx> <fy> <fz>
 * 
 * Author: Manipulator Test Team
 * Converted to ROS 2: December 2025
 */

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <manipulator_msgs/srv/apply_disturbance.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = rclcpp::Node::make_shared("test_disturbance_client");
  
  // Parse command line arguments
  double fx = 0.0, fy = 0.0, fz = 0.0;
  
  if (argc >= 4) {
    fx = std::atof(argv[1]);
    fy = std::atof(argv[2]);
    fz = std::atof(argv[3]);
  } else {
    RCLCPP_INFO(node->get_logger(), "Usage: ros2 run manipulator_disturbance_control test_disturbance_client <fx> <fy> <fz>");
    RCLCPP_INFO(node->get_logger(), "Using default values: fx=50.0 fy=0.0 fz=0.0");
    fx = 50.0;
  }
  
  // Create service client
  auto client = node->create_client<manipulator_msgs::srv::ApplyDisturbance>(
    "/apply_disturbance"
  );
  
  RCLCPP_INFO(node->get_logger(), "Waiting for /apply_disturbance service...");
  
  if (!client->wait_for_service(10s)) {
    RCLCPP_ERROR(node->get_logger(), "Service not available after 10 seconds");
    rclcpp::shutdown();
    return 1;
  }
  
  // Prepare service request
  auto request = std::make_shared<manipulator_msgs::srv::ApplyDisturbance::Request>();
  request->force.x = fx;
  request->force.y = fy;
  request->force.z = fz;
  
  RCLCPP_INFO(node->get_logger(), "Calling service with force: [%.2f, %.2f, %.2f] N", fx, fy, fz);
  
  // Call service
  auto result_future = client->async_send_request(request);
  
  // Wait for result
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success) {
      RCLCPP_INFO(node->get_logger(), "SUCCESS: %s", result->message.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "FAILED: %s", result->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service /apply_disturbance");
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
