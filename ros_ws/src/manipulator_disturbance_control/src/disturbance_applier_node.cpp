/*
 * disturbance_applier_node.cpp
 * 
 * Node that provides a service to apply/remove external forces on the manipulator
 * Uses Gazebo's ApplyBodyWrench service to simulate disturbances
 * 
 * Author: Manipulator Test Team
 * Date: November 2025
 * Converted to ROS 2: December 2025
 */

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/apply_body_wrench.hpp>
#include <manipulator_msgs/srv/apply_disturbance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DisturbanceApplier : public rclcpp::Node {
private:
  rclcpp::Service<manipulator_msgs::srv::ApplyDisturbance>::SharedPtr disturbance_service_;
  rclcpp::Client<gazebo_msgs::srv::ApplyBodyWrench>::SharedPtr gazebo_wrench_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  bool disturbance_active_;
  geometry_msgs::msg::Vector3 current_force_;
  std::string link_name_;
  std::string reference_frame_;
  double force_duration_;
  int marker_id_;
  
public:
  DisturbanceApplier() : Node("disturbance_applier_node"), disturbance_active_(false), marker_id_(0) {
    // Declare and load parameters
    this->declare_parameter("disturbance_link", "manipulator::wrist_3_link");
    this->declare_parameter("reference_frame", "world");
    this->declare_parameter("force_duration", 100.0);
    
    this->get_parameter("disturbance_link", link_name_);
    this->get_parameter("reference_frame", reference_frame_);
    this->get_parameter("force_duration", force_duration_);
    
    // Service server (our custom service)
    disturbance_service_ = this->create_service<manipulator_msgs::srv::ApplyDisturbance>(
      "/apply_disturbance",
      std::bind(&DisturbanceApplier::disturbanceCallback, this, 
                std::placeholders::_1, std::placeholders::_2)
    );
    
    // Service client (Gazebo's service)
    gazebo_wrench_client_ = this->create_client<gazebo_msgs::srv::ApplyBodyWrench>(
      "/apply_body_wrench"
    );
    
    // Publisher for force visualization
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/disturbance_marker", 10
    );
    
    RCLCPP_INFO(this->get_logger(), "==============================================");
    RCLCPP_INFO(this->get_logger(), "Disturbance Applier Node Initialized");
    RCLCPP_INFO(this->get_logger(), "  Target Link: %s", link_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Reference Frame: %s", reference_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Service: /apply_disturbance");
    RCLCPP_INFO(this->get_logger(), "==============================================");
  }
  
  void disturbanceCallback(
    const std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance::Request> request,
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance::Response> response
  ) {
    // Toggle disturbance state
    disturbance_active_ = !disturbance_active_;
    
    if (disturbance_active_) {
      // Store requested force
      current_force_ = request->force;
      
      RCLCPP_INFO(this->get_logger(), "Applying disturbance force: [%.2f, %.2f, %.2f] N", 
                  request->force.x, request->force.y, request->force.z);
      
      // Apply force in Gazebo
      if (applyForceInGazebo(current_force_)) {
        response->success = true;
        response->message = "Disturbance applied successfully";
        
        // Publish visualization marker
        publishForceMarker(current_force_, true);
      } else {
        response->success = false;
        response->message = "Failed to apply disturbance in Gazebo";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        disturbance_active_ = false;
      }
    } else {
      // Remove force (apply zero force)
      geometry_msgs::msg::Vector3 zero_force;
      zero_force.x = zero_force.y = zero_force.z = 0.0;
      
      RCLCPP_INFO(this->get_logger(), "Removing disturbance force");
      
      if (applyForceInGazebo(zero_force)) {
        response->success = true;
        response->message = "Disturbance removed successfully";
        
        // Remove visualization marker
        publishForceMarker(zero_force, false);
      } else {
        response->success = false;
        response->message = "Failed to remove disturbance";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      }
    }
  }
  
  bool applyForceInGazebo(const geometry_msgs::msg::Vector3& force) {
    // Wait for service to be available
    if (!gazebo_wrench_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Gazebo ApplyBodyWrench service not available");
      return false;
    }
    
    auto request = std::make_shared<gazebo_msgs::srv::ApplyBodyWrench::Request>();
    
    request->body_name = link_name_;
    request->reference_frame = reference_frame_;
    request->reference_point.x = 0.0;
    request->reference_point.y = 0.0;
    request->reference_point.z = 0.0;
    request->wrench.force = force;
    request->wrench.torque.x = 0.0;
    request->wrench.torque.y = 0.0;
    request->wrench.torque.z = 0.0;
    request->start_time = this->now();
    request->duration = rclcpp::Duration::from_seconds(force_duration_);
    
    auto result_future = gazebo_wrench_client_->async_send_request(request);
    
    // Wait for result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      if (result->success) {
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Gazebo service returned failure: %s", 
                     result->status_message.c_str());
        return false;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call Gazebo ApplyBodyWrench service");
      return false;
    }
  }
  
  void publishForceMarker(const geometry_msgs::msg::Vector3& force, bool active) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = link_name_.substr(link_name_.find("::") + 2);  // Remove model name
    marker.header.stamp = this->now();
    marker.ns = "disturbance_force";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = active ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETEALL;
    
    if (active) {
      // Arrow start point (at link origin)
      geometry_msgs::msg::Point start;
      start.x = 0.0;
      start.y = 0.0;
      start.z = 0.0;
      
      // Arrow end point (scaled force vector)
      geometry_msgs::msg::Point end;
      double scale = 0.01;  // Scale for visualization (1N = 1cm)
      end.x = force.x * scale;
      end.y = force.y * scale;
      end.z = force.z * scale;
      
      marker.points.push_back(start);
      marker.points.push_back(end);
      
      // Arrow appearance
      marker.scale.x = 0.01;   // Shaft diameter
      marker.scale.y = 0.02;   // Head diameter
      marker.scale.z = 0.03;   // Head length
      
      // Red color
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Never expire
    }
    
    marker_pub_->publish(marker);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<DisturbanceApplier>();
  
  RCLCPP_INFO(node->get_logger(), "Disturbance Applier Node running... Press Ctrl+C to exit");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
