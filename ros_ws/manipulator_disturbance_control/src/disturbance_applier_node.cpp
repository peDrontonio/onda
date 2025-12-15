/*
 * disturbance_applier_node.cpp
 * 
 * Node that provides a service to apply/remove external forces on the manipulator
 * Uses Gazebo's ApplyBodyWrench service to simulate disturbances
 * 
 * Author: Manipulator Test Team
 * Date: November 2025
 */

#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <manipulator_msgs/ApplyDisturbance.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class DisturbanceApplier {
private:
  ros::NodeHandle nh_;
  ros::ServiceServer disturbance_service_;
  ros::ServiceClient gazebo_wrench_client_;
  ros::Publisher marker_pub_;
  
  bool disturbance_active_;
  geometry_msgs::Vector3 current_force_;
  std::string link_name_;
  std::string reference_frame_;
  double force_duration_;
  int marker_id_;
  
public:
  DisturbanceApplier() : disturbance_active_(false), marker_id_(0) {
    // Load parameters
    nh_.param<std::string>("disturbance_link", link_name_, "manipulator::wrist_3_link");
    nh_.param<std::string>("reference_frame", reference_frame_, "world");
    nh_.param<double>("force_duration", force_duration_, 100.0);
    
    // Service server (our custom service)
    disturbance_service_ = nh_.advertiseService(
      "/apply_disturbance", 
      &DisturbanceApplier::disturbanceCallback, 
      this
    );
    
    // Service client (Gazebo's service)
    gazebo_wrench_client_ = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>(
      "/gazebo/apply_body_wrench"
    );
    
    // Publisher for force visualization
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/disturbance_marker", 1);
    
    ROS_INFO("==============================================");
    ROS_INFO("Disturbance Applier Node Initialized");
    ROS_INFO("  Target Link: %s", link_name_.c_str());
    ROS_INFO("  Reference Frame: %s", reference_frame_.c_str());
    ROS_INFO("  Service: /apply_disturbance");
    ROS_INFO("==============================================");
  }
  
  bool disturbanceCallback(
    manipulator_msgs::ApplyDisturbance::Request& req,
    manipulator_msgs::ApplyDisturbance::Response& res
  ) {
    // Toggle disturbance state
    disturbance_active_ = !disturbance_active_;
    
    if (disturbance_active_) {
      // Store requested force
      current_force_ = req.force;
      
      ROS_INFO("Applying disturbance force: [%.2f, %.2f, %.2f] N", 
               req.force.x, req.force.y, req.force.z);
      
      // Apply force in Gazebo
      if (applyForceInGazebo(current_force_)) {
        res.success = true;
        res.message = "Disturbance applied successfully";
        
        // Publish visualization marker
        publishForceMarker(current_force_, true);
      } else {
        res.success = false;
        res.message = "Failed to apply disturbance in Gazebo";
        ROS_ERROR("%s", res.message.c_str());
        disturbance_active_ = false;
      }
    } else {
      // Remove force (apply zero force)
      geometry_msgs::Vector3 zero_force;
      zero_force.x = zero_force.y = zero_force.z = 0.0;
      
      ROS_INFO("Removing disturbance force");
      
      if (applyForceInGazebo(zero_force)) {
        res.success = true;
        res.message = "Disturbance removed successfully";
        
        // Remove visualization marker
        publishForceMarker(zero_force, false);
      } else {
        res.success = false;
        res.message = "Failed to remove disturbance";
        ROS_ERROR("%s", res.message.c_str());
      }
    }
    
    return true;
  }
  
  bool applyForceInGazebo(const geometry_msgs::Vector3& force) {
    gazebo_msgs::ApplyBodyWrench wrench_srv;
    
    wrench_srv.request.body_name = link_name_;
    wrench_srv.request.reference_frame = reference_frame_;
    wrench_srv.request.reference_point.x = 0.0;
    wrench_srv.request.reference_point.y = 0.0;
    wrench_srv.request.reference_point.z = 0.0;
    wrench_srv.request.wrench.force = force;
    wrench_srv.request.wrench.torque.x = 0.0;
    wrench_srv.request.wrench.torque.y = 0.0;
    wrench_srv.request.wrench.torque.z = 0.0;
    wrench_srv.request.start_time = ros::Time(0);  // Apply immediately
    wrench_srv.request.duration = ros::Duration(force_duration_);
    
    if (gazebo_wrench_client_.call(wrench_srv)) {
      if (wrench_srv.response.success) {
        return true;
      } else {
        ROS_ERROR("Gazebo service returned failure: %s", 
                  wrench_srv.response.status_message.c_str());
        return false;
      }
    } else {
      ROS_ERROR("Failed to call Gazebo ApplyBodyWrench service");
      return false;
    }
  }
  
  void publishForceMarker(const geometry_msgs::Vector3& force, bool active) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = link_name_.substr(link_name_.find("::") + 2);  // Remove model name
    marker.header.stamp = ros::Time::now();
    marker.ns = "disturbance_force";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = active ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETEALL;
    
    if (active) {
      // Arrow start point (at link origin)
      geometry_msgs::Point start;
      start.x = 0.0;
      start.y = 0.0;
      start.z = 0.0;
      
      // Arrow end point (scaled force vector)
      geometry_msgs::Point end;
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
      
      marker.lifetime = ros::Duration(0);  // Never expire
    }
    
    marker_pub_.publish(marker);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "disturbance_applier_node");
  
  DisturbanceApplier applier;
  
  ROS_INFO("Disturbance Applier Node running... Press Ctrl+C to exit");
  ros::spin();
  
  return 0;
}
