/*
 * disturbance_applier_node.cpp
 *
 * Node that provides a ROS 2 service to apply/remove external forces on a link.
 *
 * Ported from Gazebo Classic (gazebo_msgs/ApplyBodyWrench service) to modern
 * Gazebo (gz-sim / Harmonic). Classic's ApplyBodyWrench does not exist under
 * `gz sim`; instead the `ApplyLinkWrench` system exposes gz-transport topics:
 *
 *   /world/<world>/wrench/persistent  (gz.msgs.EntityWrench) -> apply a wrench
 *                                       that persists until cleared
 *   /world/<world>/wrench/clear       (gz.msgs.Entity)       -> clear it
 *
 * Requirements to actually move the robot:
 *   - The world must load the ApplyLinkWrench system, e.g. add to the .sdf:
 *       <plugin filename="gz-sim-apply-link-wrench-system"
 *               name="gz::sim::systems::ApplyLinkWrench"/>
 *   - `disturbance_link` must be the model-scoped link name, e.g. "braco::rot4_1".
 *   - `world_name` must match the running world (README launch uses "empty").
 *
 * Author: Manipulator Test Team
 * Date: November 2025
 * Converted to ROS 2: December 2025
 * Ported to gz-sim (Harmonic): 2026
 */

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <manipulator_msgs/srv/apply_disturbance.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>

class DisturbanceApplier : public rclcpp::Node {
private:
  rclcpp::Service<manipulator_msgs::srv::ApplyDisturbance>::SharedPtr disturbance_service_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // gz-transport side (talks to the ApplyLinkWrench system in gz sim)
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher wrench_pub_;
  gz::transport::Node::Publisher clear_pub_;

  bool disturbance_active_;
  geometry_msgs::msg::Vector3 current_force_;
  std::string link_name_;
  std::string reference_frame_;
  std::string world_name_;
  double force_duration_;   // kept for launch-arg compatibility (unused: wrench is persistent)
  int marker_id_;

public:
  DisturbanceApplier() : Node("disturbance_applier_node"), disturbance_active_(false), marker_id_(0) {
    // Declare and load parameters
    this->declare_parameter("disturbance_link", "braco::rot4_1");
    this->declare_parameter("reference_frame", "world");
    this->declare_parameter("world_name", "empty");
    this->declare_parameter("force_duration", 100.0);

    this->get_parameter("disturbance_link", link_name_);
    this->get_parameter("reference_frame", reference_frame_);
    this->get_parameter("world_name", world_name_);
    this->get_parameter("force_duration", force_duration_);

    // Service server (our custom service)
    disturbance_service_ = this->create_service<manipulator_msgs::srv::ApplyDisturbance>(
      "/apply_disturbance",
      std::bind(&DisturbanceApplier::disturbanceCallback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    // gz-transport publishers to the ApplyLinkWrench system
    const std::string base = "/world/" + world_name_ + "/wrench";
    wrench_pub_ = gz_node_.Advertise<gz::msgs::EntityWrench>(base + "/persistent");
    clear_pub_  = gz_node_.Advertise<gz::msgs::Entity>(base + "/clear");

    // Publisher for force visualization
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/disturbance_marker", 10
    );

    RCLCPP_INFO(this->get_logger(), "==============================================");
    RCLCPP_INFO(this->get_logger(), "Disturbance Applier Node Initialized (gz sim)");
    RCLCPP_INFO(this->get_logger(), "  Target Link: %s", link_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  World:       %s", world_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Wrench topic: %s/persistent", base.c_str());
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
      current_force_ = request->force;

      RCLCPP_INFO(this->get_logger(), "Applying disturbance force: [%.2f, %.2f, %.2f] N",
                  request->force.x, request->force.y, request->force.z);

      if (applyForceInGz(current_force_)) {
        response->success = true;
        response->message = "Disturbance applied successfully";
        publishForceMarker(current_force_, true);
      } else {
        response->success = false;
        response->message = "Failed to apply disturbance in gz sim";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        disturbance_active_ = false;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Removing disturbance force");

      if (clearForceInGz()) {
        response->success = true;
        response->message = "Disturbance removed successfully";
        geometry_msgs::msg::Vector3 zero_force;
        publishForceMarker(zero_force, false);
      } else {
        response->success = false;
        response->message = "Failed to remove disturbance";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      }
    }
  }

  bool applyForceInGz(const geometry_msgs::msg::Vector3& force) {
    if (!wrench_pub_) {
      RCLCPP_ERROR(this->get_logger(), "gz wrench publisher is not valid");
      return false;
    }

    gz::msgs::EntityWrench msg;
    msg.mutable_entity()->set_name(link_name_);
    msg.mutable_entity()->set_type(gz::msgs::Entity::LINK);

    auto* wrench = msg.mutable_wrench();
    wrench->mutable_force()->set_x(force.x);
    wrench->mutable_force()->set_y(force.y);
    wrench->mutable_force()->set_z(force.z);
    wrench->mutable_torque()->set_x(0.0);
    wrench->mutable_torque()->set_y(0.0);
    wrench->mutable_torque()->set_z(0.0);

    return wrench_pub_.Publish(msg);
  }

  bool clearForceInGz() {
    if (!clear_pub_) {
      RCLCPP_ERROR(this->get_logger(), "gz clear publisher is not valid");
      return false;
    }

    gz::msgs::Entity ent;
    ent.set_name(link_name_);
    ent.set_type(gz::msgs::Entity::LINK);

    return clear_pub_.Publish(ent);
  }

  void publishForceMarker(const geometry_msgs::msg::Vector3& force, bool active) {
    auto marker = visualization_msgs::msg::Marker();
    // Strip the "model::" prefix so the marker frame matches the RViz TF link name
    const auto sep = link_name_.find("::");
    marker.header.frame_id = (sep == std::string::npos)
                               ? link_name_
                               : link_name_.substr(sep + 2);
    marker.header.stamp = this->now();
    marker.ns = "disturbance_force";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = active ? visualization_msgs::msg::Marker::ADD
                           : visualization_msgs::msg::Marker::DELETEALL;

    if (active) {
      geometry_msgs::msg::Point start;
      start.x = 0.0;
      start.y = 0.0;
      start.z = 0.0;

      geometry_msgs::msg::Point end;
      double scale = 0.01;  // 1 N = 1 cm
      end.x = force.x * scale;
      end.y = force.y * scale;
      end.z = force.z * scale;

      marker.points.push_back(start);
      marker.points.push_back(end);

      marker.scale.x = 0.01;   // Shaft diameter
      marker.scale.y = 0.02;   // Head diameter
      marker.scale.z = 0.03;   // Head length

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
