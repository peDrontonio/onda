// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from manipulator_msgs:srv/ApplyDisturbance.idl
// generated code does not contain a copyright notice

#ifndef MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__BUILDER_HPP_
#define MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "manipulator_msgs/srv/detail/apply_disturbance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace manipulator_msgs
{

namespace srv
{

namespace builder
{

class Init_ApplyDisturbance_Request_force
{
public:
  Init_ApplyDisturbance_Request_force()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::manipulator_msgs::srv::ApplyDisturbance_Request force(::manipulator_msgs::srv::ApplyDisturbance_Request::_force_type arg)
  {
    msg_.force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::manipulator_msgs::srv::ApplyDisturbance_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::manipulator_msgs::srv::ApplyDisturbance_Request>()
{
  return manipulator_msgs::srv::builder::Init_ApplyDisturbance_Request_force();
}

}  // namespace manipulator_msgs


namespace manipulator_msgs
{

namespace srv
{

namespace builder
{

class Init_ApplyDisturbance_Response_message
{
public:
  explicit Init_ApplyDisturbance_Response_message(::manipulator_msgs::srv::ApplyDisturbance_Response & msg)
  : msg_(msg)
  {}
  ::manipulator_msgs::srv::ApplyDisturbance_Response message(::manipulator_msgs::srv::ApplyDisturbance_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::manipulator_msgs::srv::ApplyDisturbance_Response msg_;
};

class Init_ApplyDisturbance_Response_success
{
public:
  Init_ApplyDisturbance_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ApplyDisturbance_Response_message success(::manipulator_msgs::srv::ApplyDisturbance_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ApplyDisturbance_Response_message(msg_);
  }

private:
  ::manipulator_msgs::srv::ApplyDisturbance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::manipulator_msgs::srv::ApplyDisturbance_Response>()
{
  return manipulator_msgs::srv::builder::Init_ApplyDisturbance_Response_success();
}

}  // namespace manipulator_msgs

#endif  // MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__BUILDER_HPP_
