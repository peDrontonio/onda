// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from manipulator_msgs:srv/ApplyDisturbance.idl
// generated code does not contain a copyright notice

#ifndef MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__TRAITS_HPP_
#define MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "manipulator_msgs/srv/detail/apply_disturbance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'force'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace manipulator_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ApplyDisturbance_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: force
  {
    out << "force: ";
    to_flow_style_yaml(msg.force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ApplyDisturbance_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force:\n";
    to_block_style_yaml(msg.force, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ApplyDisturbance_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace manipulator_msgs

namespace rosidl_generator_traits
{

[[deprecated("use manipulator_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const manipulator_msgs::srv::ApplyDisturbance_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  manipulator_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use manipulator_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const manipulator_msgs::srv::ApplyDisturbance_Request & msg)
{
  return manipulator_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<manipulator_msgs::srv::ApplyDisturbance_Request>()
{
  return "manipulator_msgs::srv::ApplyDisturbance_Request";
}

template<>
inline const char * name<manipulator_msgs::srv::ApplyDisturbance_Request>()
{
  return "manipulator_msgs/srv/ApplyDisturbance_Request";
}

template<>
struct has_fixed_size<manipulator_msgs::srv::ApplyDisturbance_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<manipulator_msgs::srv::ApplyDisturbance_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<manipulator_msgs::srv::ApplyDisturbance_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace manipulator_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ApplyDisturbance_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ApplyDisturbance_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ApplyDisturbance_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace manipulator_msgs

namespace rosidl_generator_traits
{

[[deprecated("use manipulator_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const manipulator_msgs::srv::ApplyDisturbance_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  manipulator_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use manipulator_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const manipulator_msgs::srv::ApplyDisturbance_Response & msg)
{
  return manipulator_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<manipulator_msgs::srv::ApplyDisturbance_Response>()
{
  return "manipulator_msgs::srv::ApplyDisturbance_Response";
}

template<>
inline const char * name<manipulator_msgs::srv::ApplyDisturbance_Response>()
{
  return "manipulator_msgs/srv/ApplyDisturbance_Response";
}

template<>
struct has_fixed_size<manipulator_msgs::srv::ApplyDisturbance_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<manipulator_msgs::srv::ApplyDisturbance_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<manipulator_msgs::srv::ApplyDisturbance_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<manipulator_msgs::srv::ApplyDisturbance>()
{
  return "manipulator_msgs::srv::ApplyDisturbance";
}

template<>
inline const char * name<manipulator_msgs::srv::ApplyDisturbance>()
{
  return "manipulator_msgs/srv/ApplyDisturbance";
}

template<>
struct has_fixed_size<manipulator_msgs::srv::ApplyDisturbance>
  : std::integral_constant<
    bool,
    has_fixed_size<manipulator_msgs::srv::ApplyDisturbance_Request>::value &&
    has_fixed_size<manipulator_msgs::srv::ApplyDisturbance_Response>::value
  >
{
};

template<>
struct has_bounded_size<manipulator_msgs::srv::ApplyDisturbance>
  : std::integral_constant<
    bool,
    has_bounded_size<manipulator_msgs::srv::ApplyDisturbance_Request>::value &&
    has_bounded_size<manipulator_msgs::srv::ApplyDisturbance_Response>::value
  >
{
};

template<>
struct is_service<manipulator_msgs::srv::ApplyDisturbance>
  : std::true_type
{
};

template<>
struct is_service_request<manipulator_msgs::srv::ApplyDisturbance_Request>
  : std::true_type
{
};

template<>
struct is_service_response<manipulator_msgs::srv::ApplyDisturbance_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__TRAITS_HPP_
