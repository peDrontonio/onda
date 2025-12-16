// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from manipulator_msgs:srv/ApplyDisturbance.idl
// generated code does not contain a copyright notice

#ifndef MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__STRUCT_HPP_
#define MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'force'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Request __attribute__((deprecated))
#else
# define DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Request __declspec(deprecated)
#endif

namespace manipulator_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ApplyDisturbance_Request_
{
  using Type = ApplyDisturbance_Request_<ContainerAllocator>;

  explicit ApplyDisturbance_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : force(_init)
  {
    (void)_init;
  }

  explicit ApplyDisturbance_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : force(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _force_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _force_type force;

  // setters for named parameter idiom
  Type & set__force(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->force = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Request
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Request
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ApplyDisturbance_Request_ & other) const
  {
    if (this->force != other.force) {
      return false;
    }
    return true;
  }
  bool operator!=(const ApplyDisturbance_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ApplyDisturbance_Request_

// alias to use template instance with default allocator
using ApplyDisturbance_Request =
  manipulator_msgs::srv::ApplyDisturbance_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace manipulator_msgs


#ifndef _WIN32
# define DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Response __attribute__((deprecated))
#else
# define DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Response __declspec(deprecated)
#endif

namespace manipulator_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ApplyDisturbance_Response_
{
  using Type = ApplyDisturbance_Response_<ContainerAllocator>;

  explicit ApplyDisturbance_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ApplyDisturbance_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Response
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__manipulator_msgs__srv__ApplyDisturbance_Response
    std::shared_ptr<manipulator_msgs::srv::ApplyDisturbance_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ApplyDisturbance_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ApplyDisturbance_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ApplyDisturbance_Response_

// alias to use template instance with default allocator
using ApplyDisturbance_Response =
  manipulator_msgs::srv::ApplyDisturbance_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace manipulator_msgs

namespace manipulator_msgs
{

namespace srv
{

struct ApplyDisturbance
{
  using Request = manipulator_msgs::srv::ApplyDisturbance_Request;
  using Response = manipulator_msgs::srv::ApplyDisturbance_Response;
};

}  // namespace srv

}  // namespace manipulator_msgs

#endif  // MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__STRUCT_HPP_
