// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from manipulator_msgs:srv/ApplyDisturbance.idl
// generated code does not contain a copyright notice

#ifndef MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__STRUCT_H_
#define MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'force'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in srv/ApplyDisturbance in the package manipulator_msgs.
typedef struct manipulator_msgs__srv__ApplyDisturbance_Request
{
  geometry_msgs__msg__Vector3 force;
} manipulator_msgs__srv__ApplyDisturbance_Request;

// Struct for a sequence of manipulator_msgs__srv__ApplyDisturbance_Request.
typedef struct manipulator_msgs__srv__ApplyDisturbance_Request__Sequence
{
  manipulator_msgs__srv__ApplyDisturbance_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} manipulator_msgs__srv__ApplyDisturbance_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ApplyDisturbance in the package manipulator_msgs.
typedef struct manipulator_msgs__srv__ApplyDisturbance_Response
{
  bool success;
  rosidl_runtime_c__String message;
} manipulator_msgs__srv__ApplyDisturbance_Response;

// Struct for a sequence of manipulator_msgs__srv__ApplyDisturbance_Response.
typedef struct manipulator_msgs__srv__ApplyDisturbance_Response__Sequence
{
  manipulator_msgs__srv__ApplyDisturbance_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} manipulator_msgs__srv__ApplyDisturbance_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__STRUCT_H_
