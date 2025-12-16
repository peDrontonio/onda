// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from manipulator_msgs:srv/ApplyDisturbance.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "manipulator_msgs/srv/detail/apply_disturbance__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace manipulator_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ApplyDisturbance_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ApplyDisturbance_Request_type_support_ids_t;

static const _ApplyDisturbance_Request_type_support_ids_t _ApplyDisturbance_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ApplyDisturbance_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ApplyDisturbance_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ApplyDisturbance_Request_type_support_symbol_names_t _ApplyDisturbance_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, manipulator_msgs, srv, ApplyDisturbance_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, manipulator_msgs, srv, ApplyDisturbance_Request)),
  }
};

typedef struct _ApplyDisturbance_Request_type_support_data_t
{
  void * data[2];
} _ApplyDisturbance_Request_type_support_data_t;

static _ApplyDisturbance_Request_type_support_data_t _ApplyDisturbance_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ApplyDisturbance_Request_message_typesupport_map = {
  2,
  "manipulator_msgs",
  &_ApplyDisturbance_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ApplyDisturbance_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ApplyDisturbance_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ApplyDisturbance_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ApplyDisturbance_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace manipulator_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<manipulator_msgs::srv::ApplyDisturbance_Request>()
{
  return &::manipulator_msgs::srv::rosidl_typesupport_cpp::ApplyDisturbance_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, manipulator_msgs, srv, ApplyDisturbance_Request)() {
  return get_message_type_support_handle<manipulator_msgs::srv::ApplyDisturbance_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "manipulator_msgs/srv/detail/apply_disturbance__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace manipulator_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ApplyDisturbance_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ApplyDisturbance_Response_type_support_ids_t;

static const _ApplyDisturbance_Response_type_support_ids_t _ApplyDisturbance_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ApplyDisturbance_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ApplyDisturbance_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ApplyDisturbance_Response_type_support_symbol_names_t _ApplyDisturbance_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, manipulator_msgs, srv, ApplyDisturbance_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, manipulator_msgs, srv, ApplyDisturbance_Response)),
  }
};

typedef struct _ApplyDisturbance_Response_type_support_data_t
{
  void * data[2];
} _ApplyDisturbance_Response_type_support_data_t;

static _ApplyDisturbance_Response_type_support_data_t _ApplyDisturbance_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ApplyDisturbance_Response_message_typesupport_map = {
  2,
  "manipulator_msgs",
  &_ApplyDisturbance_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ApplyDisturbance_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ApplyDisturbance_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ApplyDisturbance_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ApplyDisturbance_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace manipulator_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<manipulator_msgs::srv::ApplyDisturbance_Response>()
{
  return &::manipulator_msgs::srv::rosidl_typesupport_cpp::ApplyDisturbance_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, manipulator_msgs, srv, ApplyDisturbance_Response)() {
  return get_message_type_support_handle<manipulator_msgs::srv::ApplyDisturbance_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "manipulator_msgs/srv/detail/apply_disturbance__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace manipulator_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ApplyDisturbance_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ApplyDisturbance_type_support_ids_t;

static const _ApplyDisturbance_type_support_ids_t _ApplyDisturbance_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ApplyDisturbance_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ApplyDisturbance_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ApplyDisturbance_type_support_symbol_names_t _ApplyDisturbance_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, manipulator_msgs, srv, ApplyDisturbance)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, manipulator_msgs, srv, ApplyDisturbance)),
  }
};

typedef struct _ApplyDisturbance_type_support_data_t
{
  void * data[2];
} _ApplyDisturbance_type_support_data_t;

static _ApplyDisturbance_type_support_data_t _ApplyDisturbance_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ApplyDisturbance_service_typesupport_map = {
  2,
  "manipulator_msgs",
  &_ApplyDisturbance_service_typesupport_ids.typesupport_identifier[0],
  &_ApplyDisturbance_service_typesupport_symbol_names.symbol_name[0],
  &_ApplyDisturbance_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ApplyDisturbance_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ApplyDisturbance_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace manipulator_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<manipulator_msgs::srv::ApplyDisturbance>()
{
  return &::manipulator_msgs::srv::rosidl_typesupport_cpp::ApplyDisturbance_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, manipulator_msgs, srv, ApplyDisturbance)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<manipulator_msgs::srv::ApplyDisturbance>();
}

#ifdef __cplusplus
}
#endif
