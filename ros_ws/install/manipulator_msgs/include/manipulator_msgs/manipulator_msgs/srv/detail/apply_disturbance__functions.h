// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from manipulator_msgs:srv/ApplyDisturbance.idl
// generated code does not contain a copyright notice

#ifndef MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__FUNCTIONS_H_
#define MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "manipulator_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "manipulator_msgs/srv/detail/apply_disturbance__struct.h"

/// Initialize srv/ApplyDisturbance message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * manipulator_msgs__srv__ApplyDisturbance_Request
 * )) before or use
 * manipulator_msgs__srv__ApplyDisturbance_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Request__init(manipulator_msgs__srv__ApplyDisturbance_Request * msg);

/// Finalize srv/ApplyDisturbance message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Request__fini(manipulator_msgs__srv__ApplyDisturbance_Request * msg);

/// Create srv/ApplyDisturbance message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * manipulator_msgs__srv__ApplyDisturbance_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
manipulator_msgs__srv__ApplyDisturbance_Request *
manipulator_msgs__srv__ApplyDisturbance_Request__create();

/// Destroy srv/ApplyDisturbance message.
/**
 * It calls
 * manipulator_msgs__srv__ApplyDisturbance_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Request__destroy(manipulator_msgs__srv__ApplyDisturbance_Request * msg);

/// Check for srv/ApplyDisturbance message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Request__are_equal(const manipulator_msgs__srv__ApplyDisturbance_Request * lhs, const manipulator_msgs__srv__ApplyDisturbance_Request * rhs);

/// Copy a srv/ApplyDisturbance message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Request__copy(
  const manipulator_msgs__srv__ApplyDisturbance_Request * input,
  manipulator_msgs__srv__ApplyDisturbance_Request * output);

/// Initialize array of srv/ApplyDisturbance messages.
/**
 * It allocates the memory for the number of elements and calls
 * manipulator_msgs__srv__ApplyDisturbance_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__init(manipulator_msgs__srv__ApplyDisturbance_Request__Sequence * array, size_t size);

/// Finalize array of srv/ApplyDisturbance messages.
/**
 * It calls
 * manipulator_msgs__srv__ApplyDisturbance_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__fini(manipulator_msgs__srv__ApplyDisturbance_Request__Sequence * array);

/// Create array of srv/ApplyDisturbance messages.
/**
 * It allocates the memory for the array and calls
 * manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
manipulator_msgs__srv__ApplyDisturbance_Request__Sequence *
manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__create(size_t size);

/// Destroy array of srv/ApplyDisturbance messages.
/**
 * It calls
 * manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__destroy(manipulator_msgs__srv__ApplyDisturbance_Request__Sequence * array);

/// Check for srv/ApplyDisturbance message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__are_equal(const manipulator_msgs__srv__ApplyDisturbance_Request__Sequence * lhs, const manipulator_msgs__srv__ApplyDisturbance_Request__Sequence * rhs);

/// Copy an array of srv/ApplyDisturbance messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Request__Sequence__copy(
  const manipulator_msgs__srv__ApplyDisturbance_Request__Sequence * input,
  manipulator_msgs__srv__ApplyDisturbance_Request__Sequence * output);

/// Initialize srv/ApplyDisturbance message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * manipulator_msgs__srv__ApplyDisturbance_Response
 * )) before or use
 * manipulator_msgs__srv__ApplyDisturbance_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Response__init(manipulator_msgs__srv__ApplyDisturbance_Response * msg);

/// Finalize srv/ApplyDisturbance message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Response__fini(manipulator_msgs__srv__ApplyDisturbance_Response * msg);

/// Create srv/ApplyDisturbance message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * manipulator_msgs__srv__ApplyDisturbance_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
manipulator_msgs__srv__ApplyDisturbance_Response *
manipulator_msgs__srv__ApplyDisturbance_Response__create();

/// Destroy srv/ApplyDisturbance message.
/**
 * It calls
 * manipulator_msgs__srv__ApplyDisturbance_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Response__destroy(manipulator_msgs__srv__ApplyDisturbance_Response * msg);

/// Check for srv/ApplyDisturbance message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Response__are_equal(const manipulator_msgs__srv__ApplyDisturbance_Response * lhs, const manipulator_msgs__srv__ApplyDisturbance_Response * rhs);

/// Copy a srv/ApplyDisturbance message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Response__copy(
  const manipulator_msgs__srv__ApplyDisturbance_Response * input,
  manipulator_msgs__srv__ApplyDisturbance_Response * output);

/// Initialize array of srv/ApplyDisturbance messages.
/**
 * It allocates the memory for the number of elements and calls
 * manipulator_msgs__srv__ApplyDisturbance_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__init(manipulator_msgs__srv__ApplyDisturbance_Response__Sequence * array, size_t size);

/// Finalize array of srv/ApplyDisturbance messages.
/**
 * It calls
 * manipulator_msgs__srv__ApplyDisturbance_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__fini(manipulator_msgs__srv__ApplyDisturbance_Response__Sequence * array);

/// Create array of srv/ApplyDisturbance messages.
/**
 * It allocates the memory for the array and calls
 * manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
manipulator_msgs__srv__ApplyDisturbance_Response__Sequence *
manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__create(size_t size);

/// Destroy array of srv/ApplyDisturbance messages.
/**
 * It calls
 * manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
void
manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__destroy(manipulator_msgs__srv__ApplyDisturbance_Response__Sequence * array);

/// Check for srv/ApplyDisturbance message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__are_equal(const manipulator_msgs__srv__ApplyDisturbance_Response__Sequence * lhs, const manipulator_msgs__srv__ApplyDisturbance_Response__Sequence * rhs);

/// Copy an array of srv/ApplyDisturbance messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_manipulator_msgs
bool
manipulator_msgs__srv__ApplyDisturbance_Response__Sequence__copy(
  const manipulator_msgs__srv__ApplyDisturbance_Response__Sequence * input,
  manipulator_msgs__srv__ApplyDisturbance_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MANIPULATOR_MSGS__SRV__DETAIL__APPLY_DISTURBANCE__FUNCTIONS_H_
