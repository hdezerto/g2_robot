// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_custom_interfaces:srv/Pickup.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_custom_interfaces/srv/pickup.h"


#ifndef MY_CUSTOM_INTERFACES__SRV__DETAIL__PICKUP__STRUCT_H_
#define MY_CUSTOM_INTERFACES__SRV__DETAIL__PICKUP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Pickup in the package my_custom_interfaces.
typedef struct my_custom_interfaces__srv__Pickup_Request
{
  /// e.g., "Pick" or "Drop"
  rosidl_runtime_c__String command;
} my_custom_interfaces__srv__Pickup_Request;

// Struct for a sequence of my_custom_interfaces__srv__Pickup_Request.
typedef struct my_custom_interfaces__srv__Pickup_Request__Sequence
{
  my_custom_interfaces__srv__Pickup_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_custom_interfaces__srv__Pickup_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Pickup in the package my_custom_interfaces.
typedef struct my_custom_interfaces__srv__Pickup_Response
{
  bool success;
  /// optional feedback like "Picked successfully"
  rosidl_runtime_c__String message;
} my_custom_interfaces__srv__Pickup_Response;

// Struct for a sequence of my_custom_interfaces__srv__Pickup_Response.
typedef struct my_custom_interfaces__srv__Pickup_Response__Sequence
{
  my_custom_interfaces__srv__Pickup_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_custom_interfaces__srv__Pickup_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  my_custom_interfaces__srv__Pickup_Event__request__MAX_SIZE = 1
};
// response
enum
{
  my_custom_interfaces__srv__Pickup_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Pickup in the package my_custom_interfaces.
typedef struct my_custom_interfaces__srv__Pickup_Event
{
  service_msgs__msg__ServiceEventInfo info;
  my_custom_interfaces__srv__Pickup_Request__Sequence request;
  my_custom_interfaces__srv__Pickup_Response__Sequence response;
} my_custom_interfaces__srv__Pickup_Event;

// Struct for a sequence of my_custom_interfaces__srv__Pickup_Event.
typedef struct my_custom_interfaces__srv__Pickup_Event__Sequence
{
  my_custom_interfaces__srv__Pickup_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_custom_interfaces__srv__Pickup_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_CUSTOM_INTERFACES__SRV__DETAIL__PICKUP__STRUCT_H_
