// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/Target in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Target
{
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist velocity;
} auto_aim_interfaces__msg__Target;

// Struct for a sequence of auto_aim_interfaces__msg__Target.
typedef struct auto_aim_interfaces__msg__Target__Sequence
{
  auto_aim_interfaces__msg__Target * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Target__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_H_
