// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Measurement.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'measurement'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Measurement in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Measurement
{
  geometry_msgs__msg__Pose measurement;
  float timestamp;
} auto_aim_interfaces__msg__Measurement;

// Struct for a sequence of auto_aim_interfaces__msg__Measurement.
typedef struct auto_aim_interfaces__msg__Measurement__Sequence
{
  auto_aim_interfaces__msg__Measurement * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Measurement__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__STRUCT_H_
