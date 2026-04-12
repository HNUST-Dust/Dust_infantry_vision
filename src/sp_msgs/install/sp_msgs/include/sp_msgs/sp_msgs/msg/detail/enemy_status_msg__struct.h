// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sp_msgs:msg/EnemyStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__STRUCT_H_
#define SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'invincible_enemy_ids'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/EnemyStatusMsg in the package sp_msgs.
typedef struct sp_msgs__msg__EnemyStatusMsg
{
  builtin_interfaces__msg__Time timestamp;
  rosidl_runtime_c__int8__Sequence invincible_enemy_ids;
} sp_msgs__msg__EnemyStatusMsg;

// Struct for a sequence of sp_msgs__msg__EnemyStatusMsg.
typedef struct sp_msgs__msg__EnemyStatusMsg__Sequence
{
  sp_msgs__msg__EnemyStatusMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sp_msgs__msg__EnemyStatusMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__STRUCT_H_
