// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sp_msgs:msg/AutoaimTargetMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sp_msgs/msg/detail/autoaim_target_msg__rosidl_typesupport_introspection_c.h"
#include "sp_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sp_msgs/msg/detail/autoaim_target_msg__functions.h"
#include "sp_msgs/msg/detail/autoaim_target_msg__struct.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `target_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sp_msgs__msg__AutoaimTargetMsg__init(message_memory);
}

void sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_fini_function(void * message_memory)
{
  sp_msgs__msg__AutoaimTargetMsg__fini(message_memory);
}

size_t sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__size_function__AutoaimTargetMsg__target_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int8__Sequence * member =
    (const rosidl_runtime_c__int8__Sequence *)(untyped_member);
  return member->size;
}

const void * sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__get_const_function__AutoaimTargetMsg__target_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int8__Sequence * member =
    (const rosidl_runtime_c__int8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__get_function__AutoaimTargetMsg__target_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int8__Sequence * member =
    (rosidl_runtime_c__int8__Sequence *)(untyped_member);
  return &member->data[index];
}

void sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__fetch_function__AutoaimTargetMsg__target_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int8_t * item =
    ((const int8_t *)
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__get_const_function__AutoaimTargetMsg__target_ids(untyped_member, index));
  int8_t * value =
    (int8_t *)(untyped_value);
  *value = *item;
}

void sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__assign_function__AutoaimTargetMsg__target_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int8_t * item =
    ((int8_t *)
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__get_function__AutoaimTargetMsg__target_ids(untyped_member, index));
  const int8_t * value =
    (const int8_t *)(untyped_value);
  *item = *value;
}

bool sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__resize_function__AutoaimTargetMsg__target_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int8__Sequence * member =
    (rosidl_runtime_c__int8__Sequence *)(untyped_member);
  rosidl_runtime_c__int8__Sequence__fini(member);
  return rosidl_runtime_c__int8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_member_array[2] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sp_msgs__msg__AutoaimTargetMsg, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sp_msgs__msg__AutoaimTargetMsg, target_ids),  // bytes offset in struct
    NULL,  // default value
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__size_function__AutoaimTargetMsg__target_ids,  // size() function pointer
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__get_const_function__AutoaimTargetMsg__target_ids,  // get_const(index) function pointer
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__get_function__AutoaimTargetMsg__target_ids,  // get(index) function pointer
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__fetch_function__AutoaimTargetMsg__target_ids,  // fetch(index, &value) function pointer
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__assign_function__AutoaimTargetMsg__target_ids,  // assign(index, value) function pointer
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__resize_function__AutoaimTargetMsg__target_ids  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_members = {
  "sp_msgs__msg",  // message namespace
  "AutoaimTargetMsg",  // message name
  2,  // number of fields
  sizeof(sp_msgs__msg__AutoaimTargetMsg),
  sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_member_array,  // message members
  sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_type_support_handle = {
  0,
  &sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sp_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sp_msgs, msg, AutoaimTargetMsg)() {
  sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_type_support_handle.typesupport_identifier) {
    sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sp_msgs__msg__AutoaimTargetMsg__rosidl_typesupport_introspection_c__AutoaimTargetMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
