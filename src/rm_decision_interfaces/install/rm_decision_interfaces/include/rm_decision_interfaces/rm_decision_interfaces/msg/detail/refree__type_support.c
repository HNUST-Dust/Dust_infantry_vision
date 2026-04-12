// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rm_decision_interfaces:msg/Refree.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rm_decision_interfaces/msg/detail/refree__rosidl_typesupport_introspection_c.h"
#include "rm_decision_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rm_decision_interfaces/msg/detail/refree__functions.h"
#include "rm_decision_interfaces/msg/detail/refree__struct.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rm_decision_interfaces__msg__Refree__init(message_memory);
}

void rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_fini_function(void * message_memory)
{
  rm_decision_interfaces__msg__Refree__fini(message_memory);
}

size_t rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__size_function__Refree__robot_hp(
  const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_const_function__Refree__robot_hp(
  const void * untyped_member, size_t index)
{
  const uint16_t * member =
    (const uint16_t *)(untyped_member);
  return &member[index];
}

void * rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_function__Refree__robot_hp(
  void * untyped_member, size_t index)
{
  uint16_t * member =
    (uint16_t *)(untyped_member);
  return &member[index];
}

void rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__fetch_function__Refree__robot_hp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_const_function__Refree__robot_hp(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__assign_function__Refree__robot_hp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_function__Refree__robot_hp(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

size_t rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__size_function__Refree__robot_status(
  const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_const_function__Refree__robot_status(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_function__Refree__robot_status(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__fetch_function__Refree__robot_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_const_function__Refree__robot_status(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__assign_function__Refree__robot_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_function__Refree__robot_status(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_member_array[4] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces__msg__Refree, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "game_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces__msg__Refree, game_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_hp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces__msg__Refree, robot_hp),  // bytes offset in struct
    NULL,  // default value
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__size_function__Refree__robot_hp,  // size() function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_const_function__Refree__robot_hp,  // get_const(index) function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_function__Refree__robot_hp,  // get(index) function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__fetch_function__Refree__robot_hp,  // fetch(index, &value) function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__assign_function__Refree__robot_hp,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces__msg__Refree, robot_status),  // bytes offset in struct
    NULL,  // default value
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__size_function__Refree__robot_status,  // size() function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_const_function__Refree__robot_status,  // get_const(index) function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__get_function__Refree__robot_status,  // get(index) function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__fetch_function__Refree__robot_status,  // fetch(index, &value) function pointer
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__assign_function__Refree__robot_status,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_members = {
  "rm_decision_interfaces__msg",  // message namespace
  "Refree",  // message name
  4,  // number of fields
  sizeof(rm_decision_interfaces__msg__Refree),
  rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_member_array,  // message members
  rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_init_function,  // function to initialize message memory (memory has to be allocated)
  rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_type_support_handle = {
  0,
  &rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rm_decision_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rm_decision_interfaces, msg, Refree)() {
  rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_type_support_handle.typesupport_identifier) {
    rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rm_decision_interfaces__msg__Refree__rosidl_typesupport_introspection_c__Refree_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
