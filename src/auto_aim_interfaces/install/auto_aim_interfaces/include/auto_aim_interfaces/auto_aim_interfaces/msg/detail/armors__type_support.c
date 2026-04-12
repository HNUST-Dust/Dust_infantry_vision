// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from auto_aim_interfaces:msg/Armors.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "auto_aim_interfaces/msg/detail/armors__rosidl_typesupport_introspection_c.h"
#include "auto_aim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "auto_aim_interfaces/msg/detail/armors__functions.h"
#include "auto_aim_interfaces/msg/detail/armors__struct.h"


// Include directives for member types
// Member `poses`
#include "geometry_msgs/msg/pose.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  auto_aim_interfaces__msg__Armors__init(message_memory);
}

void auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_fini_function(void * message_memory)
{
  auto_aim_interfaces__msg__Armors__fini(message_memory);
}

size_t auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__size_function__Armors__poses(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__get_const_function__Armors__poses(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__get_function__Armors__poses(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__fetch_function__Armors__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Pose * item =
    ((const geometry_msgs__msg__Pose *)
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__get_const_function__Armors__poses(untyped_member, index));
  geometry_msgs__msg__Pose * value =
    (geometry_msgs__msg__Pose *)(untyped_value);
  *value = *item;
}

void auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__assign_function__Armors__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Pose * item =
    ((geometry_msgs__msg__Pose *)
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__get_function__Armors__poses(untyped_member, index));
  const geometry_msgs__msg__Pose * value =
    (const geometry_msgs__msg__Pose *)(untyped_value);
  *item = *value;
}

bool auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__resize_function__Armors__poses(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_member_array[1] = {
  {
    "poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__Armors, poses),  // bytes offset in struct
    NULL,  // default value
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__size_function__Armors__poses,  // size() function pointer
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__get_const_function__Armors__poses,  // get_const(index) function pointer
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__get_function__Armors__poses,  // get(index) function pointer
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__fetch_function__Armors__poses,  // fetch(index, &value) function pointer
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__assign_function__Armors__poses,  // assign(index, value) function pointer
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__resize_function__Armors__poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_members = {
  "auto_aim_interfaces__msg",  // message namespace
  "Armors",  // message name
  1,  // number of fields
  sizeof(auto_aim_interfaces__msg__Armors),
  auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_member_array,  // message members
  auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_init_function,  // function to initialize message memory (memory has to be allocated)
  auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_type_support_handle = {
  0,
  &auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, auto_aim_interfaces, msg, Armors)() {
  auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_type_support_handle.typesupport_identifier) {
    auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &auto_aim_interfaces__msg__Armors__rosidl_typesupport_introspection_c__Armors_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
