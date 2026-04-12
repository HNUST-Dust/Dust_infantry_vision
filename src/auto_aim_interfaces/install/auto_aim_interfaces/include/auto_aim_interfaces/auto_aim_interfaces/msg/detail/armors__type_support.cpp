// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from auto_aim_interfaces:msg/Armors.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "auto_aim_interfaces/msg/detail/armors__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace auto_aim_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Armors_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) auto_aim_interfaces::msg::Armors(_init);
}

void Armors_fini_function(void * message_memory)
{
  auto typed_message = static_cast<auto_aim_interfaces::msg::Armors *>(message_memory);
  typed_message->~Armors();
}

size_t size_function__Armors__poses(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Armors__poses(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__Armors__poses(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void fetch_function__Armors__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Pose *>(
    get_const_function__Armors__poses(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Pose *>(untyped_value);
  value = item;
}

void assign_function__Armors__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Pose *>(
    get_function__Armors__poses(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Pose *>(untyped_value);
  item = value;
}

void resize_function__Armors__poses(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Armors_message_member_array[1] = {
  {
    "poses",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::Armors, poses),  // bytes offset in struct
    nullptr,  // default value
    size_function__Armors__poses,  // size() function pointer
    get_const_function__Armors__poses,  // get_const(index) function pointer
    get_function__Armors__poses,  // get(index) function pointer
    fetch_function__Armors__poses,  // fetch(index, &value) function pointer
    assign_function__Armors__poses,  // assign(index, value) function pointer
    resize_function__Armors__poses  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Armors_message_members = {
  "auto_aim_interfaces::msg",  // message namespace
  "Armors",  // message name
  1,  // number of fields
  sizeof(auto_aim_interfaces::msg::Armors),
  Armors_message_member_array,  // message members
  Armors_init_function,  // function to initialize message memory (memory has to be allocated)
  Armors_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Armors_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Armors_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace auto_aim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<auto_aim_interfaces::msg::Armors>()
{
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::Armors_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, auto_aim_interfaces, msg, Armors)() {
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::Armors_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
