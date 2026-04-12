// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rm_decision_interfaces:msg/Refree.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rm_decision_interfaces/msg/detail/refree__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rm_decision_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Refree_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rm_decision_interfaces::msg::Refree(_init);
}

void Refree_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rm_decision_interfaces::msg::Refree *>(message_memory);
  typed_message->~Refree();
}

size_t size_function__Refree__robot_hp(const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * get_const_function__Refree__robot_hp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint16_t, 16> *>(untyped_member);
  return &member[index];
}

void * get_function__Refree__robot_hp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint16_t, 16> *>(untyped_member);
  return &member[index];
}

void fetch_function__Refree__robot_hp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__Refree__robot_hp(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__Refree__robot_hp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__Refree__robot_hp(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

size_t size_function__Refree__robot_status(const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * get_const_function__Refree__robot_status(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 16> *>(untyped_member);
  return &member[index];
}

void * get_function__Refree__robot_status(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 16> *>(untyped_member);
  return &member[index];
}

void fetch_function__Refree__robot_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__Refree__robot_status(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__Refree__robot_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__Refree__robot_status(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Refree_message_member_array[4] = {
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces::msg::Refree, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "game_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces::msg::Refree, game_status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_hp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces::msg::Refree, robot_hp),  // bytes offset in struct
    nullptr,  // default value
    size_function__Refree__robot_hp,  // size() function pointer
    get_const_function__Refree__robot_hp,  // get_const(index) function pointer
    get_function__Refree__robot_hp,  // get(index) function pointer
    fetch_function__Refree__robot_hp,  // fetch(index, &value) function pointer
    assign_function__Refree__robot_hp,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(rm_decision_interfaces::msg::Refree, robot_status),  // bytes offset in struct
    nullptr,  // default value
    size_function__Refree__robot_status,  // size() function pointer
    get_const_function__Refree__robot_status,  // get_const(index) function pointer
    get_function__Refree__robot_status,  // get(index) function pointer
    fetch_function__Refree__robot_status,  // fetch(index, &value) function pointer
    assign_function__Refree__robot_status,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Refree_message_members = {
  "rm_decision_interfaces::msg",  // message namespace
  "Refree",  // message name
  4,  // number of fields
  sizeof(rm_decision_interfaces::msg::Refree),
  Refree_message_member_array,  // message members
  Refree_init_function,  // function to initialize message memory (memory has to be allocated)
  Refree_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Refree_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Refree_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rm_decision_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rm_decision_interfaces::msg::Refree>()
{
  return &::rm_decision_interfaces::msg::rosidl_typesupport_introspection_cpp::Refree_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rm_decision_interfaces, msg, Refree)() {
  return &::rm_decision_interfaces::msg::rosidl_typesupport_introspection_cpp::Refree_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
