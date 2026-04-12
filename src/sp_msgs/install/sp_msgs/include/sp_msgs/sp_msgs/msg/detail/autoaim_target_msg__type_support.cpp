// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from sp_msgs:msg/AutoaimTargetMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "sp_msgs/msg/detail/autoaim_target_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace sp_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void AutoaimTargetMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) sp_msgs::msg::AutoaimTargetMsg(_init);
}

void AutoaimTargetMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<sp_msgs::msg::AutoaimTargetMsg *>(message_memory);
  typed_message->~AutoaimTargetMsg();
}

size_t size_function__AutoaimTargetMsg__target_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AutoaimTargetMsg__target_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__AutoaimTargetMsg__target_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__AutoaimTargetMsg__target_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int8_t *>(
    get_const_function__AutoaimTargetMsg__target_ids(untyped_member, index));
  auto & value = *reinterpret_cast<int8_t *>(untyped_value);
  value = item;
}

void assign_function__AutoaimTargetMsg__target_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int8_t *>(
    get_function__AutoaimTargetMsg__target_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const int8_t *>(untyped_value);
  item = value;
}

void resize_function__AutoaimTargetMsg__target_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AutoaimTargetMsg_message_member_array[2] = {
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sp_msgs::msg::AutoaimTargetMsg, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "target_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sp_msgs::msg::AutoaimTargetMsg, target_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__AutoaimTargetMsg__target_ids,  // size() function pointer
    get_const_function__AutoaimTargetMsg__target_ids,  // get_const(index) function pointer
    get_function__AutoaimTargetMsg__target_ids,  // get(index) function pointer
    fetch_function__AutoaimTargetMsg__target_ids,  // fetch(index, &value) function pointer
    assign_function__AutoaimTargetMsg__target_ids,  // assign(index, value) function pointer
    resize_function__AutoaimTargetMsg__target_ids  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AutoaimTargetMsg_message_members = {
  "sp_msgs::msg",  // message namespace
  "AutoaimTargetMsg",  // message name
  2,  // number of fields
  sizeof(sp_msgs::msg::AutoaimTargetMsg),
  AutoaimTargetMsg_message_member_array,  // message members
  AutoaimTargetMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  AutoaimTargetMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t AutoaimTargetMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AutoaimTargetMsg_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace sp_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<sp_msgs::msg::AutoaimTargetMsg>()
{
  return &::sp_msgs::msg::rosidl_typesupport_introspection_cpp::AutoaimTargetMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, sp_msgs, msg, AutoaimTargetMsg)() {
  return &::sp_msgs::msg::rosidl_typesupport_introspection_cpp::AutoaimTargetMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
