// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from auto_aim_interfaces:msg/TrackerInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "auto_aim_interfaces/msg/detail/tracker_info__struct.hpp"
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

void TrackerInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) auto_aim_interfaces::msg::TrackerInfo(_init);
}

void TrackerInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<auto_aim_interfaces::msg::TrackerInfo *>(message_memory);
  typed_message->~TrackerInfo();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TrackerInfo_message_member_array[2] = {
  {
    "is_tracking",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::TrackerInfo, is_tracking),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tracking_score",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces::msg::TrackerInfo, tracking_score),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TrackerInfo_message_members = {
  "auto_aim_interfaces::msg",  // message namespace
  "TrackerInfo",  // message name
  2,  // number of fields
  sizeof(auto_aim_interfaces::msg::TrackerInfo),
  TrackerInfo_message_member_array,  // message members
  TrackerInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackerInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TrackerInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TrackerInfo_message_members,
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
get_message_type_support_handle<auto_aim_interfaces::msg::TrackerInfo>()
{
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::TrackerInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, auto_aim_interfaces, msg, TrackerInfo)() {
  return &::auto_aim_interfaces::msg::rosidl_typesupport_introspection_cpp::TrackerInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
