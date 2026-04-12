// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from sp_msgs:msg/EnemyStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "sp_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "sp_msgs/msg/detail/enemy_status_msg__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace sp_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sp_msgs
cdr_serialize(
  const sp_msgs::msg::EnemyStatusMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sp_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sp_msgs::msg::EnemyStatusMsg & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sp_msgs
get_serialized_size(
  const sp_msgs::msg::EnemyStatusMsg & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sp_msgs
max_serialized_size_EnemyStatusMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace sp_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sp_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sp_msgs, msg, EnemyStatusMsg)();

#ifdef __cplusplus
}
#endif

#endif  // SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
