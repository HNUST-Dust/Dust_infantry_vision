// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sp_msgs:msg/AutoaimTargetMsg.idl
// generated code does not contain a copyright notice

#ifndef SP_MSGS__MSG__DETAIL__AUTOAIM_TARGET_MSG__BUILDER_HPP_
#define SP_MSGS__MSG__DETAIL__AUTOAIM_TARGET_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sp_msgs/msg/detail/autoaim_target_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sp_msgs
{

namespace msg
{

namespace builder
{

class Init_AutoaimTargetMsg_target_ids
{
public:
  explicit Init_AutoaimTargetMsg_target_ids(::sp_msgs::msg::AutoaimTargetMsg & msg)
  : msg_(msg)
  {}
  ::sp_msgs::msg::AutoaimTargetMsg target_ids(::sp_msgs::msg::AutoaimTargetMsg::_target_ids_type arg)
  {
    msg_.target_ids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sp_msgs::msg::AutoaimTargetMsg msg_;
};

class Init_AutoaimTargetMsg_timestamp
{
public:
  Init_AutoaimTargetMsg_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AutoaimTargetMsg_target_ids timestamp(::sp_msgs::msg::AutoaimTargetMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_AutoaimTargetMsg_target_ids(msg_);
  }

private:
  ::sp_msgs::msg::AutoaimTargetMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sp_msgs::msg::AutoaimTargetMsg>()
{
  return sp_msgs::msg::builder::Init_AutoaimTargetMsg_timestamp();
}

}  // namespace sp_msgs

#endif  // SP_MSGS__MSG__DETAIL__AUTOAIM_TARGET_MSG__BUILDER_HPP_
