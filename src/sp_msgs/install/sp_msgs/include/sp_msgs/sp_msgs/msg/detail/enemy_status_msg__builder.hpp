// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sp_msgs:msg/EnemyStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__BUILDER_HPP_
#define SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sp_msgs/msg/detail/enemy_status_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sp_msgs
{

namespace msg
{

namespace builder
{

class Init_EnemyStatusMsg_invincible_enemy_ids
{
public:
  explicit Init_EnemyStatusMsg_invincible_enemy_ids(::sp_msgs::msg::EnemyStatusMsg & msg)
  : msg_(msg)
  {}
  ::sp_msgs::msg::EnemyStatusMsg invincible_enemy_ids(::sp_msgs::msg::EnemyStatusMsg::_invincible_enemy_ids_type arg)
  {
    msg_.invincible_enemy_ids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sp_msgs::msg::EnemyStatusMsg msg_;
};

class Init_EnemyStatusMsg_timestamp
{
public:
  Init_EnemyStatusMsg_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EnemyStatusMsg_invincible_enemy_ids timestamp(::sp_msgs::msg::EnemyStatusMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_EnemyStatusMsg_invincible_enemy_ids(msg_);
  }

private:
  ::sp_msgs::msg::EnemyStatusMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sp_msgs::msg::EnemyStatusMsg>()
{
  return sp_msgs::msg::builder::Init_EnemyStatusMsg_timestamp();
}

}  // namespace sp_msgs

#endif  // SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__BUILDER_HPP_
