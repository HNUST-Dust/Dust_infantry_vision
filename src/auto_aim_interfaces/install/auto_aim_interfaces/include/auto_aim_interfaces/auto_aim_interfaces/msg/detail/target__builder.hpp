// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Target_velocity
{
public:
  explicit Init_Target_velocity(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Target velocity(::auto_aim_interfaces::msg::Target::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_pose
{
public:
  Init_Target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_velocity pose(::auto_aim_interfaces::msg::Target::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Target_velocity(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Target>()
{
  return auto_aim_interfaces::msg::builder::Init_Target_pose();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
