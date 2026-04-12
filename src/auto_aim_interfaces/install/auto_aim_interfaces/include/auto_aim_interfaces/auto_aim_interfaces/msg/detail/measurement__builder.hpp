// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Measurement.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/measurement__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Measurement_timestamp
{
public:
  explicit Init_Measurement_timestamp(::auto_aim_interfaces::msg::Measurement & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Measurement timestamp(::auto_aim_interfaces::msg::Measurement::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Measurement msg_;
};

class Init_Measurement_measurement
{
public:
  Init_Measurement_measurement()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Measurement_timestamp measurement(::auto_aim_interfaces::msg::Measurement::_measurement_type arg)
  {
    msg_.measurement = std::move(arg);
    return Init_Measurement_timestamp(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Measurement msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Measurement>()
{
  return auto_aim_interfaces::msg::builder::Init_Measurement_measurement();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__BUILDER_HPP_
