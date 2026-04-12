// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TrackerInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/tracker_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrackerInfo_tracking_score
{
public:
  explicit Init_TrackerInfo_tracking_score(::auto_aim_interfaces::msg::TrackerInfo & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TrackerInfo tracking_score(::auto_aim_interfaces::msg::TrackerInfo::_tracking_score_type arg)
  {
    msg_.tracking_score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerInfo msg_;
};

class Init_TrackerInfo_is_tracking
{
public:
  Init_TrackerInfo_is_tracking()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackerInfo_tracking_score is_tracking(::auto_aim_interfaces::msg::TrackerInfo::_is_tracking_type arg)
  {
    msg_.is_tracking = std::move(arg);
    return Init_TrackerInfo_tracking_score(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TrackerInfo>()
{
  return auto_aim_interfaces::msg::builder::Init_TrackerInfo_is_tracking();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__BUILDER_HPP_
