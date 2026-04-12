// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/TrackerInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/tracker_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrackerInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_tracking
  {
    out << "is_tracking: ";
    rosidl_generator_traits::value_to_yaml(msg.is_tracking, out);
    out << ", ";
  }

  // member: tracking_score
  {
    out << "tracking_score: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_score, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrackerInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_tracking
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_tracking: ";
    rosidl_generator_traits::value_to_yaml(msg.is_tracking, out);
    out << "\n";
  }

  // member: tracking_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_score: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_score, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackerInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace auto_aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim_interfaces::msg::TrackerInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::TrackerInfo & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::TrackerInfo>()
{
  return "auto_aim_interfaces::msg::TrackerInfo";
}

template<>
inline const char * name<auto_aim_interfaces::msg::TrackerInfo>()
{
  return "auto_aim_interfaces/msg/TrackerInfo";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::TrackerInfo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::TrackerInfo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim_interfaces::msg::TrackerInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__TRAITS_HPP_
