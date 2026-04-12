// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/Measurement.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/measurement__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'measurement'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Measurement & msg,
  std::ostream & out)
{
  out << "{";
  // member: measurement
  {
    out << "measurement: ";
    to_flow_style_yaml(msg.measurement, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Measurement & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: measurement
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measurement:\n";
    to_block_style_yaml(msg.measurement, out, indentation + 2);
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Measurement & msg, bool use_flow_style = false)
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
  const auto_aim_interfaces::msg::Measurement & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::Measurement & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::Measurement>()
{
  return "auto_aim_interfaces::msg::Measurement";
}

template<>
inline const char * name<auto_aim_interfaces::msg::Measurement>()
{
  return "auto_aim_interfaces/msg/Measurement";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::Measurement>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::Measurement>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<auto_aim_interfaces::msg::Measurement>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__TRAITS_HPP_
