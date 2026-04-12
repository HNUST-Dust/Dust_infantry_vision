// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/gimbal_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalCmd & msg, bool use_flow_style = false)
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
  const auto_aim_interfaces::msg::GimbalCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::GimbalCmd & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::GimbalCmd>()
{
  return "auto_aim_interfaces::msg::GimbalCmd";
}

template<>
inline const char * name<auto_aim_interfaces::msg::GimbalCmd>()
{
  return "auto_aim_interfaces/msg/GimbalCmd";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::GimbalCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::GimbalCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim_interfaces::msg::GimbalCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__TRAITS_HPP_
