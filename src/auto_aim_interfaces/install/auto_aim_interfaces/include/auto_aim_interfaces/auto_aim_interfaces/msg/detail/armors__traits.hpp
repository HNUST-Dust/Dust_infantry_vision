// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/Armors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ARMORS__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ARMORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/armors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Armors & msg,
  std::ostream & out)
{
  out << "{";
  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Armors & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Armors & msg, bool use_flow_style = false)
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
  const auto_aim_interfaces::msg::Armors & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::Armors & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::Armors>()
{
  return "auto_aim_interfaces::msg::Armors";
}

template<>
inline const char * name<auto_aim_interfaces::msg::Armors>()
{
  return "auto_aim_interfaces/msg/Armors";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::Armors>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::Armors>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<auto_aim_interfaces::msg::Armors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ARMORS__TRAITS_HPP_
