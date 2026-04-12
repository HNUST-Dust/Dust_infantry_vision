// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rm_decision_interfaces:msg/Refree.idl
// generated code does not contain a copyright notice

#ifndef RM_DECISION_INTERFACES__MSG__DETAIL__REFREE__TRAITS_HPP_
#define RM_DECISION_INTERFACES__MSG__DETAIL__REFREE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rm_decision_interfaces/msg/detail/refree__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rm_decision_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Refree & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: game_status
  {
    out << "game_status: ";
    rosidl_generator_traits::value_to_yaml(msg.game_status, out);
    out << ", ";
  }

  // member: robot_hp
  {
    if (msg.robot_hp.size() == 0) {
      out << "robot_hp: []";
    } else {
      out << "robot_hp: [";
      size_t pending_items = msg.robot_hp.size();
      for (auto item : msg.robot_hp) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: robot_status
  {
    if (msg.robot_status.size() == 0) {
      out << "robot_status: []";
    } else {
      out << "robot_status: [";
      size_t pending_items = msg.robot_status.size();
      for (auto item : msg.robot_status) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Refree & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: game_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "game_status: ";
    rosidl_generator_traits::value_to_yaml(msg.game_status, out);
    out << "\n";
  }

  // member: robot_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_hp.size() == 0) {
      out << "robot_hp: []\n";
    } else {
      out << "robot_hp:\n";
      for (auto item : msg.robot_hp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: robot_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_status.size() == 0) {
      out << "robot_status: []\n";
    } else {
      out << "robot_status:\n";
      for (auto item : msg.robot_status) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Refree & msg, bool use_flow_style = false)
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

}  // namespace rm_decision_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rm_decision_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rm_decision_interfaces::msg::Refree & msg,
  std::ostream & out, size_t indentation = 0)
{
  rm_decision_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rm_decision_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rm_decision_interfaces::msg::Refree & msg)
{
  return rm_decision_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rm_decision_interfaces::msg::Refree>()
{
  return "rm_decision_interfaces::msg::Refree";
}

template<>
inline const char * name<rm_decision_interfaces::msg::Refree>()
{
  return "rm_decision_interfaces/msg/Refree";
}

template<>
struct has_fixed_size<rm_decision_interfaces::msg::Refree>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rm_decision_interfaces::msg::Refree>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rm_decision_interfaces::msg::Refree>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RM_DECISION_INTERFACES__MSG__DETAIL__REFREE__TRAITS_HPP_
