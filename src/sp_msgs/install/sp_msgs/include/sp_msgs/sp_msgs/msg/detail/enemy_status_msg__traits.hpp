// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sp_msgs:msg/EnemyStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__TRAITS_HPP_
#define SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sp_msgs/msg/detail/enemy_status_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace sp_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EnemyStatusMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: invincible_enemy_ids
  {
    if (msg.invincible_enemy_ids.size() == 0) {
      out << "invincible_enemy_ids: []";
    } else {
      out << "invincible_enemy_ids: [";
      size_t pending_items = msg.invincible_enemy_ids.size();
      for (auto item : msg.invincible_enemy_ids) {
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
  const EnemyStatusMsg & msg,
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

  // member: invincible_enemy_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.invincible_enemy_ids.size() == 0) {
      out << "invincible_enemy_ids: []\n";
    } else {
      out << "invincible_enemy_ids:\n";
      for (auto item : msg.invincible_enemy_ids) {
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

inline std::string to_yaml(const EnemyStatusMsg & msg, bool use_flow_style = false)
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

}  // namespace sp_msgs

namespace rosidl_generator_traits
{

[[deprecated("use sp_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sp_msgs::msg::EnemyStatusMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  sp_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sp_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const sp_msgs::msg::EnemyStatusMsg & msg)
{
  return sp_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sp_msgs::msg::EnemyStatusMsg>()
{
  return "sp_msgs::msg::EnemyStatusMsg";
}

template<>
inline const char * name<sp_msgs::msg::EnemyStatusMsg>()
{
  return "sp_msgs/msg/EnemyStatusMsg";
}

template<>
struct has_fixed_size<sp_msgs::msg::EnemyStatusMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sp_msgs::msg::EnemyStatusMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sp_msgs::msg::EnemyStatusMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SP_MSGS__MSG__DETAIL__ENEMY_STATUS_MSG__TRAITS_HPP_
