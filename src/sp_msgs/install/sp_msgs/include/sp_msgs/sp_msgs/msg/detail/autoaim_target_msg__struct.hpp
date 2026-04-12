// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sp_msgs:msg/AutoaimTargetMsg.idl
// generated code does not contain a copyright notice

#ifndef SP_MSGS__MSG__DETAIL__AUTOAIM_TARGET_MSG__STRUCT_HPP_
#define SP_MSGS__MSG__DETAIL__AUTOAIM_TARGET_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sp_msgs__msg__AutoaimTargetMsg __attribute__((deprecated))
#else
# define DEPRECATED__sp_msgs__msg__AutoaimTargetMsg __declspec(deprecated)
#endif

namespace sp_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AutoaimTargetMsg_
{
  using Type = AutoaimTargetMsg_<ContainerAllocator>;

  explicit AutoaimTargetMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    (void)_init;
  }

  explicit AutoaimTargetMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _target_ids_type =
    std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>>;
  _target_ids_type target_ids;

  // setters for named parameter idiom
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__target_ids(
    const std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>> & _arg)
  {
    this->target_ids = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sp_msgs__msg__AutoaimTargetMsg
    std::shared_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sp_msgs__msg__AutoaimTargetMsg
    std::shared_ptr<sp_msgs::msg::AutoaimTargetMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AutoaimTargetMsg_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->target_ids != other.target_ids) {
      return false;
    }
    return true;
  }
  bool operator!=(const AutoaimTargetMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AutoaimTargetMsg_

// alias to use template instance with default allocator
using AutoaimTargetMsg =
  sp_msgs::msg::AutoaimTargetMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sp_msgs

#endif  // SP_MSGS__MSG__DETAIL__AUTOAIM_TARGET_MSG__STRUCT_HPP_
