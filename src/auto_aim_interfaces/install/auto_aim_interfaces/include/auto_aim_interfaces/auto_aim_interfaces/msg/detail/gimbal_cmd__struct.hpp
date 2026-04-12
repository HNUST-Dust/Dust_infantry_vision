// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__GimbalCmd __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__GimbalCmd __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GimbalCmd_
{
  using Type = GimbalCmd_<ContainerAllocator>;

  explicit GimbalCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0f;
      this->pitch = 0.0f;
    }
  }

  explicit GimbalCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0f;
      this->pitch = 0.0f;
    }
  }

  // field types and members
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _pitch_type =
    float;
  _pitch_type pitch;

  // setters for named parameter idiom
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__GimbalCmd
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__GimbalCmd
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalCmd_ & other) const
  {
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalCmd_

// alias to use template instance with default allocator
using GimbalCmd =
  auto_aim_interfaces::msg::GimbalCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_HPP_
