// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__Target __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__Target __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Target_
{
  using Type = Target_<ContainerAllocator>;

  explicit Target_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    velocity(_init)
  {
    (void)_init;
  }

  explicit Target_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    velocity(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::Target_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::Target_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Target_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Target_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__Target
    std::shared_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__Target
    std::shared_ptr<auto_aim_interfaces::msg::Target_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_

// alias to use template instance with default allocator
using Target =
  auto_aim_interfaces::msg::Target_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__STRUCT_HPP_
