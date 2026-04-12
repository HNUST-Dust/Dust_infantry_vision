// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/Measurement.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'measurement'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__Measurement __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__Measurement __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Measurement_
{
  using Type = Measurement_<ContainerAllocator>;

  explicit Measurement_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : measurement(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0f;
    }
  }

  explicit Measurement_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : measurement(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0f;
    }
  }

  // field types and members
  using _measurement_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _measurement_type measurement;
  using _timestamp_type =
    float;
  _timestamp_type timestamp;

  // setters for named parameter idiom
  Type & set__measurement(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->measurement = _arg;
    return *this;
  }
  Type & set__timestamp(
    const float & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::Measurement_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::Measurement_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Measurement_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Measurement_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__Measurement
    std::shared_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__Measurement
    std::shared_ptr<auto_aim_interfaces::msg::Measurement_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Measurement_ & other) const
  {
    if (this->measurement != other.measurement) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Measurement_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Measurement_

// alias to use template instance with default allocator
using Measurement =
  auto_aim_interfaces::msg::Measurement_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__MEASUREMENT__STRUCT_HPP_
