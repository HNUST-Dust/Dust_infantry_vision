// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/Target.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
auto_aim_interfaces__msg__Target__init(auto_aim_interfaces__msg__Target * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    auto_aim_interfaces__msg__Target__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__init(&msg->velocity)) {
    auto_aim_interfaces__msg__Target__fini(msg);
    return false;
  }
  return true;
}

void
auto_aim_interfaces__msg__Target__fini(auto_aim_interfaces__msg__Target * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // velocity
  geometry_msgs__msg__Twist__fini(&msg->velocity);
}

bool
auto_aim_interfaces__msg__Target__are_equal(const auto_aim_interfaces__msg__Target * lhs, const auto_aim_interfaces__msg__Target * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__Target__copy(
  const auto_aim_interfaces__msg__Target * input,
  auto_aim_interfaces__msg__Target * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  return true;
}

auto_aim_interfaces__msg__Target *
auto_aim_interfaces__msg__Target__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Target * msg = (auto_aim_interfaces__msg__Target *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Target), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__Target));
  bool success = auto_aim_interfaces__msg__Target__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__Target__destroy(auto_aim_interfaces__msg__Target * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__Target__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__Target__Sequence__init(auto_aim_interfaces__msg__Target__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Target * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__Target *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__Target), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__Target__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__Target__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
auto_aim_interfaces__msg__Target__Sequence__fini(auto_aim_interfaces__msg__Target__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      auto_aim_interfaces__msg__Target__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

auto_aim_interfaces__msg__Target__Sequence *
auto_aim_interfaces__msg__Target__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Target__Sequence * array = (auto_aim_interfaces__msg__Target__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Target__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__Target__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__Target__Sequence__destroy(auto_aim_interfaces__msg__Target__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__Target__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__Target__Sequence__are_equal(const auto_aim_interfaces__msg__Target__Sequence * lhs, const auto_aim_interfaces__msg__Target__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__Target__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__Target__Sequence__copy(
  const auto_aim_interfaces__msg__Target__Sequence * input,
  auto_aim_interfaces__msg__Target__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__Target);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__Target * data =
      (auto_aim_interfaces__msg__Target *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__Target__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__Target__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__Target__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
