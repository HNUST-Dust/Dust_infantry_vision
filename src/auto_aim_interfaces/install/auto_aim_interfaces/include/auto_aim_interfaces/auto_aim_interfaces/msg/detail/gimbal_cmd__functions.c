// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/gimbal_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
auto_aim_interfaces__msg__GimbalCmd__init(auto_aim_interfaces__msg__GimbalCmd * msg)
{
  if (!msg) {
    return false;
  }
  // yaw
  // pitch
  return true;
}

void
auto_aim_interfaces__msg__GimbalCmd__fini(auto_aim_interfaces__msg__GimbalCmd * msg)
{
  if (!msg) {
    return;
  }
  // yaw
  // pitch
}

bool
auto_aim_interfaces__msg__GimbalCmd__are_equal(const auto_aim_interfaces__msg__GimbalCmd * lhs, const auto_aim_interfaces__msg__GimbalCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__GimbalCmd__copy(
  const auto_aim_interfaces__msg__GimbalCmd * input,
  auto_aim_interfaces__msg__GimbalCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // pitch
  output->pitch = input->pitch;
  return true;
}

auto_aim_interfaces__msg__GimbalCmd *
auto_aim_interfaces__msg__GimbalCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__GimbalCmd * msg = (auto_aim_interfaces__msg__GimbalCmd *)allocator.allocate(sizeof(auto_aim_interfaces__msg__GimbalCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__GimbalCmd));
  bool success = auto_aim_interfaces__msg__GimbalCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__GimbalCmd__destroy(auto_aim_interfaces__msg__GimbalCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__GimbalCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__GimbalCmd__Sequence__init(auto_aim_interfaces__msg__GimbalCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__GimbalCmd * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__GimbalCmd *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__GimbalCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__GimbalCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__GimbalCmd__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__GimbalCmd__Sequence__fini(auto_aim_interfaces__msg__GimbalCmd__Sequence * array)
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
      auto_aim_interfaces__msg__GimbalCmd__fini(&array->data[i]);
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

auto_aim_interfaces__msg__GimbalCmd__Sequence *
auto_aim_interfaces__msg__GimbalCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__GimbalCmd__Sequence * array = (auto_aim_interfaces__msg__GimbalCmd__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__GimbalCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__GimbalCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__GimbalCmd__Sequence__destroy(auto_aim_interfaces__msg__GimbalCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__GimbalCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__GimbalCmd__Sequence__are_equal(const auto_aim_interfaces__msg__GimbalCmd__Sequence * lhs, const auto_aim_interfaces__msg__GimbalCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__GimbalCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__GimbalCmd__Sequence__copy(
  const auto_aim_interfaces__msg__GimbalCmd__Sequence * input,
  auto_aim_interfaces__msg__GimbalCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__GimbalCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__GimbalCmd * data =
      (auto_aim_interfaces__msg__GimbalCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__GimbalCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__GimbalCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__GimbalCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
