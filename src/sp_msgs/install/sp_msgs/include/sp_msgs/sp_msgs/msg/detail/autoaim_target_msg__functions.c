// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sp_msgs:msg/AutoaimTargetMsg.idl
// generated code does not contain a copyright notice
#include "sp_msgs/msg/detail/autoaim_target_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `target_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
sp_msgs__msg__AutoaimTargetMsg__init(sp_msgs__msg__AutoaimTargetMsg * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    sp_msgs__msg__AutoaimTargetMsg__fini(msg);
    return false;
  }
  // target_ids
  if (!rosidl_runtime_c__int8__Sequence__init(&msg->target_ids, 0)) {
    sp_msgs__msg__AutoaimTargetMsg__fini(msg);
    return false;
  }
  return true;
}

void
sp_msgs__msg__AutoaimTargetMsg__fini(sp_msgs__msg__AutoaimTargetMsg * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // target_ids
  rosidl_runtime_c__int8__Sequence__fini(&msg->target_ids);
}

bool
sp_msgs__msg__AutoaimTargetMsg__are_equal(const sp_msgs__msg__AutoaimTargetMsg * lhs, const sp_msgs__msg__AutoaimTargetMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // target_ids
  if (!rosidl_runtime_c__int8__Sequence__are_equal(
      &(lhs->target_ids), &(rhs->target_ids)))
  {
    return false;
  }
  return true;
}

bool
sp_msgs__msg__AutoaimTargetMsg__copy(
  const sp_msgs__msg__AutoaimTargetMsg * input,
  sp_msgs__msg__AutoaimTargetMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // target_ids
  if (!rosidl_runtime_c__int8__Sequence__copy(
      &(input->target_ids), &(output->target_ids)))
  {
    return false;
  }
  return true;
}

sp_msgs__msg__AutoaimTargetMsg *
sp_msgs__msg__AutoaimTargetMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sp_msgs__msg__AutoaimTargetMsg * msg = (sp_msgs__msg__AutoaimTargetMsg *)allocator.allocate(sizeof(sp_msgs__msg__AutoaimTargetMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sp_msgs__msg__AutoaimTargetMsg));
  bool success = sp_msgs__msg__AutoaimTargetMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sp_msgs__msg__AutoaimTargetMsg__destroy(sp_msgs__msg__AutoaimTargetMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sp_msgs__msg__AutoaimTargetMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sp_msgs__msg__AutoaimTargetMsg__Sequence__init(sp_msgs__msg__AutoaimTargetMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sp_msgs__msg__AutoaimTargetMsg * data = NULL;

  if (size) {
    data = (sp_msgs__msg__AutoaimTargetMsg *)allocator.zero_allocate(size, sizeof(sp_msgs__msg__AutoaimTargetMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sp_msgs__msg__AutoaimTargetMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sp_msgs__msg__AutoaimTargetMsg__fini(&data[i - 1]);
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
sp_msgs__msg__AutoaimTargetMsg__Sequence__fini(sp_msgs__msg__AutoaimTargetMsg__Sequence * array)
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
      sp_msgs__msg__AutoaimTargetMsg__fini(&array->data[i]);
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

sp_msgs__msg__AutoaimTargetMsg__Sequence *
sp_msgs__msg__AutoaimTargetMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sp_msgs__msg__AutoaimTargetMsg__Sequence * array = (sp_msgs__msg__AutoaimTargetMsg__Sequence *)allocator.allocate(sizeof(sp_msgs__msg__AutoaimTargetMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sp_msgs__msg__AutoaimTargetMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sp_msgs__msg__AutoaimTargetMsg__Sequence__destroy(sp_msgs__msg__AutoaimTargetMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sp_msgs__msg__AutoaimTargetMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sp_msgs__msg__AutoaimTargetMsg__Sequence__are_equal(const sp_msgs__msg__AutoaimTargetMsg__Sequence * lhs, const sp_msgs__msg__AutoaimTargetMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sp_msgs__msg__AutoaimTargetMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sp_msgs__msg__AutoaimTargetMsg__Sequence__copy(
  const sp_msgs__msg__AutoaimTargetMsg__Sequence * input,
  sp_msgs__msg__AutoaimTargetMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sp_msgs__msg__AutoaimTargetMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sp_msgs__msg__AutoaimTargetMsg * data =
      (sp_msgs__msg__AutoaimTargetMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sp_msgs__msg__AutoaimTargetMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sp_msgs__msg__AutoaimTargetMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sp_msgs__msg__AutoaimTargetMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
