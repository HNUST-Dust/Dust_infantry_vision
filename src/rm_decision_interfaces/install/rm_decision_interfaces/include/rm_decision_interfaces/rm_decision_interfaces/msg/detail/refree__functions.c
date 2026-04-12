// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rm_decision_interfaces:msg/Refree.idl
// generated code does not contain a copyright notice
#include "rm_decision_interfaces/msg/detail/refree__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
rm_decision_interfaces__msg__Refree__init(rm_decision_interfaces__msg__Refree * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    rm_decision_interfaces__msg__Refree__fini(msg);
    return false;
  }
  // game_status
  // robot_hp
  // robot_status
  return true;
}

void
rm_decision_interfaces__msg__Refree__fini(rm_decision_interfaces__msg__Refree * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // game_status
  // robot_hp
  // robot_status
}

bool
rm_decision_interfaces__msg__Refree__are_equal(const rm_decision_interfaces__msg__Refree * lhs, const rm_decision_interfaces__msg__Refree * rhs)
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
  // game_status
  if (lhs->game_status != rhs->game_status) {
    return false;
  }
  // robot_hp
  for (size_t i = 0; i < 16; ++i) {
    if (lhs->robot_hp[i] != rhs->robot_hp[i]) {
      return false;
    }
  }
  // robot_status
  for (size_t i = 0; i < 16; ++i) {
    if (lhs->robot_status[i] != rhs->robot_status[i]) {
      return false;
    }
  }
  return true;
}

bool
rm_decision_interfaces__msg__Refree__copy(
  const rm_decision_interfaces__msg__Refree * input,
  rm_decision_interfaces__msg__Refree * output)
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
  // game_status
  output->game_status = input->game_status;
  // robot_hp
  for (size_t i = 0; i < 16; ++i) {
    output->robot_hp[i] = input->robot_hp[i];
  }
  // robot_status
  for (size_t i = 0; i < 16; ++i) {
    output->robot_status[i] = input->robot_status[i];
  }
  return true;
}

rm_decision_interfaces__msg__Refree *
rm_decision_interfaces__msg__Refree__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rm_decision_interfaces__msg__Refree * msg = (rm_decision_interfaces__msg__Refree *)allocator.allocate(sizeof(rm_decision_interfaces__msg__Refree), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rm_decision_interfaces__msg__Refree));
  bool success = rm_decision_interfaces__msg__Refree__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rm_decision_interfaces__msg__Refree__destroy(rm_decision_interfaces__msg__Refree * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rm_decision_interfaces__msg__Refree__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rm_decision_interfaces__msg__Refree__Sequence__init(rm_decision_interfaces__msg__Refree__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rm_decision_interfaces__msg__Refree * data = NULL;

  if (size) {
    data = (rm_decision_interfaces__msg__Refree *)allocator.zero_allocate(size, sizeof(rm_decision_interfaces__msg__Refree), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rm_decision_interfaces__msg__Refree__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rm_decision_interfaces__msg__Refree__fini(&data[i - 1]);
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
rm_decision_interfaces__msg__Refree__Sequence__fini(rm_decision_interfaces__msg__Refree__Sequence * array)
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
      rm_decision_interfaces__msg__Refree__fini(&array->data[i]);
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

rm_decision_interfaces__msg__Refree__Sequence *
rm_decision_interfaces__msg__Refree__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rm_decision_interfaces__msg__Refree__Sequence * array = (rm_decision_interfaces__msg__Refree__Sequence *)allocator.allocate(sizeof(rm_decision_interfaces__msg__Refree__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rm_decision_interfaces__msg__Refree__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rm_decision_interfaces__msg__Refree__Sequence__destroy(rm_decision_interfaces__msg__Refree__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rm_decision_interfaces__msg__Refree__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rm_decision_interfaces__msg__Refree__Sequence__are_equal(const rm_decision_interfaces__msg__Refree__Sequence * lhs, const rm_decision_interfaces__msg__Refree__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rm_decision_interfaces__msg__Refree__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rm_decision_interfaces__msg__Refree__Sequence__copy(
  const rm_decision_interfaces__msg__Refree__Sequence * input,
  rm_decision_interfaces__msg__Refree__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rm_decision_interfaces__msg__Refree);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rm_decision_interfaces__msg__Refree * data =
      (rm_decision_interfaces__msg__Refree *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rm_decision_interfaces__msg__Refree__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rm_decision_interfaces__msg__Refree__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rm_decision_interfaces__msg__Refree__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
