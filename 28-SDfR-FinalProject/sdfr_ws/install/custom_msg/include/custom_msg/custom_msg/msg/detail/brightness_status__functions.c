// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msg:msg/BrightnessStatus.idl
// generated code does not contain a copyright notice
#include "custom_msg/msg/detail/brightness_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `brightness_status`
#include "rosidl_runtime_c/string_functions.h"

bool
custom_msg__msg__BrightnessStatus__init(custom_msg__msg__BrightnessStatus * msg)
{
  if (!msg) {
    return false;
  }
  // brightness_status
  if (!rosidl_runtime_c__String__init(&msg->brightness_status)) {
    custom_msg__msg__BrightnessStatus__fini(msg);
    return false;
  }
  // light_level
  return true;
}

void
custom_msg__msg__BrightnessStatus__fini(custom_msg__msg__BrightnessStatus * msg)
{
  if (!msg) {
    return;
  }
  // brightness_status
  rosidl_runtime_c__String__fini(&msg->brightness_status);
  // light_level
}

bool
custom_msg__msg__BrightnessStatus__are_equal(const custom_msg__msg__BrightnessStatus * lhs, const custom_msg__msg__BrightnessStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // brightness_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->brightness_status), &(rhs->brightness_status)))
  {
    return false;
  }
  // light_level
  if (lhs->light_level != rhs->light_level) {
    return false;
  }
  return true;
}

bool
custom_msg__msg__BrightnessStatus__copy(
  const custom_msg__msg__BrightnessStatus * input,
  custom_msg__msg__BrightnessStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // brightness_status
  if (!rosidl_runtime_c__String__copy(
      &(input->brightness_status), &(output->brightness_status)))
  {
    return false;
  }
  // light_level
  output->light_level = input->light_level;
  return true;
}

custom_msg__msg__BrightnessStatus *
custom_msg__msg__BrightnessStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__BrightnessStatus * msg = (custom_msg__msg__BrightnessStatus *)allocator.allocate(sizeof(custom_msg__msg__BrightnessStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msg__msg__BrightnessStatus));
  bool success = custom_msg__msg__BrightnessStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msg__msg__BrightnessStatus__destroy(custom_msg__msg__BrightnessStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msg__msg__BrightnessStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msg__msg__BrightnessStatus__Sequence__init(custom_msg__msg__BrightnessStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__BrightnessStatus * data = NULL;

  if (size) {
    data = (custom_msg__msg__BrightnessStatus *)allocator.zero_allocate(size, sizeof(custom_msg__msg__BrightnessStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msg__msg__BrightnessStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msg__msg__BrightnessStatus__fini(&data[i - 1]);
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
custom_msg__msg__BrightnessStatus__Sequence__fini(custom_msg__msg__BrightnessStatus__Sequence * array)
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
      custom_msg__msg__BrightnessStatus__fini(&array->data[i]);
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

custom_msg__msg__BrightnessStatus__Sequence *
custom_msg__msg__BrightnessStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__BrightnessStatus__Sequence * array = (custom_msg__msg__BrightnessStatus__Sequence *)allocator.allocate(sizeof(custom_msg__msg__BrightnessStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msg__msg__BrightnessStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msg__msg__BrightnessStatus__Sequence__destroy(custom_msg__msg__BrightnessStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msg__msg__BrightnessStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msg__msg__BrightnessStatus__Sequence__are_equal(const custom_msg__msg__BrightnessStatus__Sequence * lhs, const custom_msg__msg__BrightnessStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msg__msg__BrightnessStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msg__msg__BrightnessStatus__Sequence__copy(
  const custom_msg__msg__BrightnessStatus__Sequence * input,
  custom_msg__msg__BrightnessStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msg__msg__BrightnessStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msg__msg__BrightnessStatus * data =
      (custom_msg__msg__BrightnessStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msg__msg__BrightnessStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msg__msg__BrightnessStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msg__msg__BrightnessStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
