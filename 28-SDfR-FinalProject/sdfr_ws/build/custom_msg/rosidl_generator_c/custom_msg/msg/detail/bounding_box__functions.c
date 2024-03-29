// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#include "custom_msg/msg/detail/bounding_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_msg__msg__BoundingBox__init(custom_msg__msg__BoundingBox * msg)
{
  if (!msg) {
    return false;
  }
  // center_point_x
  // center_point_y
  // width
  // height
  // ball_found
  return true;
}

void
custom_msg__msg__BoundingBox__fini(custom_msg__msg__BoundingBox * msg)
{
  if (!msg) {
    return;
  }
  // center_point_x
  // center_point_y
  // width
  // height
  // ball_found
}

bool
custom_msg__msg__BoundingBox__are_equal(const custom_msg__msg__BoundingBox * lhs, const custom_msg__msg__BoundingBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // center_point_x
  if (lhs->center_point_x != rhs->center_point_x) {
    return false;
  }
  // center_point_y
  if (lhs->center_point_y != rhs->center_point_y) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // ball_found
  if (lhs->ball_found != rhs->ball_found) {
    return false;
  }
  return true;
}

bool
custom_msg__msg__BoundingBox__copy(
  const custom_msg__msg__BoundingBox * input,
  custom_msg__msg__BoundingBox * output)
{
  if (!input || !output) {
    return false;
  }
  // center_point_x
  output->center_point_x = input->center_point_x;
  // center_point_y
  output->center_point_y = input->center_point_y;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // ball_found
  output->ball_found = input->ball_found;
  return true;
}

custom_msg__msg__BoundingBox *
custom_msg__msg__BoundingBox__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__BoundingBox * msg = (custom_msg__msg__BoundingBox *)allocator.allocate(sizeof(custom_msg__msg__BoundingBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msg__msg__BoundingBox));
  bool success = custom_msg__msg__BoundingBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msg__msg__BoundingBox__destroy(custom_msg__msg__BoundingBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msg__msg__BoundingBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msg__msg__BoundingBox__Sequence__init(custom_msg__msg__BoundingBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__BoundingBox * data = NULL;

  if (size) {
    data = (custom_msg__msg__BoundingBox *)allocator.zero_allocate(size, sizeof(custom_msg__msg__BoundingBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msg__msg__BoundingBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msg__msg__BoundingBox__fini(&data[i - 1]);
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
custom_msg__msg__BoundingBox__Sequence__fini(custom_msg__msg__BoundingBox__Sequence * array)
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
      custom_msg__msg__BoundingBox__fini(&array->data[i]);
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

custom_msg__msg__BoundingBox__Sequence *
custom_msg__msg__BoundingBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__BoundingBox__Sequence * array = (custom_msg__msg__BoundingBox__Sequence *)allocator.allocate(sizeof(custom_msg__msg__BoundingBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msg__msg__BoundingBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msg__msg__BoundingBox__Sequence__destroy(custom_msg__msg__BoundingBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msg__msg__BoundingBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msg__msg__BoundingBox__Sequence__are_equal(const custom_msg__msg__BoundingBox__Sequence * lhs, const custom_msg__msg__BoundingBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msg__msg__BoundingBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msg__msg__BoundingBox__Sequence__copy(
  const custom_msg__msg__BoundingBox__Sequence * input,
  custom_msg__msg__BoundingBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msg__msg__BoundingBox);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msg__msg__BoundingBox * data =
      (custom_msg__msg__BoundingBox *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msg__msg__BoundingBox__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msg__msg__BoundingBox__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msg__msg__BoundingBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
