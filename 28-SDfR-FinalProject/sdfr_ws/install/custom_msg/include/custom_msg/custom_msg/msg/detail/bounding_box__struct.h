// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BoundingBox in the package custom_msg.
/**
  * BoundingBox.msg
 */
typedef struct custom_msg__msg__BoundingBox
{
  float center_point_x;
  float center_point_y;
  float width;
  float height;
  bool ball_found;
} custom_msg__msg__BoundingBox;

// Struct for a sequence of custom_msg__msg__BoundingBox.
typedef struct custom_msg__msg__BoundingBox__Sequence
{
  custom_msg__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
