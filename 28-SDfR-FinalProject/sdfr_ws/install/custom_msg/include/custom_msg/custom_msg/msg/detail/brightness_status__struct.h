// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:msg/BrightnessStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__STRUCT_H_
#define CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'brightness_status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BrightnessStatus in the package custom_msg.
/**
  * My custom message for brightness status
 */
typedef struct custom_msg__msg__BrightnessStatus
{
  rosidl_runtime_c__String brightness_status;
  int8_t light_level;
} custom_msg__msg__BrightnessStatus;

// Struct for a sequence of custom_msg__msg__BrightnessStatus.
typedef struct custom_msg__msg__BrightnessStatus__Sequence
{
  custom_msg__msg__BrightnessStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__msg__BrightnessStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__STRUCT_H_
