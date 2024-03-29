// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msg:msg/BrightnessStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__TRAITS_HPP_
#define CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msg/msg/detail/brightness_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const BrightnessStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: brightness_status
  {
    out << "brightness_status: ";
    rosidl_generator_traits::value_to_yaml(msg.brightness_status, out);
    out << ", ";
  }

  // member: light_level
  {
    out << "light_level: ";
    rosidl_generator_traits::value_to_yaml(msg.light_level, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BrightnessStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: brightness_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brightness_status: ";
    rosidl_generator_traits::value_to_yaml(msg.brightness_status, out);
    out << "\n";
  }

  // member: light_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "light_level: ";
    rosidl_generator_traits::value_to_yaml(msg.light_level, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BrightnessStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msg

namespace rosidl_generator_traits
{

[[deprecated("use custom_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msg::msg::BrightnessStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msg::msg::BrightnessStatus & msg)
{
  return custom_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msg::msg::BrightnessStatus>()
{
  return "custom_msg::msg::BrightnessStatus";
}

template<>
inline const char * name<custom_msg::msg::BrightnessStatus>()
{
  return "custom_msg/msg/BrightnessStatus";
}

template<>
struct has_fixed_size<custom_msg::msg::BrightnessStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msg::msg::BrightnessStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msg::msg::BrightnessStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__TRAITS_HPP_
