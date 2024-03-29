// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msg/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoundingBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: center_point_x
  {
    out << "center_point_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_point_x, out);
    out << ", ";
  }

  // member: center_point_y
  {
    out << "center_point_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_point_y, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: ball_found
  {
    out << "ball_found: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_found, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: center_point_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_point_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_point_x, out);
    out << "\n";
  }

  // member: center_point_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_point_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_point_y, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: ball_found
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_found: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_found, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoundingBox & msg, bool use_flow_style = false)
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
  const custom_msg::msg::BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msg::msg::BoundingBox & msg)
{
  return custom_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msg::msg::BoundingBox>()
{
  return "custom_msg::msg::BoundingBox";
}

template<>
inline const char * name<custom_msg::msg::BoundingBox>()
{
  return "custom_msg/msg/BoundingBox";
}

template<>
struct has_fixed_size<custom_msg::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msg::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msg::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
