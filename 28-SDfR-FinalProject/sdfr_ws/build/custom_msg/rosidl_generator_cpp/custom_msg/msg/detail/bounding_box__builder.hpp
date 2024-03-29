// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_ball_found
{
public:
  explicit Init_BoundingBox_ball_found(::custom_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::custom_msg::msg::BoundingBox ball_found(::custom_msg::msg::BoundingBox::_ball_found_type arg)
  {
    msg_.ball_found = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_height
{
public:
  explicit Init_BoundingBox_height(::custom_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_ball_found height(::custom_msg::msg::BoundingBox::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_BoundingBox_ball_found(msg_);
  }

private:
  ::custom_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_width
{
public:
  explicit Init_BoundingBox_width(::custom_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_height width(::custom_msg::msg::BoundingBox::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_BoundingBox_height(msg_);
  }

private:
  ::custom_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_center_point_y
{
public:
  explicit Init_BoundingBox_center_point_y(::custom_msg::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_width center_point_y(::custom_msg::msg::BoundingBox::_center_point_y_type arg)
  {
    msg_.center_point_y = std::move(arg);
    return Init_BoundingBox_width(msg_);
  }

private:
  ::custom_msg::msg::BoundingBox msg_;
};

class Init_BoundingBox_center_point_x
{
public:
  Init_BoundingBox_center_point_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_center_point_y center_point_x(::custom_msg::msg::BoundingBox::_center_point_x_type arg)
  {
    msg_.center_point_x = std::move(arg);
    return Init_BoundingBox_center_point_y(msg_);
  }

private:
  ::custom_msg::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::BoundingBox>()
{
  return custom_msg::msg::builder::Init_BoundingBox_center_point_x();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
