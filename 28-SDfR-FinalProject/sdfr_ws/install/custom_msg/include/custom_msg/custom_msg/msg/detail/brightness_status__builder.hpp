// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/BrightnessStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/brightness_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_BrightnessStatus_light_level
{
public:
  explicit Init_BrightnessStatus_light_level(::custom_msg::msg::BrightnessStatus & msg)
  : msg_(msg)
  {}
  ::custom_msg::msg::BrightnessStatus light_level(::custom_msg::msg::BrightnessStatus::_light_level_type arg)
  {
    msg_.light_level = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::BrightnessStatus msg_;
};

class Init_BrightnessStatus_brightness_status
{
public:
  Init_BrightnessStatus_brightness_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BrightnessStatus_light_level brightness_status(::custom_msg::msg::BrightnessStatus::_brightness_status_type arg)
  {
    msg_.brightness_status = std::move(arg);
    return Init_BrightnessStatus_light_level(msg_);
  }

private:
  ::custom_msg::msg::BrightnessStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::BrightnessStatus>()
{
  return custom_msg::msg::builder::Init_BrightnessStatus_brightness_status();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__BUILDER_HPP_
