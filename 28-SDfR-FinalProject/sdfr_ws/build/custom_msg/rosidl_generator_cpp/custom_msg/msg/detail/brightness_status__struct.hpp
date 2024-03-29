// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msg:msg/BrightnessStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__STRUCT_HPP_
#define CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msg__msg__BrightnessStatus __attribute__((deprecated))
#else
# define DEPRECATED__custom_msg__msg__BrightnessStatus __declspec(deprecated)
#endif

namespace custom_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BrightnessStatus_
{
  using Type = BrightnessStatus_<ContainerAllocator>;

  explicit BrightnessStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->brightness_status = "";
      this->light_level = 0;
    }
  }

  explicit BrightnessStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : brightness_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->brightness_status = "";
      this->light_level = 0;
    }
  }

  // field types and members
  using _brightness_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _brightness_status_type brightness_status;
  using _light_level_type =
    int8_t;
  _light_level_type light_level;

  // setters for named parameter idiom
  Type & set__brightness_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->brightness_status = _arg;
    return *this;
  }
  Type & set__light_level(
    const int8_t & _arg)
  {
    this->light_level = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msg::msg::BrightnessStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msg::msg::BrightnessStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::BrightnessStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::BrightnessStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msg__msg__BrightnessStatus
    std::shared_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msg__msg__BrightnessStatus
    std::shared_ptr<custom_msg::msg::BrightnessStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BrightnessStatus_ & other) const
  {
    if (this->brightness_status != other.brightness_status) {
      return false;
    }
    if (this->light_level != other.light_level) {
      return false;
    }
    return true;
  }
  bool operator!=(const BrightnessStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BrightnessStatus_

// alias to use template instance with default allocator
using BrightnessStatus =
  custom_msg::msg::BrightnessStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__BRIGHTNESS_STATUS__STRUCT_HPP_
