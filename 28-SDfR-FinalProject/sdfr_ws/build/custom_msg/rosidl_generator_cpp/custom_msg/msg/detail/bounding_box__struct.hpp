// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msg:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
#define CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msg__msg__BoundingBox __attribute__((deprecated))
#else
# define DEPRECATED__custom_msg__msg__BoundingBox __declspec(deprecated)
#endif

namespace custom_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox_
{
  using Type = BoundingBox_<ContainerAllocator>;

  explicit BoundingBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_point_x = 0.0f;
      this->center_point_y = 0.0f;
      this->width = 0.0f;
      this->height = 0.0f;
      this->ball_found = false;
    }
  }

  explicit BoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->center_point_x = 0.0f;
      this->center_point_y = 0.0f;
      this->width = 0.0f;
      this->height = 0.0f;
      this->ball_found = false;
    }
  }

  // field types and members
  using _center_point_x_type =
    float;
  _center_point_x_type center_point_x;
  using _center_point_y_type =
    float;
  _center_point_y_type center_point_y;
  using _width_type =
    float;
  _width_type width;
  using _height_type =
    float;
  _height_type height;
  using _ball_found_type =
    bool;
  _ball_found_type ball_found;

  // setters for named parameter idiom
  Type & set__center_point_x(
    const float & _arg)
  {
    this->center_point_x = _arg;
    return *this;
  }
  Type & set__center_point_y(
    const float & _arg)
  {
    this->center_point_y = _arg;
    return *this;
  }
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__ball_found(
    const bool & _arg)
  {
    this->ball_found = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msg::msg::BoundingBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msg::msg::BoundingBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::BoundingBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::BoundingBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msg__msg__BoundingBox
    std::shared_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msg__msg__BoundingBox
    std::shared_ptr<custom_msg::msg::BoundingBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox_ & other) const
  {
    if (this->center_point_x != other.center_point_x) {
      return false;
    }
    if (this->center_point_y != other.center_point_y) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->ball_found != other.ball_found) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox_

// alias to use template instance with default allocator
using BoundingBox =
  custom_msg::msg::BoundingBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
