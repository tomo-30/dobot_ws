// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dobot_msgs_v3:msg/ToolVectorActual.idl
// generated code does not contain a copyright notice

#ifndef DOBOT_MSGS_V3__MSG__DETAIL__TOOL_VECTOR_ACTUAL__STRUCT_HPP_
#define DOBOT_MSGS_V3__MSG__DETAIL__TOOL_VECTOR_ACTUAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dobot_msgs_v3__msg__ToolVectorActual __attribute__((deprecated))
#else
# define DEPRECATED__dobot_msgs_v3__msg__ToolVectorActual __declspec(deprecated)
#endif

namespace dobot_msgs_v3
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ToolVectorActual_
{
  using Type = ToolVectorActual_<ContainerAllocator>;

  explicit ToolVectorActual_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->rx = 0.0;
      this->ry = 0.0;
      this->rz = 0.0;
    }
  }

  explicit ToolVectorActual_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->rx = 0.0;
      this->ry = 0.0;
      this->rz = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _rx_type =
    double;
  _rx_type rx;
  using _ry_type =
    double;
  _ry_type ry;
  using _rz_type =
    double;
  _rz_type rz;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__rx(
    const double & _arg)
  {
    this->rx = _arg;
    return *this;
  }
  Type & set__ry(
    const double & _arg)
  {
    this->ry = _arg;
    return *this;
  }
  Type & set__rz(
    const double & _arg)
  {
    this->rz = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator> *;
  using ConstRawPtr =
    const dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dobot_msgs_v3__msg__ToolVectorActual
    std::shared_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dobot_msgs_v3__msg__ToolVectorActual
    std::shared_ptr<dobot_msgs_v3::msg::ToolVectorActual_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolVectorActual_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->rx != other.rx) {
      return false;
    }
    if (this->ry != other.ry) {
      return false;
    }
    if (this->rz != other.rz) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolVectorActual_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolVectorActual_

// alias to use template instance with default allocator
using ToolVectorActual =
  dobot_msgs_v3::msg::ToolVectorActual_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dobot_msgs_v3

#endif  // DOBOT_MSGS_V3__MSG__DETAIL__TOOL_VECTOR_ACTUAL__STRUCT_HPP_
