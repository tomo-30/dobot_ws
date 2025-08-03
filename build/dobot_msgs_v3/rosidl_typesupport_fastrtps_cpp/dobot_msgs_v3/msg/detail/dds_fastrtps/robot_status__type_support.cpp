// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v3:msg/RobotStatus.idl
// generated code does not contain a copyright notice
#include "dobot_msgs_v3/msg/detail/robot_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "dobot_msgs_v3/msg/detail/robot_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace dobot_msgs_v3
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3
cdr_serialize(
  const dobot_msgs_v3::msg::RobotStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: is_enable
  cdr << (ros_message.is_enable ? true : false);
  // Member: is_connected
  cdr << (ros_message.is_connected ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dobot_msgs_v3::msg::RobotStatus & ros_message)
{
  // Member: is_enable
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_enable = tmp ? true : false;
  }

  // Member: is_connected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_connected = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3
get_serialized_size(
  const dobot_msgs_v3::msg::RobotStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: is_enable
  {
    size_t item_size = sizeof(ros_message.is_enable);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_connected
  {
    size_t item_size = sizeof(ros_message.is_connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dobot_msgs_v3
max_serialized_size_RobotStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: is_enable
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: is_connected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dobot_msgs_v3::msg::RobotStatus;
    is_plain =
      (
      offsetof(DataType, is_connected) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RobotStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const dobot_msgs_v3::msg::RobotStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RobotStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<dobot_msgs_v3::msg::RobotStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RobotStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const dobot_msgs_v3::msg::RobotStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RobotStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RobotStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RobotStatus__callbacks = {
  "dobot_msgs_v3::msg",
  "RobotStatus",
  _RobotStatus__cdr_serialize,
  _RobotStatus__cdr_deserialize,
  _RobotStatus__get_serialized_size,
  _RobotStatus__max_serialized_size
};

static rosidl_message_type_support_t _RobotStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RobotStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dobot_msgs_v3

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dobot_msgs_v3
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v3::msg::RobotStatus>()
{
  return &dobot_msgs_v3::msg::typesupport_fastrtps_cpp::_RobotStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v3, msg, RobotStatus)() {
  return &dobot_msgs_v3::msg::typesupport_fastrtps_cpp::_RobotStatus__handle;
}

#ifdef __cplusplus
}
#endif
