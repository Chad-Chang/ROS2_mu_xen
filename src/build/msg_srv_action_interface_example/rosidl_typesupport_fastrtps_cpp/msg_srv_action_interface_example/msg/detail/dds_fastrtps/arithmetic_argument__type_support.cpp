// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from msg_srv_action_interface_example:msg/ArithmeticArgument.idl
// generated code does not contain a copyright notice
#include "msg_srv_action_interface_example/msg/detail/arithmetic_argument__rosidl_typesupport_fastrtps_cpp.hpp"
#include "msg_srv_action_interface_example/msg/detail/arithmetic_argument__struct.hpp"

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
namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Time &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  builtin_interfaces::msg::Time &);
size_t get_serialized_size(
  const builtin_interfaces::msg::Time &,
  size_t current_alignment);
size_t
max_serialized_size_Time(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace builtin_interfaces


namespace msg_srv_action_interface_example
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_srv_action_interface_example
cdr_serialize(
  const msg_srv_action_interface_example::msg::ArithmeticArgument & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.stamp,
    cdr);
  // Member: argument_a
  cdr << ros_message.argument_a;
  // Member: argument_b
  cdr << ros_message.argument_b;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_srv_action_interface_example
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msg_srv_action_interface_example::msg::ArithmeticArgument & ros_message)
{
  // Member: stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.stamp);

  // Member: argument_a
  cdr >> ros_message.argument_a;

  // Member: argument_b
  cdr >> ros_message.argument_b;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_srv_action_interface_example
get_serialized_size(
  const msg_srv_action_interface_example::msg::ArithmeticArgument & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: stamp

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.stamp, current_alignment);
  // Member: argument_a
  {
    size_t item_size = sizeof(ros_message.argument_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: argument_b
  {
    size_t item_size = sizeof(ros_message.argument_b);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msg_srv_action_interface_example
max_serialized_size_ArithmeticArgument(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: stamp
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        builtin_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        full_bounded, current_alignment);
    }
  }

  // Member: argument_a
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: argument_b
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _ArithmeticArgument__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const msg_srv_action_interface_example::msg::ArithmeticArgument *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ArithmeticArgument__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<msg_srv_action_interface_example::msg::ArithmeticArgument *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ArithmeticArgument__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const msg_srv_action_interface_example::msg::ArithmeticArgument *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ArithmeticArgument__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ArithmeticArgument(full_bounded, 0);
}

static message_type_support_callbacks_t _ArithmeticArgument__callbacks = {
  "msg_srv_action_interface_example::msg",
  "ArithmeticArgument",
  _ArithmeticArgument__cdr_serialize,
  _ArithmeticArgument__cdr_deserialize,
  _ArithmeticArgument__get_serialized_size,
  _ArithmeticArgument__max_serialized_size
};

static rosidl_message_type_support_t _ArithmeticArgument__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ArithmeticArgument__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace msg_srv_action_interface_example

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_msg_srv_action_interface_example
const rosidl_message_type_support_t *
get_message_type_support_handle<msg_srv_action_interface_example::msg::ArithmeticArgument>()
{
  return &msg_srv_action_interface_example::msg::typesupport_fastrtps_cpp::_ArithmeticArgument__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msg_srv_action_interface_example, msg, ArithmeticArgument)() {
  return &msg_srv_action_interface_example::msg::typesupport_fastrtps_cpp::_ArithmeticArgument__handle;
}

#ifdef __cplusplus
}
#endif
