// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_srv_action_interface_example:msg/ArithmeticArgument.idl
// generated code does not contain a copyright notice

#ifndef MSG_SRV_ACTION_INTERFACE_EXAMPLE__MSG__DETAIL__ARITHMETIC_ARGUMENT__TRAITS_HPP_
#define MSG_SRV_ACTION_INTERFACE_EXAMPLE__MSG__DETAIL__ARITHMETIC_ARGUMENT__TRAITS_HPP_

#include "msg_srv_action_interface_example/msg/detail/arithmetic_argument__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::msg::ArithmeticArgument>()
{
  return "msg_srv_action_interface_example::msg::ArithmeticArgument";
}

template<>
inline const char * name<msg_srv_action_interface_example::msg::ArithmeticArgument>()
{
  return "msg_srv_action_interface_example/msg/ArithmeticArgument";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::msg::ArithmeticArgument>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::msg::ArithmeticArgument>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<msg_srv_action_interface_example::msg::ArithmeticArgument>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_SRV_ACTION_INTERFACE_EXAMPLE__MSG__DETAIL__ARITHMETIC_ARGUMENT__TRAITS_HPP_
