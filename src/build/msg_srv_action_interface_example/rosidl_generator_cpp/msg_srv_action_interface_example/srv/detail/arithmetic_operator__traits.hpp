// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_srv_action_interface_example:srv/ArithmeticOperator.idl
// generated code does not contain a copyright notice

#ifndef MSG_SRV_ACTION_INTERFACE_EXAMPLE__SRV__DETAIL__ARITHMETIC_OPERATOR__TRAITS_HPP_
#define MSG_SRV_ACTION_INTERFACE_EXAMPLE__SRV__DETAIL__ARITHMETIC_OPERATOR__TRAITS_HPP_

#include "msg_srv_action_interface_example/srv/detail/arithmetic_operator__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>()
{
  return "msg_srv_action_interface_example::srv::ArithmeticOperator_Request";
}

template<>
inline const char * name<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>()
{
  return "msg_srv_action_interface_example/srv/ArithmeticOperator_Request";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>()
{
  return "msg_srv_action_interface_example::srv::ArithmeticOperator_Response";
}

template<>
inline const char * name<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>()
{
  return "msg_srv_action_interface_example/srv/ArithmeticOperator_Response";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::srv::ArithmeticOperator>()
{
  return "msg_srv_action_interface_example::srv::ArithmeticOperator";
}

template<>
inline const char * name<msg_srv_action_interface_example::srv::ArithmeticOperator>()
{
  return "msg_srv_action_interface_example/srv/ArithmeticOperator";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::srv::ArithmeticOperator>
  : std::integral_constant<
    bool,
    has_fixed_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>::value &&
    has_fixed_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>::value
  >
{
};

template<>
struct has_bounded_size<msg_srv_action_interface_example::srv::ArithmeticOperator>
  : std::integral_constant<
    bool,
    has_bounded_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>::value &&
    has_bounded_size<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>::value
  >
{
};

template<>
struct is_service<msg_srv_action_interface_example::srv::ArithmeticOperator>
  : std::true_type
{
};

template<>
struct is_service_request<msg_srv_action_interface_example::srv::ArithmeticOperator_Request>
  : std::true_type
{
};

template<>
struct is_service_response<msg_srv_action_interface_example::srv::ArithmeticOperator_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MSG_SRV_ACTION_INTERFACE_EXAMPLE__SRV__DETAIL__ARITHMETIC_OPERATOR__TRAITS_HPP_
