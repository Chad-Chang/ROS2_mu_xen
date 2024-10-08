// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_srv_action_interface_example:action/ArithmeticChecker.idl
// generated code does not contain a copyright notice

#ifndef MSG_SRV_ACTION_INTERFACE_EXAMPLE__ACTION__DETAIL__ARITHMETIC_CHECKER__TRAITS_HPP_
#define MSG_SRV_ACTION_INTERFACE_EXAMPLE__ACTION__DETAIL__ARITHMETIC_CHECKER__TRAITS_HPP_

#include "msg_srv_action_interface_example/action/detail/arithmetic_checker__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_Goal";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_Goal";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_Result>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_Result";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_Result>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_Result";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_Feedback";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_Feedback";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "msg_srv_action_interface_example/action/detail/arithmetic_checker__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_SendGoal_Request";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_SendGoal_Response";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_SendGoal";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>::value &&
    has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>::value &&
    has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<msg_srv_action_interface_example::action::ArithmeticChecker_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_GetResult_Request";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "msg_srv_action_interface_example/action/detail/arithmetic_checker__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_GetResult_Response";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_Result>::value> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_Result>::value> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_GetResult";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_GetResult";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>::value &&
    has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>::value &&
    has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>::value
  >
{
};

template<>
struct is_service<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<msg_srv_action_interface_example::action::ArithmeticChecker_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "msg_srv_action_interface_example/action/detail/arithmetic_checker__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msg_srv_action_interface_example::action::ArithmeticChecker_FeedbackMessage>()
{
  return "msg_srv_action_interface_example::action::ArithmeticChecker_FeedbackMessage";
}

template<>
inline const char * name<msg_srv_action_interface_example::action::ArithmeticChecker_FeedbackMessage>()
{
  return "msg_srv_action_interface_example/action/ArithmeticChecker_FeedbackMessage";
}

template<>
struct has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<msg_srv_action_interface_example::action::ArithmeticChecker_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<msg_srv_action_interface_example::action::ArithmeticChecker>
  : std::true_type
{
};

template<>
struct is_action_goal<msg_srv_action_interface_example::action::ArithmeticChecker_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<msg_srv_action_interface_example::action::ArithmeticChecker_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<msg_srv_action_interface_example::action::ArithmeticChecker_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // MSG_SRV_ACTION_INTERFACE_EXAMPLE__ACTION__DETAIL__ARITHMETIC_CHECKER__TRAITS_HPP_
