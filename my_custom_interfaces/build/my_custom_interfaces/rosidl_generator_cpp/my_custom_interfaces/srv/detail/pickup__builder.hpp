// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_custom_interfaces:srv/Pickup.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_custom_interfaces/srv/pickup.hpp"


#ifndef MY_CUSTOM_INTERFACES__SRV__DETAIL__PICKUP__BUILDER_HPP_
#define MY_CUSTOM_INTERFACES__SRV__DETAIL__PICKUP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_custom_interfaces/srv/detail/pickup__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_Pickup_Request_command
{
public:
  Init_Pickup_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_custom_interfaces::srv::Pickup_Request command(::my_custom_interfaces::srv::Pickup_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_custom_interfaces::srv::Pickup_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_custom_interfaces::srv::Pickup_Request>()
{
  return my_custom_interfaces::srv::builder::Init_Pickup_Request_command();
}

}  // namespace my_custom_interfaces


namespace my_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_Pickup_Response_message
{
public:
  explicit Init_Pickup_Response_message(::my_custom_interfaces::srv::Pickup_Response & msg)
  : msg_(msg)
  {}
  ::my_custom_interfaces::srv::Pickup_Response message(::my_custom_interfaces::srv::Pickup_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_custom_interfaces::srv::Pickup_Response msg_;
};

class Init_Pickup_Response_success
{
public:
  Init_Pickup_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pickup_Response_message success(::my_custom_interfaces::srv::Pickup_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Pickup_Response_message(msg_);
  }

private:
  ::my_custom_interfaces::srv::Pickup_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_custom_interfaces::srv::Pickup_Response>()
{
  return my_custom_interfaces::srv::builder::Init_Pickup_Response_success();
}

}  // namespace my_custom_interfaces


namespace my_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_Pickup_Event_response
{
public:
  explicit Init_Pickup_Event_response(::my_custom_interfaces::srv::Pickup_Event & msg)
  : msg_(msg)
  {}
  ::my_custom_interfaces::srv::Pickup_Event response(::my_custom_interfaces::srv::Pickup_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_custom_interfaces::srv::Pickup_Event msg_;
};

class Init_Pickup_Event_request
{
public:
  explicit Init_Pickup_Event_request(::my_custom_interfaces::srv::Pickup_Event & msg)
  : msg_(msg)
  {}
  Init_Pickup_Event_response request(::my_custom_interfaces::srv::Pickup_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Pickup_Event_response(msg_);
  }

private:
  ::my_custom_interfaces::srv::Pickup_Event msg_;
};

class Init_Pickup_Event_info
{
public:
  Init_Pickup_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pickup_Event_request info(::my_custom_interfaces::srv::Pickup_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Pickup_Event_request(msg_);
  }

private:
  ::my_custom_interfaces::srv::Pickup_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_custom_interfaces::srv::Pickup_Event>()
{
  return my_custom_interfaces::srv::builder::Init_Pickup_Event_info();
}

}  // namespace my_custom_interfaces

#endif  // MY_CUSTOM_INTERFACES__SRV__DETAIL__PICKUP__BUILDER_HPP_
