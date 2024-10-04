// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__PARAMETER_WITH_EVENT_HPP_
#define CRANE_BASICS__PARAMETER_WITH_EVENT_HPP_

#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>

namespace crane
{
template <typename T>
struct ParameterWithEvent
{
  ParameterWithEvent(std::string name, rclcpp::Node & node, T default_value)
  : name(name), PARAMETER_TYPE([]() {
      if constexpr (std::is_same<T, bool>::value) {
        return rclcpp::ParameterType::PARAMETER_BOOL;
      } else if constexpr (std::is_same<T, int>::value) {
        return rclcpp::ParameterType::PARAMETER_INTEGER;
      } else if constexpr (std::is_same<T, double>::value) {
        return rclcpp::ParameterType::PARAMETER_DOUBLE;
      } else if constexpr (std::is_same<T, std::string>::value) {
        return rclcpp::ParameterType::PARAMETER_STRING;
      } else {
        throw std::runtime_error(
          "[ParameterWithEvent] Unknown parameter type: " + std::string(typeid(T).name()));
      }
    }())
  {
    node.declare_parameter(name, default_value);
    fetchParameter(node);

    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(&node);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback(name, [&](const rclcpp::Parameter & p) {
        if (p.get_type() == PARAMETER_TYPE && callback) {
          if constexpr (std::is_same<T, bool>::value) {
            value = p.as_bool();
          } else if constexpr (std::is_same<T, int>::value) {
            value = p.as_int();
          } else if constexpr (std::is_same<T, double>::value) {
            value = p.as_double();
          } else if constexpr (std::is_same<T, std::string>::value) {
            value = p.as_string();
          }

          callback(value);
        }
      });
  }

  void fetchParameter(rclcpp::Node & node)
  {
    if constexpr (std::is_same<T, bool>::value) {
      value = node.get_parameter(name).as_bool();
    } else if constexpr (std::is_same<T, int>::value) {
      value = node.get_parameter(name).as_int();
    } else if constexpr (std::is_same<T, double>::value) {
      value = node.get_parameter(name).as_double();
    } else if constexpr (std::is_same<T, std::string>::value) {
      value = node.get_parameter(name).as_string();
    }
  }

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  std::function<void(T)> callback;

  T getValue() { return value; }

  T value;

  std::string name;

  const int PARAMETER_TYPE;
};
}  // namespace crane
#endif  // CRANE_BASICS__PARAMETER_WITH_EVENT_HPP_
