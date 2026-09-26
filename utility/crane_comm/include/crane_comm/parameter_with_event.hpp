// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_COMM__PARAMETER_WITH_EVENT_HPP_
#define CRANE_COMM__PARAMETER_WITH_EVENT_HPP_

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
      if constexpr (std::is_same_v<T, bool>) {
        return rclcpp::ParameterType::PARAMETER_BOOL;
      } else if constexpr (std::is_same_v<T, int>) {
        return rclcpp::ParameterType::PARAMETER_INTEGER;
      } else if constexpr (std::is_same_v<T, double>) {
        return rclcpp::ParameterType::PARAMETER_DOUBLE;
      } else if constexpr (std::is_same_v<T, std::string>) {
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
        if (p.get_type() != PARAMETER_TYPE) return;
        value = fromParameter(p);
        if (callback) callback(value);
      });
  }

  auto fetchParameter(rclcpp::Node & node) -> void
  {
    value = fromParameter(node.get_parameter(name));
  }

  static auto fromParameter(const rclcpp::Parameter & p) -> T
  {
    if constexpr (std::is_same_v<T, bool>) {
      return p.as_bool();
    } else if constexpr (std::is_same_v<T, int>) {
      return p.as_int();
    } else if constexpr (std::is_same_v<T, double>) {
      return p.as_double();
    } else if constexpr (std::is_same_v<T, std::string>) {
      return p.as_string();
    }
  }

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  std::function<void(T)> callback;

  auto getValue() const -> T { return value; }

  T value;

  std::string name;

  const int PARAMETER_TYPE;
};
}  // namespace crane
#endif  // CRANE_COMM__PARAMETER_WITH_EVENT_HPP_
