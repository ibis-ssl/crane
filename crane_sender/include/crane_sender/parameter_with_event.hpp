// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__PARAMETER_WITH_EVENT_HPP_
#define CRANE_SENDER__PARAMETER_WITH_EVENT_HPP_
namespace crane
{
struct ParameterWithEvent
{
  ParameterWithEvent(std::string name, rclcpp::Node & node) : name(name)
  {
    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(&node);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback(name, [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE && callback) {
          value = p.as_double();
          callback(value);
        } else {
          std::cout << "debug_id is not integer" << std::endl;
        }
      });
  }

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  std::function<void(double)> callback;

  double getValue() { return value; }

  double value;

  std::string name;
};
}  // namespace crane
#endif  // CRANE_SENDER__PARAMETER_WITH_EVENT_HPP_
