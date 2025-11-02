// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIM_SENDER_HPP_
#define CRANE_SENDER__SIM_SENDER_HPP_

#include <crane_comm/diagnosed_publisher.hpp>
#include <crane_comm/parameter_with_event.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/commands.hpp>
#include <string>

#include "sender_base.hpp"

namespace crane
{
class SimSenderComponent : public SenderBase
{
public:
  explicit SimSenderComponent(const rclcpp::NodeOptions & options)
  : SenderBase("sim_sender", options),
    pub_commands(this, "/commands", 10, 50., 70.),
    p_gain("p_gain", *this, 4.0),
    i_gain("i_gain", *this, 0.0),
    d_gain("d_gain", *this, 0.0),
    theta_p_gain("theta_p_gain", *this, 4.0),
    theta_i_gain("theta_i_gain", *this, 0.0),
    theta_d_gain("theta_d_gain", *this, 0.1)
  {
    p_gain.callback = [&](double value) {
      for (auto & controller : vx_controllers) {
        controller.setGain(value, i_gain.getValue(), d_gain.getValue());
      }
      for (auto & controller : vy_controllers) {
        controller.setGain(value, i_gain.getValue(), d_gain.getValue());
      }
    };

    i_gain.callback = [&](double value) {
      for (auto & controller : vx_controllers) {
        controller.setGain(p_gain.getValue(), value, d_gain.getValue());
      }
      for (auto & controller : vy_controllers) {
        controller.setGain(p_gain.getValue(), value, d_gain.getValue());
      }
    };

    d_gain.callback = [&](double value) {
      for (auto & controller : vx_controllers) {
        controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
      }
      for (auto & controller : vy_controllers) {
        controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
      }
    };

    declare_parameter("i_saturation", I_SATURATION);
    I_SATURATION = get_parameter("i_saturation").as_double();

    for (auto & controller : vx_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
    }

    for (auto & controller : vy_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
    }

    for (auto & controller : theta_controllers) {
      controller.setGain(theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
    }
    // the parameters of the PID controller
    theta_p_gain.callback = [this](double) {
      for (auto & controller : theta_controllers) {
        controller.setGain(
          theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };

    theta_i_gain.callback = [this](double) {
      for (auto & controller : theta_controllers) {
        controller.setGain(
          theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };

    theta_d_gain.callback = [this](double) {
      for (auto & controller : theta_controllers) {
        controller.setGain(
          theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override;

  //  bool checkNan(const crane_msgs::msg::RobotCommands & msg)
  //  {
  //    bool is_nan = false;
  //    for (const auto & command : msg.robot_commands) {
  //      if (std::isnan(command.target_velocity.x)) {
  //        std::cout << "id: " << command.robot_id << " target_velocity.x is nan" << std::endl;
  //        is_nan = true;
  //      }
  //      if (std::isnan(command.target_velocity.y)) {
  //        std::cout << "id: " << command.robot_id << "target_velocity.y is nan" << std::endl;
  //        is_nan = true;
  //      }
  //      if (std::isnan(command.target_velocity.theta)) {
  //        std::cout << "id: " << command.robot_id << "target_velocity.theta is nan" << std::endl;
  //        is_nan = true;
  //      }
  //      if (std::isnan(command.kick_power)) {
  //        std::cout << "id: " << command.robot_id << "kick_power is nan" << std::endl;
  //        is_nan = true;
  //      }
  //      if (std::isnan(command.dribble_power)) {
  //        std::cout << "id: " << command.robot_id << "dribble_power is nan" << std::endl;
  //        is_nan = true;
  //      }
  //    }
  //    return is_nan;
  //  }

  DiagnosedPublisher<robocup_ssl_msgs::msg::Commands> pub_commands;

  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;
  std::array<PIDController, 20> theta_controllers;

  ParameterWithEvent<double> p_gain;
  ParameterWithEvent<double> i_gain;
  ParameterWithEvent<double> d_gain;

  ParameterWithEvent<double> theta_p_gain;
  ParameterWithEvent<double> theta_i_gain;
  ParameterWithEvent<double> theta_d_gain;

  double I_SATURATION = 0.0;
};
}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
