// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIMULATION_PROTOCOL_SENDER_HPP_
#define CRANE_SENDER__SIMULATION_PROTOCOL_SENDER_HPP_

#include <robocup_ssl_msgs/ssl_simulation_robot_control.pb.h>

#include <crane_comm/parameter_with_event.hpp>
#include <crane_comm/udp_sender.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "sender_base.hpp"

namespace crane
{
class SimulationProtocolSenderComponent : public SenderBase
{
public:
  explicit SimulationProtocolSenderComponent(const rclcpp::NodeOptions & options)
  : SenderBase("simulation_protocol_sender", options),
    p_gain("p_gain", *this, 4.0),
    i_gain("i_gain", *this, 0.0),
    d_gain("d_gain", *this, 0.0),
    theta_p_gain("theta_p_gain", *this, 4.0),
    theta_i_gain("theta_i_gain", *this, 0.0),
    theta_d_gain("theta_d_gain", *this, 0.1)
  {
    blue_sender = std::make_unique<UDPSender>("127.0.0.1", 10301);
    yellow_sender = std::make_unique<UDPSender>("127.0.0.1", 10302);
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

    declare_parameter("chip_angle_deg", chip_angle_deg);
    chip_angle_deg = get_parameter("chip_angle_deg").as_double();
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override;

  std::unique_ptr<UDPSender> yellow_sender;
  std::unique_ptr<UDPSender> blue_sender;

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

  double chip_angle_deg = 30.0;
};
}  // namespace crane
#endif  // CRANE_SENDER__SIMULATION_PROTOCOL_SENDER_HPP_
