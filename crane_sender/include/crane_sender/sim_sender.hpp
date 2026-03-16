// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIM_SENDER_HPP_
#define CRANE_SENDER__SIM_SENDER_HPP_

#include <robocup_ssl_msgs/ssl_simulation_robot_control.pb.h>

#include <crane_comm/parameter_with_event.hpp>
#include <crane_comm/udp_sender.hpp>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "sender_base.hpp"

namespace crane
{
class SimSenderComponent : public SenderBase
{
public:
  explicit SimSenderComponent(const rclcpp::NodeOptions & options)
  : SenderBase("sim_sender", options),
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
      for (auto & state : robot_states_) {
        state.vx_controller.setGain(value, i_gain.getValue(), d_gain.getValue());
        state.vy_controller.setGain(value, i_gain.getValue(), d_gain.getValue());
      }
    };

    i_gain.callback = [&](double value) {
      for (auto & state : robot_states_) {
        state.vx_controller.setGain(p_gain.getValue(), value, d_gain.getValue());
        state.vy_controller.setGain(p_gain.getValue(), value, d_gain.getValue());
      }
    };

    d_gain.callback = [&](double value) {
      for (auto & state : robot_states_) {
        state.vx_controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
        state.vy_controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
      }
    };

    declare_parameter("i_saturation", i_saturation_);
    i_saturation_ = get_parameter("i_saturation").as_double();

    // 全ロボットの状態を初期化
    for (auto & state : robot_states_) {
      state.vx_controller.setGain(
        p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), i_saturation_);
      state.vy_controller.setGain(
        p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), i_saturation_);
      state.theta_controller.setGain(
        theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      state.previous_velocity = Velocity::Zero();
    }

    // the parameters of the PID controller
    auto update_theta_gains = [this](double) {
      for (auto & state : robot_states_) {
        state.theta_controller.setGain(
          theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };
    theta_p_gain.callback = update_theta_gains;
    theta_i_gain.callback = update_theta_gains;
    theta_d_gain.callback = update_theta_gains;

    declare_parameter("chip_angle_deg", chip_angle_deg);
    chip_angle_deg = get_parameter("chip_angle_deg").as_double();
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override;

  std::unique_ptr<UDPSender> yellow_sender;
  std::unique_ptr<UDPSender> blue_sender;

  ParameterWithEvent<double> p_gain;
  ParameterWithEvent<double> i_gain;
  ParameterWithEvent<double> d_gain;

  ParameterWithEvent<double> theta_p_gain;
  ParameterWithEvent<double> theta_i_gain;
  ParameterWithEvent<double> theta_d_gain;

  double i_saturation_ = 0.0;

  double chip_angle_deg = 30.0;

private:
  // 各ロボットの状態を管理する構造体
  struct PerRobotState
  {
    PIDController vx_controller;
    PIDController vy_controller;
    PIDController theta_controller;
    Velocity previous_velocity;
  };

  // 全ロボットの状態
  std::array<PerRobotState, 20> robot_states_;
};
}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
