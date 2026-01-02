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
#include <crane_msgs/msg/velocity_commands.hpp>
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

    declare_parameter("i_saturation", I_SATURATION);
    I_SATURATION = get_parameter("i_saturation").as_double();

    // 全ロボットの状態を初期化
    for (auto & state : robot_states_) {
      state.vx_controller.setGain(
        p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
      state.vy_controller.setGain(
        p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
      state.theta_controller.setGain(
        theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      state.previous_velocity = Velocity::Zero();
    }

    // the parameters of the PID controller
    theta_p_gain.callback = [this](double) {
      for (auto & state : robot_states_) {
        state.theta_controller.setGain(
          theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };

    theta_i_gain.callback = [this](double) {
      for (auto & state : robot_states_) {
        state.theta_controller.setGain(
          theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };

    theta_d_gain.callback = [this](double) {
      for (auto & state : robot_states_) {
        state.theta_controller.setGain(
          theta_p_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };

    // ロボット送信用の加速度パラメータを読み込み
    declare_parameter("robot_acceleration.acceleration", 2.5);
    get_parameter("robot_acceleration.acceleration", robot_acceleration_acceleration_);

    declare_parameter("robot_acceleration.deceleration_high", 3.0);
    get_parameter("robot_acceleration.deceleration_high", robot_acceleration_deceleration_high_);

    declare_parameter("robot_acceleration.deceleration_low", 2.0);
    get_parameter("robot_acceleration.deceleration_low", robot_acceleration_deceleration_low_);

    declare_parameter("robot_acceleration.velocity_threshold", 1.5);
    get_parameter("robot_acceleration.velocity_threshold", robot_acceleration_velocity_threshold_);

    declare_parameter("chip_angle_deg", chip_angle_deg);
    chip_angle_deg = get_parameter("chip_angle_deg").as_double();
  }

  void sendCommands(const crane_msgs::msg::VelocityCommands & msg) override;

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

  std::unique_ptr<UDPSender> yellow_sender;
  std::unique_ptr<UDPSender> blue_sender;

  ParameterWithEvent<double> p_gain;
  ParameterWithEvent<double> i_gain;
  ParameterWithEvent<double> d_gain;

  ParameterWithEvent<double> theta_p_gain;
  ParameterWithEvent<double> theta_i_gain;
  ParameterWithEvent<double> theta_d_gain;

  double I_SATURATION = 0.0;

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

  // ロボット送信用の加速度パラメータ
  double robot_acceleration_acceleration_;
  double robot_acceleration_deceleration_high_;
  double robot_acceleration_deceleration_low_;
  double robot_acceleration_velocity_threshold_;

  // 加速度制限を計算
  double calculateAccelerationLimit(
    double current_velocity, double target_velocity, double robot_acceleration_acceleration,
    double robot_acceleration_deceleration_high, double robot_acceleration_deceleration_low,
    double robot_acceleration_velocity_threshold);
};
}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
