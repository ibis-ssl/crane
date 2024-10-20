// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIM_SENDER_HPP_
#define CRANE_SENDER__SIM_SENDER_HPP_

#include <crane_basics/parameter_with_event.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/commands.hpp>
#include <robocup_ssl_msgs/msg/replacement.hpp>
#include <robocup_ssl_msgs/msg/robot_command.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "sender_base.hpp"

namespace crane
{
class SimSenderComponent : public SenderBase
{
public:
  explicit SimSenderComponent(const rclcpp::NodeOptions & options)
  : SenderBase("sim_sender", options),
    pub_commands(create_publisher<robocup_ssl_msgs::msg::Commands>("/commands", 10)),
    p_gain("p_gain", *this, 4.0),
    i_gain("i_gain", *this, 0.0),
    d_gain("d_gain", *this, 0.0),
    theta_k_gain("theta_k_gain", *this, 4.0),
    theta_i_gain("theta_i_gain", *this, 0.0),
    theta_d_gain("theta_p_gain", *this, 0.1)
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
      controller.setGain(theta_k_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
    }
    // the parameters of the PID controller
    theta_k_gain.callback = [this](double value) {
      for (auto & controller : theta_controllers) {
        controller.setGain(
          theta_k_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };

    theta_i_gain.callback = [this](double value) {
      for (auto & controller : theta_controllers) {
        controller.setGain(
          theta_k_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };

    theta_d_gain.callback = [this](double value) {
      for (auto & controller : theta_controllers) {
        controller.setGain(
          theta_k_gain.getValue(), theta_i_gain.getValue(), theta_d_gain.getValue());
      }
    };
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    //    if (checkNan(msg)) {
    //      return;
    //    }

    //    // 座標変換（ワールド->各ロボット）
    //    double vx = msg.target_velocity.x;
    //    double vy = msg.target_velocity.y;
    //    double omega = command.target_theta - command.current_pose.theta;
    //    double theta = command.current_pose.theta + omega * delay_s;
    //    command.target_velocity.x = vx * cos(-theta) - vy * sin(-theta);
    //    command.target_velocity.y = vx * sin(-theta) + vy * cos(-theta);

    //    command.target_velocity.theta =
    //      -theta_controllers.at(command.robot_id)
    //         .update(getAngleDiff(command.current_pose.theta, command.target_theta), 0.033);

    const double MAX_KICK_SPEED = 8.0;  // m/s
    robocup_ssl_msgs::msg::Commands commands;
    commands.isteamyellow = msg.is_yellow;
    commands.timestamp = msg.header.stamp.sec;

    for (const auto & command : msg.robot_commands) {
      robocup_ssl_msgs::msg::RobotCommand cmd;
      cmd.set__id(command.robot_id);
      float omega = theta_controllers[command.robot_id].update(
        -getAngleDiff(command.current_pose.theta, command.target_theta), 0.033);
      omega = std::clamp(omega, -command.omega_limit, command.omega_limit);
      cmd.set__velangular(omega);

      switch (command.control_mode) {
        case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE: {
          double vx = command.local_camera_mode.front().target_global_vx;
          double vy = command.local_camera_mode.front().target_global_vy;

          double theta = command.current_pose.theta + omega * delay_s;
          cmd.set__veltangent(vx * cos(-theta) - vy * sin(-theta));
          cmd.set__velnormal(vx * sin(-theta) + vy * cos(-theta));
        } break;
        case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
          Velocity vel;
          vel << vx_controllers[command.robot_id].update(
            command.position_target_mode.front().target_x - command.current_pose.x, 1.f / 30.f),
            vy_controllers[command.robot_id].update(
              command.position_target_mode.front().target_y - command.current_pose.y, 1.f / 30.f);
          vel += vel.normalized() * command.local_planner_config.terminal_velocity;
          double max_velocity = command.local_planner_config.max_velocity;
          double current_velocity =
            std::hypot(command.current_velocity.x, command.current_velocity.y);
          max_velocity = std::min(
            max_velocity, current_velocity + command.local_planner_config.max_acceleration * 0.1);
          if (vel.norm() > max_velocity) {
            vel = vel.normalized() * max_velocity;
          }
          Velocity vel_local;
          vel_local << vel.x() * cos(-command.current_pose.theta) -
                         vel.y() * sin(-command.current_pose.theta),
            vel.x() * sin(-command.current_pose.theta) + vel.y() * cos(-command.current_pose.theta);
          cmd.set__veltangent(vel_local.x());
          cmd.set__velnormal(vel_local.y());
        } break;
        case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
          double vx = command.simple_velocity_target_mode.front().target_vx;
          double vy = command.simple_velocity_target_mode.front().target_vy;
          double theta = command.current_pose.theta + omega * delay_s;
          cmd.set__veltangent(vx * cos(-theta) - vy * sin(-theta));
          cmd.set__velnormal(vx * sin(-theta) + vy * cos(-theta));
        } break;
        default:
          std::cout << "Invalid control mode" << std::endl;
          break;
      }

      // ストップ
      if (command.stop_flag) {
        cmd.set__veltangent(0);
        cmd.set__velnormal(0);
        cmd.set__velangular(0);
      }

      // キック速度
      double kick_speed = MAX_KICK_SPEED * command.kick_power;

      // チップキック
      if (command.chip_enable) {
        cmd.set__kickspeedx(kick_speed * 0.5);
        cmd.set__kickspeedz(kick_speed * 0.5);
      } else {
        cmd.set__kickspeedx(kick_speed * 1.0);
        cmd.set__kickspeedz(0);
      }

      // ドリブル
      cmd.set__spinner(command.dribble_power > 0);

      // タイヤ個別に速度設定しない
      cmd.set__wheelsspeed(false);

      if (no_movement) {
        cmd.set__spinner(false);
      }
      commands.robot_commands.emplace_back(cmd);
    }

    pub_commands->publish(commands);
  }

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

  const rclcpp::Publisher<robocup_ssl_msgs::msg::Commands>::SharedPtr pub_commands;

  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;
  std::array<PIDController, 20> theta_controllers;

  ParameterWithEvent<double> p_gain;
  ParameterWithEvent<double> i_gain;
  ParameterWithEvent<double> d_gain;

  ParameterWithEvent<double> theta_k_gain;
  ParameterWithEvent<double> theta_i_gain;
  ParameterWithEvent<double> theta_d_gain;

  double I_SATURATION = 0.0;
};
}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
