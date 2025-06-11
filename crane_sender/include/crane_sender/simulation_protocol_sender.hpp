// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIMULATION_PROTOCOL_SENDER_HPP_
#define CRANE_SENDER__SIMULATION_PROTOCOL_SENDER_HPP_

#include <robocup_ssl_msgs/ssl_simulation_robot_control.pb.h>

#include <crane_basics/parameter_with_event.hpp>
#include <crane_basics/udp_sender.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <iostream>
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

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    auto & sender = msg.is_yellow ? yellow_sender : blue_sender;

    RobotControl packet;

    for (const auto & command : msg.robot_commands) {
      auto cmd = packet.add_robot_commands();
      cmd->set_id(command.robot_id);

      auto move_command = new RobotMoveCommand();
      auto move_local_velocity = new MoveLocalVelocity();

      float omega = theta_controllers[command.robot_id].update(
        -getAngleDiff(command.current_pose.theta, command.target_theta), 0.033);
      omega = std::clamp(omega, -command.omega_limit, command.omega_limit);
      move_local_velocity->set_angular(omega);

      switch (command.control_mode) {
        case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE: {
          double vx = command.local_camera_mode.front().target_global_vx;
          double vy = command.local_camera_mode.front().target_global_vy;

          double theta = command.current_pose.theta + omega * delay_s;
          move_local_velocity->set_forward(vx * cos(-theta) - vy * sin(-theta));
          move_local_velocity->set_left(vx * sin(-theta) + vy * cos(-theta));
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
          move_local_velocity->set_forward(vel_local.x());
          move_local_velocity->set_left(vel_local.y());
        } break;
        case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
          double vx = command.simple_velocity_target_mode.front().target_vx;
          double vy = command.simple_velocity_target_mode.front().target_vy;
          double theta = command.current_pose.theta + omega * delay_s;
          move_local_velocity->set_forward(vx * cos(-theta) - vy * sin(-theta));
          move_local_velocity->set_left(vx * sin(-theta) + vy * cos(-theta));
        } break;
        case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
          double v_r = command.polar_velocity_target_mode.front().target_velocity_r;
          double current_theta = command.current_pose.theta + omega * delay_s;
          double velocity_theta =
            command.polar_velocity_target_mode.front().target_velocity_theta - current_theta;
          double vx = v_r * cos(velocity_theta);
          double vy = v_r * sin(velocity_theta);
          move_local_velocity->set_forward(vx);
          move_local_velocity->set_left(vy);
        } break;
        default:
          std::cout << "Invalid control mode" << std::endl;
          break;
      }

      // ストップ
      if (command.stop_flag) {
        move_local_velocity->set_forward(0);
        move_local_velocity->set_left(0);
        move_local_velocity->set_angular(0);
      }

      move_command->set_allocated_local_velocity(move_local_velocity);
      cmd->set_allocated_move_command(move_command);

      // キック速度
      constexpr double MAX_KICK_SPEED = 20.0;  // m/s
      double kick_speed = MAX_KICK_SPEED * command.kick_power;

      // チップキック
      if (command.chip_enable) {
        cmd->set_kick_angle(chip_angle_deg * M_PI / 180.);
        cmd->set_kick_speed(kick_speed * 0.5);
      } else {
        cmd->set_kick_angle(0.);
        cmd->set_kick_speed(kick_speed * 1.0);
      }

      // ドリブル(単位：rpm)
      cmd->set_dribbler_speed(command.dribble_power * 1000.);
    }

    std::string output;
    packet.SerializeToString(&output);
    sender->send(output);
  }

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
