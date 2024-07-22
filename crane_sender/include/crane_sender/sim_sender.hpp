// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIM_SENDER_HPP_
#define CRANE_SENDER__SIM_SENDER_HPP_

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
struct ParameterWithEvent
{
  ParameterWithEvent(std::string name, rclcpp::Node & node) : name(name)
  {
    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(&node);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback(name, [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
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

class SimSenderComponent : public SenderBase
{
public:
  explicit SimSenderComponent(const rclcpp::NodeOptions & options)
  : SenderBase("sim_sender", options),
    pub_commands(create_publisher<robocup_ssl_msgs::msg::Commands>("/commands", 10)),
    p_gain("p_gain", *this),
    i_gain("i_gain", *this),
    d_gain("d_gain", *this)
  {
    declare_parameter("p_gain", 4.0);
    p_gain.value = get_parameter("p_gain").as_double();
    declare_parameter("i_gain", 0.0);
    i_gain.value = get_parameter("i_gain").as_double();
    declare_parameter("d_gain", 0.0);
    d_gain.value = get_parameter("d_gain").as_double();

    p_gain.callback = [&](double value) {
      for (auto & controller : vx_controllers) {
        controller.setGain(value, i_gain.getValue(), d_gain.getValue());
      }
      for (auto & controller : vy_controllers) {
        controller.setGain(value, i_gain.getValue(), d_gain.getValue());
      }
    };

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

    //  node.declare_parameter("p_gain", P_GAIN);
    //  P_GAIN = node.get_parameter("p_gain").as_double();
    //  node.declare_parameter("i_gain", I_GAIN);
    //  I_GAIN = node.get_parameter("i_gain").as_double();
    declare_parameter("i_saturation", I_SATURATION);
    I_SATURATION = get_parameter("i_saturation").as_double();
    //  node.declare_parameter("d_gain", D_GAIN);
    //  D_GAIN = node.get_parameter("d_gain").as_double();

    for (auto & controller : vx_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
    }

    for (auto & controller : vy_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
    }
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
      float omega = command.target_theta - command.current_pose.theta;
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
          if (vel.norm() > command.local_planner_config.max_velocity) {
            vel = vel.normalized() * command.local_planner_config.max_velocity;
          }
          Velocity vel_local;
          vel_local << vel.x() * cos(-command.current_pose.theta) -
                         vel.y() * sin(-command.current_pose.theta),
            vel.x() * sin(-command.current_pose.theta) + vel.y() * cos(-command.current_pose.theta);
          cmd.set__veltangent(vel_local.x());
          cmd.set__velnormal(vel_local.y());
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
      double kick_speed = command.kick_power * MAX_KICK_SPEED;

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

  ParameterWithEvent p_gain;
  ParameterWithEvent i_gain;
  ParameterWithEvent d_gain;

  //  double P_GAIN = 4.0;
  //  double I_GAIN = 0.0;
  double I_SATURATION = 0.0;
  //  double D_GAIN = 0.0;
};
}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
