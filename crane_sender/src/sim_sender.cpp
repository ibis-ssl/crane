// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_sender/sim_sender.hpp"

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <iostream>

namespace crane
{
void SimSenderComponent::sendCommands(const crane_msgs::msg::RobotCommands & msg)
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
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
        double v_r = command.polar_velocity_target_mode.front().target_velocity_r;
        double current_theta = command.current_pose.theta + omega * delay_s;
        double velocity_theta =
          command.polar_velocity_target_mode.front().target_velocity_theta - current_theta;
        double vx = v_r * cos(velocity_theta);
        double vy = v_r * sin(velocity_theta);
        cmd.set__veltangent(vx);
        cmd.set__velnormal(vy);
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

  pub_commands.publish(commands);
}
}  // namespace crane
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SimSenderComponent)
