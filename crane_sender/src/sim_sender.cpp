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
  auto & sender = msg.is_yellow ? yellow_sender : blue_sender;

  robocup_ssl::RobotControl packet;

  for (const auto & command : msg.robot_commands) {
    auto cmd = packet.add_robot_commands();
    cmd->set_id(command.robot_id);

    auto move_command = new robocup_ssl::RobotMoveCommand();
    auto move_local_velocity = new robocup_ssl::MoveLocalVelocity();

    float omega = robot_states_[command.robot_id].theta_controller.update(
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
        vel << robot_states_[command.robot_id].vx_controller.update(
          command.position_target_mode.front().target_x - command.current_pose.x, 1.f / 30.f),
          robot_states_[command.robot_id].vy_controller.update(
            command.position_target_mode.front().target_y - command.current_pose.y, 1.f / 30.f);
        vel += vel.normalized() * command.local_planner_config.terminal_velocity;

        // SimulationProtocolSenderの速度・加速度制限を追加
        double max_velocity = command.local_planner_config.final_planned_max_velocity.value;
        double current_velocity =
          std::hypot(command.current_velocity.x, command.current_velocity.y);
        max_velocity = std::min(
          max_velocity, current_velocity +
                          command.local_planner_config.final_planned_max_acceleration.value * 0.1);
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

        double current_velocity =
          std::hypot(command.current_velocity.x, command.current_velocity.y);
        double target_velocity = v_r;

        double acceleration_limit = calculateAccelerationLimit(
          current_velocity, target_velocity, robot_acceleration_acceleration_,
          robot_acceleration_deceleration_high_, robot_acceleration_deceleration_low_,
          robot_acceleration_velocity_threshold_);

        const double dt = 1.0 / 30.0;

        double max_velocity_by_acceleration = current_velocity + acceleration_limit * dt;

        double limited_velocity = std::min(target_velocity, max_velocity_by_acceleration);

        double vx = limited_velocity * cos(velocity_theta);
        double vy = limited_velocity * sin(velocity_theta);

        move_local_velocity->set_forward(vx);
        move_local_velocity->set_left(vy);

        robot_states_[command.robot_id].previous_velocity = Velocity(vx, vy);
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
    constexpr double MAX_KICK_SPEED = 8.0;  // m/s
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

double SimSenderComponent::calculateAccelerationLimit(
  double current_velocity, double target_velocity, double robot_acceleration_acceleration,
  double robot_acceleration_deceleration_high, double robot_acceleration_deceleration_low,
  double robot_acceleration_velocity_threshold)
{
  // ibis_sender_node.cppの行114-133と同じロジック
  double selected_acceleration;
  if (current_velocity < target_velocity) {
    // 加速時
    selected_acceleration = robot_acceleration_acceleration;
  } else {
    // 減速時：速度に応じて選択
    if (current_velocity >= robot_acceleration_velocity_threshold) {
      // 高速域
      selected_acceleration = robot_acceleration_deceleration_high;
    } else {
      // 低速域
      selected_acceleration = robot_acceleration_deceleration_low;
    }
  }
  return selected_acceleration;
}
}  // namespace crane
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SimSenderComponent)
