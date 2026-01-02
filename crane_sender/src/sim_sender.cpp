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
void SimSenderComponent::sendCommands(const crane_msgs::msg::VelocityCommands & msg)
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

    // VelocityCommand は常に極座標速度モード
    double v_r = command.target_velocity_r;
    double current_theta = command.current_pose.theta + omega * delay_s;
    double velocity_theta = command.target_velocity_theta - current_theta;

    double current_velocity = std::hypot(command.current_velocity.x, command.current_velocity.y);
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
