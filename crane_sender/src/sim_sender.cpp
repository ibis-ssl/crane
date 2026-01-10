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

    // VelocityCommand は極座標速度モード
    //
    // 【座標系の設計について】
    // RVO2Plannerから受け取る座標系：
    // - target_velocity_theta: theta_offsetを含むGLOBAL角度
    // - current_pose.theta: theta_offsetを含むGLOBAL角度
    // - current_velocity.x/y: フィールド座標系（theta_offset未適用）※参考用のみ
    //
    // 本実装では：
    // - velocity_theta = target_velocity_theta - current_theta でロボットローカル角度に変換
    // - ローカル座標系で速度ベクトルを構築してベクトル加速度制限を適用
    // - previous_velocity（前回の出力、ローカル座標）を現在速度として使用
    double current_theta = command.current_pose.theta + omega * delay_s;
    const double dt = 1.0 / 30.0;

    // Step 1: ロボットローカル座標系での目標速度ベクトルを構築
    double velocity_theta = command.target_velocity_theta - current_theta;
    Eigen::Vector2d target_vel_local(
      command.target_velocity_r * std::cos(velocity_theta),
      command.target_velocity_r * std::sin(velocity_theta));

    // Step 2: ローカル座標系での現在速度（前回の出力速度を使用）
    Eigen::Vector2d current_vel_local = robot_states_[command.robot_id].previous_velocity;

    // Step 3: 加速度制限を選択（速度の大きさで加速/減速を判定）
    double current_speed = current_vel_local.norm();
    double target_speed = target_vel_local.norm();
    double acceleration_limit = calculateAccelerationLimit(
      current_speed, target_speed, robot_acceleration_acceleration_,
      robot_acceleration_deceleration_high_, robot_acceleration_deceleration_low_,
      robot_acceleration_velocity_threshold_);

    // Step 4: ローカル座標系で速度変化ベクトルの大きさを制限（方向は維持）
    Eigen::Vector2d delta_vel = target_vel_local - current_vel_local;
    double max_delta_vel = acceleration_limit * dt;
    Eigen::Vector2d limited_vel_local;
    if (delta_vel.norm() > max_delta_vel) {
      limited_vel_local = current_vel_local + delta_vel.normalized() * max_delta_vel;
    } else {
      limited_vel_local = target_vel_local;
    }

    double vx = limited_vel_local.x();  // forward
    double vy = limited_vel_local.y();  // left

    move_local_velocity->set_forward(vx);
    move_local_velocity->set_left(vy);

    // 次回の計算用に保存
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
