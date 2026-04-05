// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/robot_test_session.hpp>

namespace crane
{
RobotTestSession::RobotTestSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: SessionBase("robot_test_session", world_model),
  topics_interface_(node.get_node_topics_interface())
{
  target_sub_ = rclcpp::create_subscription<crane_msgs::msg::RobotCommand>(
    topics_interface_, "/robot_test/target", rclcpp::QoS(1),
    [this](crane_msgs::msg::RobotCommand::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(target_mutex_);
      latest_target_ = std::make_shared<crane_msgs::msg::RobotCommand>(*msg);
      target_robot_id_ = msg->robot_id;

      // max_velocity / max_acceleration をキャッシュ
      for (const auto & factor : msg->local_planner_config.max_velocity_factors) {
        if (factor.name == "robot_test") {
          max_velocity_ = factor.value;
        }
      }
      for (const auto & factor : msg->local_planner_config.max_acceleration_factors) {
        if (factor.name == "robot_test") {
          max_acceleration_ = factor.value;
        }
      }
    });
}

std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
RobotTestSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  if (robots.empty()) {
    return {SessionBase::Status::RUNNING, robot_commands};
  }

  auto robot_id = robots[0];

  if (!command_ || command_->getMsg().robot_id != robot_id.id) {
    command_ = std::make_shared<crane::PositionCommandWrapper>(
      "robot_test_session", robot_id.id, world_model);
  }

  crane_msgs::msg::RobotCommand::SharedPtr target;
  double max_vel, max_acc;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target = latest_target_;
    max_vel = max_velocity_;
    max_acc = max_acceleration_;
  }

  if (!target || target->position_target_mode.empty()) {
    // ターゲット未受信時は現在位置で停止
    command_->stopHere();
  } else {
    const auto & pos_target = target->position_target_mode[0];
    command_->clearMaxVelocityFactors()
      .clearMaxAccelerationFactors()
      .setMaxVelocity("robot_test_session", max_vel)
      .setMaxAcceleration("robot_test_session", max_acc)
      .setTargetPosition(pos_target.target_x, pos_target.target_y)
      .setTargetTheta(target->target_theta);
  }

  robot_commands.emplace_back(command_->getMsg());
  return {SessionBase::Status::RUNNING, robot_commands};
}

}  // namespace crane
