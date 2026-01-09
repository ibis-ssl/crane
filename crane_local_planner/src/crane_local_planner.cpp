// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
#include <crane_utils/time.hpp>

#include "crane_local_planner/local_planner.hpp"

namespace crane
{
auto LocalPlannerComponent::callbackPositionCommands(const crane_msgs::msg::PositionCommands & msg)
  -> void
{
  auto & world_model = planner->world_model;
  if (not planner or not world_model or not world_model->hasUpdated()) {
    return;
  }
  ScopedTimer process_timer(process_time_pub);

  // 遅延監視用の遅延チェックポイント保存
  auto delay_checkpoints = msg.delay_checkpoints;
  DelayMonitorWrapper::addDelayCheckpoint(
    delay_checkpoints, "local_planner_start", "commands_received");

  // msg.robot_commands内のrobot_idダブリチェック
  {
    auto commands = msg.robot_commands;
    ranges::sort(commands, [](const auto & a, const auto & b) { return a.robot_id < b.robot_id; });
    for (size_t i = 1; i < commands.size(); i++) {
      if (commands[i - 1].robot_id == commands[i].robot_id) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "ロボット " << static_cast<int>(commands[i].robot_id)
                                    << " が重複しています(" << commands[i].planner_name << ", "
                                    << aggregateStates(commands[i].state_factors) << " と "
                                    << commands[i - 1].planner_name << ", "
                                    << aggregateStates(commands[i - 1].state_factors) << ")");
      }
    }
  }

  // 位置指令を検証して処理
  crane_msgs::msg::PositionCommands commands;
  for (const auto & raw_command : msg.robot_commands) {
    // 位置目標の可視化
    planner->visualizer->drawLine(
      Point(raw_command.current_pose.x, raw_command.current_pose.y),
      Point(raw_command.target_x, raw_command.target_y), "yellow", 20, 0.3);

    crane_msgs::msg::PositionCommand command = raw_command;
    auto robot = world_model->getOurRobot(command.robot_id);
    command.current_pose.x = robot->pose.pos.x();
    command.current_pose.y = robot->pose.pos.y();
    command.current_pose.theta = robot->pose.theta + theta_offset;
    command.current_velocity.x = robot->vel.linear.x();
    command.current_velocity.y = robot->vel.linear.y();
    command.current_velocity.theta = robot->vel.omega;
    command.target_theta += theta_offset;
    commands.robot_commands.push_back(command);
  }

  // キックパワーの調整
  for (auto & command : commands.robot_commands) {
    command.kick_power = kick_power_calculator.getKickPower(command);
  }

  auto pub_msg = planner->calculateRobotCommand(commands, theta_offset);
  pub_msg.header.stamp = rclcpp::Clock().now();
  pub_msg.is_yellow = world_model->isYellow();

  // 遅延監視: LocalPlanner処理完了、最終コマンド送信
  pub_msg.delay_checkpoints = delay_checkpoints;
  DelayMonitorWrapper::addDelayCheckpoint(
    pub_msg.delay_checkpoints, "local_planner_end", "path_planned");

  commands_pub.publish(pub_msg);

  planner->visualizer->flush();
  crane::CraneVisualizerBuffer::publish();

  // 診断情報を更新
  diagnostic_updater_.force_update();
}

auto LocalPlannerComponent::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  -> void
{
  // プランナーの状態をチェック
  if (!planner) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "プランナーが初期化されていません");
    return;
  }

  auto & world_model = planner->world_model;
  if (not world_model) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "ワールドモデルが利用できません");
    return;
  }

  if (not world_model->hasUpdated()) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "ワールドモデルがまだ更新されていません");
    return;
  }

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "経路計画は正常に動作しています");
}

auto LocalPlannerComponent::aggregateStates(
  const std::vector<crane_msgs::msg::NamedString> & state_factors) const -> std::string
{
  std::stringstream ss;
  ss << "[";
  for (const auto & state_factor : state_factors) {
    ss << state_factor.name << ":" << state_factor.value << ", ";
  }
  ss << "]";
  return ss.str();
}

auto LocalPlannerComponent::logValidationError(
  uint8_t robot_id, const std::string & mode_name,
  const std::vector<crane_msgs::msg::NamedString> & state_factors,
  const std::string & error_detail) const -> void
{
  RCLCPP_ERROR_STREAM(
    get_logger(), "ロボット " << static_cast<int>(robot_id) << " は \""
                              << aggregateStates(state_factors) << "\" スキルにより " << mode_name
                              << " に指定されていますが、" << error_detail);
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
