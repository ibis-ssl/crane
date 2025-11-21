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
auto LocalPlannerComponent::callbackRobotCommands(const crane_msgs::msg::RobotCommands & msg)
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
        std::stringstream what;
        what << "ロボット " << static_cast<int>(commands[i].robot_id) << " が重複しています("
             << commands[i].planner_name << ", " << aggregateStates(commands[i].state_factors)
             << " と " << commands[i - 1].planner_name << ", "
             << aggregateStates(commands[i - 1].state_factors) << ")";
        RCLCPP_ERROR(get_logger(), what.str().c_str());
      }
    }
  }

  // 各種制御モードをメッセージの内容を照合
  crane_msgs::msg::RobotCommands commands;
  for (const auto & raw_command : msg.robot_commands) {
    bool is_valid = true;
    switch (raw_command.control_mode) {
      case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE:
        if (raw_command.local_camera_mode.empty()) {
          is_valid = false;
          logValidationError(
            raw_command.robot_id, "LOCAL_CAMERA_MODE", raw_command.state_factors,
            "local_camera_mode が設定されていません。");
        }
        break;
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        if (raw_command.position_target_mode.empty()) {
          is_valid = false;
          logValidationError(
            raw_command.robot_id, "POSITION_TARGET_MODE", raw_command.state_factors,
            "position_target_mode が設定されていません。");
        } else {
          planner->visualizer->drawLine(
            Point(raw_command.current_pose.x, raw_command.current_pose.y),
            Point(
              raw_command.position_target_mode.front().target_x,
              raw_command.position_target_mode.front().target_y),
            "yellow", 20, 0.3);
        }
        break;
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE:
        if (raw_command.simple_velocity_target_mode.empty()) {
          is_valid = false;
          logValidationError(
            raw_command.robot_id, "SIMPLE_VELOCITY_TARGET_MODE", raw_command.state_factors,
            "simple_velocity_target_mode が設定されていません。");
        }
        break;
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE:
        if (raw_command.polar_velocity_target_mode.empty()) {
          is_valid = false;
          logValidationError(
            raw_command.robot_id, "POLAR_VELOCITY_TARGET_MODE", raw_command.state_factors,
            "polar_velocity_target_mode が設定されていません。");
        }
        break;
      default:
        is_valid = false;
        logValidationError(
          raw_command.robot_id, "不明な制御モード", raw_command.state_factors,
          "未知の制御モードです。");
        break;
    }
    // 一致しなかったらエラーメッセージ＆ロボットを待機状態にする
    if (is_valid) {
      crane_msgs::msg::RobotCommand command = raw_command;
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
  std::stringstream what;
  what << "ロボット " << static_cast<int>(robot_id) << " は \"" << aggregateStates(state_factors)
       << "\" スキルにより " << mode_name << " に指定されていますが、" << error_detail;
  RCLCPP_ERROR(get_logger(), what.str().c_str());
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
