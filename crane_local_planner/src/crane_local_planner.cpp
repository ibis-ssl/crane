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
        what << "ロボット " << static_cast<int>(commands[i].robot_id) << " が重複しています(";
        what << commands[i].planner_name << ", " << aggregate_states(commands[i].state_factors)
             << "と" << commands[i - 1].planner_name << ", "
             << aggregate_states(commands[i - 1].state_factors) << ")";
             << commands[i].planner_name << ", " << aggregateStates(commands[i].state_factors)
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
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as LOCAL_CAMERA_MODE by \""
               << aggregate_states(raw_command.state_factors)
               << "\" skill , but no local_camera_mode is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        }
        break;
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        if (raw_command.position_target_mode.empty()) {
          is_valid = false;
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as POSITION_TARGET_MODE by \""
               << aggregate_states(raw_command.state_factors)
               << "\" skill , but no position_target_mode is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        } else {
          planner->visualizer->line()
            .start(raw_command.current_pose.x, raw_command.current_pose.y)
            .end(
              raw_command.position_target_mode.front().target_x,
              raw_command.position_target_mode.front().target_y)
            .stroke("yellow", 0.3)
            .strokeWidth(20)
            .build();
        }
        break;
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE:
        if (raw_command.simple_velocity_target_mode.empty()) {
          is_valid = false;
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as SIMPLE_VELOCITY_TARGET_MODE by \""
               << aggregate_states(raw_command.state_factors)
               << "\" skill , but simple_velocity_target_mode "
                  "is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        }
        break;
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE:
        if (raw_command.polar_velocity_target_mode.empty()) {
          is_valid = false;
          std::stringstream what;
          what << "The robot " << static_cast<int>(raw_command.robot_id)
               << " is specified as POLAR_VELOCITY_TARGET_MODE by \""
               << aggregate_states(raw_command.state_factors)
               << "\" skill , but no polar_velocity_target_mode is set.";
          RCLCPP_ERROR(get_logger(), what.str().c_str());
        }
        break;
      default:
        is_valid = false;
        std::stringstream what;
        what << "The robot " << static_cast<int>(raw_command.robot_id)
             << " is specified as an unknown control mode by \""
             << aggregate_states(raw_command.state_factors) << "\" skill.";
        RCLCPP_ERROR(get_logger(), what.str().c_str());
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

  for (const auto & command : pub_msg.robot_commands) {
    auto robot = world_model->getOurRobot(command.robot_id);
    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
        // planner->visualizer->line()
        //   .start(robot->pose.pos)
        //   .end(
        //     command.position_target_mode.front().target_x,
        //     command.position_target_mode.front().target_y)
        //   .stroke("yellow", 0.5)
        //   .strokeWidth(10)
        //   .build();
      } break;
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
        // planner->visualizer->line()
        //   .start(robot->pose.pos)
        //   .end(
        //     robot->pose.pos.x() + command.simple_velocity_target_mode.front().target_vx * 0.5,
        //     robot->pose.pos.y() + command.simple_velocity_target_mode.front().target_vy * 0.5)
        //   .stroke("white", 0.5)
        //   .strokeWidth(10)
        //   .build();
      } break;
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
        // planner->visualizer->line()
        //   .start(robot->pose.pos)
        //   .end(
        //     robot->pose.pos.x() +
        //       0.5 * command.polar_velocity_target_mode.front().target_velocity_r *
        //         std::cos(command.polar_velocity_target_mode.front().target_velocity_theta),
        //     robot->pose.pos.y() +
        //       0.5 * command.polar_velocity_target_mode.front().target_velocity_r *
        //         std::sin(command.polar_velocity_target_mode.front().target_velocity_theta))
        //   .stroke("white", 0.5)
        //   .strokeWidth(10)
        //   .build();
      } break;
    }
  }

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
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Planner not initialized");
    return;
  }

  auto & world_model = planner->world_model;
  if (not world_model) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "World model not available");
    return;
  }

  if (not world_model->hasUpdated()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "World model not updated yet");
    return;
  }

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Path planning is running normally");
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
