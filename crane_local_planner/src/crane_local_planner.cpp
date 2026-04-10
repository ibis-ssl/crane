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
auto LocalPlannerComponent::callbackPositionCommands(const crane_msgs::msg::RobotCommands & msg)
  -> void
{
  if (!planner) {
    return;
  }
  auto world_model = planner->getWorldModel();
  if (!world_model || !world_model->hasUpdated()) {
    return;
  }

  dropped_command_count_last_cycle_ = 0;
  planner_exception_count_last_cycle_ = 0;

  auto publishFallback = [&]() {
    planner_exception_count_last_cycle_++;
    planner_exception_count_total_++;
    crane_msgs::msg::RobotCommands fallback_msg;
    fallback_msg.header.stamp = now();
    fallback_msg.is_yellow = planner->getWorldModel()->isYellow();
    commands_pub.publish(fallback_msg);
  };

  try {
    ScopedTimer process_timer(process_time_pub);

    // 遅延監視用の遅延チェックポイント保存
    auto delay_checkpoints = msg.delay_checkpoints;
    DelayMonitorWrapper::addDelayCheckpoint(
      delay_checkpoints, "local_planner_start", "commands_received");

    // msg.robot_commands内のrobot_idダブリチェック（インデックスのみソートしてディープコピーを回避）
    {
      std::vector<std::pair<uint8_t, size_t>> id_pairs;
      id_pairs.reserve(msg.robot_commands.size());
      for (size_t i = 0; i < msg.robot_commands.size(); ++i) {
        id_pairs.emplace_back(msg.robot_commands[i].robot_id, i);
      }
      std::sort(id_pairs.begin(), id_pairs.end());
      for (size_t i = 1; i < id_pairs.size(); i++) {
        if (id_pairs[i - 1].first == id_pairs[i].first) {
          const auto & prev = msg.robot_commands[id_pairs[i - 1].second];
          const auto & curr = msg.robot_commands[id_pairs[i].second];
          RCLCPP_ERROR_STREAM(
            get_logger(), "ロボット " << static_cast<int>(curr.robot_id) << " が重複しています("
                                      << curr.planner_name << ", "
                                      << aggregateStates(curr.planning_factors) << " と "
                                      << prev.planner_name << ", "
                                      << aggregateStates(prev.planning_factors) << ")");
        }
      }
    }

    // 位置指令を検証して処理
    // 【座標系の設計】
    // - 位置・速度のベクトル成分(x,y)：フィールド座標系のまま（theta_offset未適用）
    // - 角度(theta)：theta_offsetを適用（half_court_practice_mode対応）
    // - この設計により、位置・速度は元の座標系を保持しつつ、角度だけ変換できる
    crane_msgs::msg::RobotCommands commands;
    commands.robot_commands.reserve(msg.robot_commands.size());
    for (const auto & raw_command : msg.robot_commands) {
      try {
        if (raw_command.robot_id >= world_model->ours().robots.size()) {
          dropped_command_count_last_cycle_++;
          dropped_command_count_total_++;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "robot_id %d is out of range [0, %zu), command is dropped",
            static_cast<int>(raw_command.robot_id), world_model->ours().robots.size());
          continue;
        }

        if (raw_command.position_target_mode.empty()) {
          dropped_command_count_last_cycle_++;
          dropped_command_count_total_++;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "robot_id=%d has no position_target_mode. command is dropped.",
            static_cast<int>(raw_command.robot_id));
          continue;
        }
        const auto & raw_pos_mode = raw_command.position_target_mode.front();
        const auto target_x = raw_pos_mode.target_x;
        const auto target_y = raw_pos_mode.target_y;

        // 位置目標の可視化
        planner->getVisualizer()->drawLine(
          Point(raw_command.current_pose.x, raw_command.current_pose.y), Point(target_x, target_y),
          "yellow", 20, 0.3);

        crane_msgs::msg::RobotCommand command = raw_command;
        auto & pos_mode = command.position_target_mode.front();
        pos_mode.target_x = target_x;
        pos_mode.target_y = target_y;
        pos_mode.position_tolerance = raw_pos_mode.position_tolerance;
        pos_mode.speed_limit_at_target = raw_pos_mode.speed_limit_at_target;
        auto robot = world_model->getOurRobot(command.robot_id);
        command.current_pose.x = robot->pose.pos.x();                   // フィールド座標系
        command.current_pose.y = robot->pose.pos.y();                   // フィールド座標系
        command.current_pose.theta = robot->pose.theta + theta_offset;  // theta_offset適用
        command.current_velocity.x = robot->vel.linear.x();             // フィールド座標系
        command.current_velocity.y = robot->vel.linear.y();             // フィールド座標系
        command.current_velocity.theta = robot->vel.omega;
        command.target_theta += theta_offset;  // theta_offset適用
        command.kick_power = kick_power_calculator.getKickPower(command);
        commands.robot_commands.push_back(command);
      } catch (const std::exception & e) {
        dropped_command_count_last_cycle_++;
        dropped_command_count_total_++;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Failed to process command for robot %d: %s",
          static_cast<int>(raw_command.robot_id), e.what());
      }
    }

    if (dropped_command_count_last_cycle_ > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Dropped %zu/%zu commands in this cycle",
        dropped_command_count_last_cycle_, msg.robot_commands.size());
    }

    crane_msgs::msg::RobotCommands pub_msg;
    try {
      pub_msg = planner->calculateRobotCommand(commands, theta_offset);
    } catch (const std::exception & e) {
      planner_exception_count_last_cycle_++;
      planner_exception_count_total_++;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Path planning failed. Publishing empty /robot_commands this cycle: %s", e.what());
    }

    pub_msg.header.stamp = now();
    pub_msg.is_yellow = world_model->isYellow();

    // 遅延監視: LocalPlanner処理完了、最終コマンド送信
    pub_msg.delay_checkpoints = delay_checkpoints;
    DelayMonitorWrapper::addDelayCheckpoint(
      pub_msg.delay_checkpoints, "local_planner_end", "path_planned");

    commands_pub.publish(pub_msg);

    planner->getVisualizer()->flush();
    crane::CraneVisualizerBuffer::publish();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Unhandled exception in local_planner callback. Publishing empty /robot_commands: %s",
      e.what());
    publishFallback();
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Unhandled unknown exception in local_planner callback. Publishing empty /robot_commands");
    publishFallback();
  }
  // 診断情報を更新
  diagnostic_helper_.forceUpdate();
}

auto LocalPlannerComponent::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  -> void
{
  // プランナーの状態をチェック
  if (!planner) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "プランナーが初期化されていません");
    return;
  }

  auto world_model = planner->getWorldModel();
  if (not world_model) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "ワールドモデルが利用できません");
    return;
  }

  if (not world_model->hasUpdated()) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "ワールドモデルがまだ更新されていません");
    return;
  }

  stat.add("dropped_command_count_last_cycle", std::to_string(dropped_command_count_last_cycle_));
  stat.add("dropped_command_count_total", std::to_string(dropped_command_count_total_));
  stat.add(
    "planner_exception_count_last_cycle", std::to_string(planner_exception_count_last_cycle_));
  stat.add("planner_exception_count_total", std::to_string(planner_exception_count_total_));

  if (planner_exception_count_last_cycle_ > 0) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "例外が発生しましたが、空のrobot_commandsで継続しています");
    return;
  }

  if (dropped_command_count_last_cycle_ > 0) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "一部コマンドを除外しつつ経路計画を継続しています");
    return;
  }

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "経路計画は正常に動作しています");
}

auto LocalPlannerComponent::aggregateStates(
  const std::vector<crane_msgs::msg::NamedString> & planning_factors) const -> std::string
{
  std::stringstream ss;
  ss << "[";
  for (const auto & planning_factor : planning_factors) {
    ss << planning_factor.name << ":" << planning_factor.value << ", ";
  }
  ss << "]";
  return ss.str();
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
