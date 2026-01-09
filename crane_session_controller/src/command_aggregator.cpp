// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_controller/command_aggregator.hpp"

#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
#include <range/v3/algorithm/for_each.hpp>

namespace crane
{

CommandAggregator::CommandAggregator(std::shared_ptr<TacticRegistry> tactic_registry)
: tactic_registry_(tactic_registry)
{
}

auto CommandAggregator::collectCommands(
  const WorldModelWrapper::SharedPtr & world_model, const rclcpp::Time & current_time)
  -> crane_msgs::msg::PositionCommands
{
  crane_msgs::msg::PositionCommands msg;

  // メタデータ設定
  setMessageMetadata(msg, world_model, current_time);

  // 全プランナーからコマンドを収集
  for (const auto & tactic : tactic_registry_->getAllPlanners()) {
    auto robot_count = tactic->getRobots().size();
    auto commands_msg = tactic->getPositionCommands();
    auto command_count = commands_msg.robot_commands.size();

    if (robot_count > 0 && command_count == 0) {
      // ロボットは割り当てられているがコマンドが生成されていない
      std::cerr << "[CommandAggregator] 警告: Tactic「" << tactic->name << "」にロボット "
                << robot_count << " 台が割り当てられていますが、"
                << "コマンドが0件生成されました" << std::endl;
    }

    // Note: tactic_name フィールドはメッセージ定義にないためコメントアウト
    // ranges::for_each(
    //   commands_msg.robot_commands, [&](crane_msgs::msg::PositionCommand & position_command) {
    //     position_command.tactic_name = tactic->name;
    //   });
    msg.robot_commands.insert(
      msg.robot_commands.end(), commands_msg.robot_commands.begin(),
      commands_msg.robot_commands.end());

    if (tactic->getStatus() != TacticBase::Status::RUNNING) {
      // TODO(HansRobo): プランナが成功・失敗した場合の処理
    }
  }

  // ロボット優先度を設定
  assignPriorities(msg);

  return msg;
}

auto CommandAggregator::setMessageMetadata(
  crane_msgs::msg::PositionCommands & msg, const WorldModelWrapper::SharedPtr & world_model,
  const rclcpp::Time & current_time) -> void
{
  msg.header = world_model->getMsg().header;
  msg.on_positive_half = world_model->onPositiveHalf();
  msg.is_yellow = world_model->isYellow();

  // WorldModelのdelay_checkpointsをPositionCommandsにコピー
  msg.delay_checkpoints = world_model->getDelayCheckpoints();

  msg.header.stamp = current_time;

  // 遅延監視: TacticCoordinator処理完了、PositionCommands送信
  DelayMonitorWrapper::addDelayCheckpoint(
    msg.delay_checkpoints, "session_controller_end", "strategy_computed");
}

auto CommandAggregator::assignPriorities(crane_msgs::msg::PositionCommands & msg) -> void
{
  // ロボットの優先度を設定(値が高いほど優先度が高い)
  uint8_t robot_priority = 100;
  for (auto & position_command : msg.robot_commands) {
    position_command.local_planner_config.priority = --robot_priority;
  }
}

}  // namespace crane
