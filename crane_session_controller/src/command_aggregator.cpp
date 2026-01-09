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

CommandAggregator::CommandAggregator(std::shared_ptr<PlannerRegistry> planner_registry)
: planner_registry_(planner_registry)
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
  for (const auto & planner : planner_registry_->getAllPlanners()) {
    auto commands_msg = planner->getPositionCommands();
    ranges::for_each(
      commands_msg.robot_commands, [&](crane_msgs::msg::PositionCommand & position_command) {
        position_command.planner_name = planner->name;
      });
    msg.robot_commands.insert(
      msg.robot_commands.end(), commands_msg.robot_commands.begin(),
      commands_msg.robot_commands.end());

    if (planner->getStatus() != PlannerBase::Status::RUNNING) {
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

  // 遅延監視: SessionController処理完了、PositionCommands送信
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
