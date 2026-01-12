// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTIC_COORDINATOR__COMMAND_AGGREGATOR_HPP_
#define CRANE_TACTIC_COORDINATOR__COMMAND_AGGREGATOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/position_commands.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tactic_registry.hpp"

namespace crane
{

/**
 * @brief 全プランナーからPositionCommandsを収集・構築するクラス
 */
class CommandAggregator
{
public:
  explicit CommandAggregator(std::shared_ptr<TacticRegistry> tactic_registry);

  /**
   * @brief 全プランナーからコマンドを収集してメッセージを構築
   * @param world_model WorldModelの参照
   * @param current_time 現在時刻
   * @param game_analysis GameAnalysisメッセージ
   * @return 構築されたPositionCommandsメッセージ
   */
  auto collectCommands(
    const WorldModelWrapper::SharedPtr & world_model, const rclcpp::Time & current_time,
    const crane_msgs::msg::GameAnalysis & game_analysis) -> crane_msgs::msg::PositionCommands;

private:
  /**
   * @brief メッセージのメタデータを設定（ヘッダー、遅延チェックポイント等）
   */
  auto setMessageMetadata(
    crane_msgs::msg::PositionCommands & msg, const WorldModelWrapper::SharedPtr & world_model,
    const rclcpp::Time & current_time) -> void;

  /**
   * @brief ロボット優先度を設定
   */
  auto assignPriorities(crane_msgs::msg::PositionCommands & msg) -> void;

  std::shared_ptr<TacticRegistry> tactic_registry_;
};

}  // namespace crane

#endif  // CRANE_TACTIC_COORDINATOR__COMMAND_AGGREGATOR_HPP_
