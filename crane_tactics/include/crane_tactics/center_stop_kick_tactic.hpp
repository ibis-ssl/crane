// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__CENTER_STOP_KICK_TACTIC_HPP_
#define CRANE_TACTICS__CENTER_STOP_KICK_TACTIC_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/center_stop_kick.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

#include "visibility_control.h"

namespace crane
{

/**
 * @brief フィールド中心停止キック用プランナー
 *
 * CenterStopKickスキルを実行するためのシンプルなプランナー。
 * ボールをフィールド中心(0,0)で正確に停止させるストレートキックを行う。
 * 実際のロジックはスキル側で実装されている。
 */
class CenterStopKickTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC explicit CenterStopKickTactic(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &);

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

private:
  // スキルインスタンス
  std::shared_ptr<skills::CenterStopKick> skill_;

  // 最後に選択されたロボットID
  std::optional<uint8_t> current_robot_id_;
};

}  // namespace crane

#endif  // CRANE_TACTICS__CENTER_STOP_KICK_TACTIC_HPP_
