// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__FORWARD_SESSION_HPP_
#define CRANE_SESSIONS__FORWARD_SESSION_HPP_

#include <algorithm>
#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/penalty_aware_distance.hpp>
#include <crane_robot_skills/forward.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class ForwardSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit ForwardSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("forward", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      return estimatePenaltyAwareDistance(
        robot->pose.pos, wm->getTheirGoalCenter(), wm->getOurPenaltyArea(), wm->getOurGoalCenter(),
        wm->getTheirPenaltyArea(), wm->getTheirGoalCenter(), wm->penaltyAreaSize());
    };
  }

  int getDesiredRobotNumber(int max_robots) const override { return max_robots; }

protected:
  void onRobotsChanged() override
  {
    forward_skills.clear();
    // ロボット構成が変わった場合はヒステリシス状態もリセット
    last_targets_.clear();
    last_scores_.clear();
  }

private:
  /// @brief 攻撃エリアの候補点グリッドを生成する（PA内・フィールド外を除外済み）
  auto computeCandidatePoints() const -> std::vector<Point>;

  /// @brief 候補点のスコアを計算する（乗算合成）
  auto scorePoint(
    const Point & p, const Point & robot_pos, const std::vector<Point> & forward_positions,
    const RobotList & available_enemies) const -> double;

  std::vector<std::shared_ptr<skills::Forward>> forward_skills;

  /// ロボットIDごとの前回選択目標点（ヒステリシス用）
  std::unordered_map<uint8_t, Point> last_targets_;

  /// ロボットIDごとの前回目標点の生スコア（ヒステリシス判定の再計算を省く）
  std::unordered_map<uint8_t, double> last_scores_;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__FORWARD_SESSION_HPP_
