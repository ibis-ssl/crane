// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_sessions/ball_placement_history.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class BallPlacementSkillSession : public SessionBase
{
public:
  std::shared_ptr<skills::SingleBallPlacement> skill = nullptr;

  COMPOSITION_PUBLIC explicit BallPlacementSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("ball_placement_skill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override;

protected:
  void onRobotsChanged() override;
  void onDeactivated(const crane_msgs::msg::PlaySituation & current_play_situation) override;

private:
  /// 履歴ファイル（プロセス再起動を跨ぐ）。初回 suitability/calculate 呼び出しで遅延初期化する。
  /// 遅延にしている理由: パスは YAML パラメータ経由で渡されるため、コンストラクタ時点では
  /// まだ session_params_ が空の可能性がある。
  mutable std::shared_ptr<BallPlacementHistory> history_ = nullptr;

  /// 現在のボールプレイスメント試行に割り当てられているロボットID。
  std::optional<std::uint8_t> active_robot_id_;

  /// 試行開始時点の味方チームの ball_placement_failures 値。
  std::optional<std::uint32_t> initial_ball_placement_failures_;

  /// 履歴を遅延初期化する。
  void ensureHistoryLoaded() const;

  /// パラメータから履歴ファイルパスを解決する。
  std::string resolveHistoryFilePath() const;

  /// 現在の味方チームの ball_placement_failures 値を取得する。
  std::uint32_t getCurrentBallPlacementFailures() const;

  /// PlaySituation / Referee から明示的な成否を抽出する。
  std::optional<bool> extractPlacementResult(
    const crane_msgs::msg::PlaySituation & current_play_situation) const;

  /// 成否を履歴へ反映する。
  void recordPlacementResult(std::uint8_t robot_id, bool success) const;

  // 候補絞り込み: 距離昇順で上位 N 台に限定（デフォルト 3）
  static constexpr int DEFAULT_TOP_N = 3;

  /// 候補外（ゴーリーや上位N台外）に乗せる大きなコスト。distance を加算して数値を分散させる。
  static constexpr double NON_CANDIDATE_COST = 1e6;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_
