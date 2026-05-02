// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_sessions/rotating_single_skill_session.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class BallPlacementSkillSession : public RotatingSingleSkillSession<skills::SingleBallPlacement>
{
public:
  COMPOSITION_PUBLIC explicit BallPlacementSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : RotatingSingleSkillSession("ball_placement_skill", world_model)
  {
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::SingleBallPlacement> override;

  std::filesystem::path resolveHistoryFilePath() const override;

  std::optional<bool> determineAssignmentResult(
    const crane_msgs::msg::PlaySituation & current_play_situation) override;

  void onSkillStarted() override;

  void onBeforeSkillRun() override;

  void onRobotsChangedHook() override;

private:
  /// 試行開始時点の味方チームの ball_placement_failures 値。
  /// determineAssignmentResult からの参照後にリセットするため mutable 不要。
  std::optional<std::uint32_t> initial_ball_placement_failures_;

  std::uint32_t getCurrentBallPlacementFailures() const;

  /// PlaySituation / Referee から明示的な成否を抽出する。
  std::optional<bool> extractPlacementResult(
    const crane_msgs::msg::PlaySituation & current_play_situation) const;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_
