// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__FREE_KICKER_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__FREE_KICKER_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/free_kicker.hpp>
#include <crane_sessions/rotating_single_skill_session.hpp>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class FreeKickerSkillSession : public RotatingSingleSkillSession<skills::FreeKicker>
{
public:
  COMPOSITION_PUBLIC explicit FreeKickerSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : RotatingSingleSkillSession("free_kicker_skill", world_model)
  {
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::FreeKicker> override;

  std::filesystem::path resolveHistoryFilePath() const override;

  /// FreeKick の成否は以下の優先順で判定する:
  /// 1. 試行中に NO_PROGRESS_IN_GAME / 自チームの BOT_DRIBBLED_BALL_TOO_FAR /
  ///    自チームの ATTACKER_DOUBLE_TOUCHED_BALL が新規発生 → 失敗
  /// 2. スキルが FINISH に到達 → skill->kickActuallyLaunched() の真偽そのまま
  ///    (タイムアウトで FINISH した空振りは false=失敗)
  /// 3. それ以外（FINISH 未到達で割当解除） → 失敗扱い
  std::optional<bool> determineAssignmentResult(
    const crane_msgs::msg::PlaySituation & current_play_situation) override;

  void onSkillStarted() override;
  void onRobotsChangedHook() override;

private:
  /// 試行開始時点での referee_raw.game_events のサイズ。
  /// 「試行中に発生したイベント」だけを判定対象にするためのスナップショット。
  std::optional<std::size_t> initial_game_events_size_;

  /// 試行開始以降に追加された game events のうち、FreeKick 失敗を示すものがあるか。
  bool hasOurFailureGameEvent(
    const crane_msgs::msg::PlaySituation & current_play_situation,
    std::size_t start_index) const;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__FREE_KICKER_SKILL_SESSION_HPP_
