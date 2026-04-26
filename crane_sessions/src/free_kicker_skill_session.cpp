// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_sessions/free_kicker_skill_session.hpp>
#include <robocup_ssl_msgs/msg/game_event_type.hpp>
#include <robocup_ssl_msgs/msg/team.hpp>

namespace crane
{
auto FreeKickerSkillSession::createSkill(uint8_t robot_id) -> std::shared_ptr<skills::FreeKicker>
{
  return std::make_shared<skills::FreeKicker>(robot_id, world_model);
}

std::filesystem::path FreeKickerSkillSession::resolveHistoryFilePath() const
{
  const auto override_path = getSessionParameter<std::string>("history_file_path", "");
  if (!override_path.empty()) {
    return override_path;
  }

  const auto config_path =
    std::filesystem::path(
      ament_index_cpp::get_package_share_directory("crane_session_coordinator")) /
    "config" / "free_kicker_history.yaml";
  try {
    return std::filesystem::weakly_canonical(config_path);
  } catch (const std::exception &) {
    return config_path;
  }
}

std::optional<bool> FreeKickerSkillSession::determineAssignmentResult(
  const crane_msgs::msg::PlaySituation & current_play_situation)
{
  const auto start_index = initial_game_events_size_.value_or(0);
  initial_game_events_size_.reset();

  if (hasOurFailureGameEvent(current_play_situation, start_index)) {
    return false;
  }

  if (skill && skill->getCurrentState() == skills::FreeKickerState::FINISH) {
    return skill->kickActuallyLaunched();
  }

  return false;
}

void FreeKickerSkillSession::onSkillStarted()
{
  initial_game_events_size_ =
    world_model->getMsg().play_situation.referee_raw.game_events.size();
}

void FreeKickerSkillSession::onRobotsChangedHook() { initial_game_events_size_.reset(); }

bool FreeKickerSkillSession::hasOurFailureGameEvent(
  const crane_msgs::msg::PlaySituation & current_play_situation, std::size_t start_index) const
{
  const auto & events = current_play_situation.referee_raw.game_events;
  if (start_index > events.size()) {
    // 何らかの理由でリストが縮んだ場合は判定不能 → 失敗扱いしない
    return false;
  }

  const int our_team_value = world_model->isYellow() ? robocup_ssl_msgs::msg::Team::YELLOW
                                                     : robocup_ssl_msgs::msg::Team::BLUE;

  using GET = robocup_ssl_msgs::msg::GameEventType;
  for (std::size_t i = start_index; i < events.size(); ++i) {
    const auto & ge = events[i];
    switch (ge.type.value) {
      case GET::NO_PROGRESS_IN_GAME:
        // ゲーム全体が膠着した: フリーキックで打開できなかった → 失敗
        return true;
      case GET::BOT_DRIBBLED_BALL_TOO_FAR:
        if (ge.event.bot_dribbled_ball_too_far.by_team.value == our_team_value) return true;
        break;
      case GET::ATTACKER_DOUBLE_TOUCHED_BALL:
        if (ge.event.attacker_double_touched_ball.by_team.value == our_team_value) return true;
        break;
      default:
        break;
    }
  }
  return false;
}

}  // namespace crane
