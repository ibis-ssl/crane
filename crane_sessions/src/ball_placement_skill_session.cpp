// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_sessions/ball_placement_skill_session.hpp>
#include <robocup_ssl_msgs/msg/game_event_type.hpp>

namespace crane
{
auto BallPlacementSkillSession::createSkill(uint8_t robot_id)
  -> std::shared_ptr<skills::SingleBallPlacement>
{
  return std::make_shared<skills::SingleBallPlacement>(
    "ball_placement_skill_planner", robot_id, world_model);
}

std::filesystem::path BallPlacementSkillSession::resolveHistoryFilePath() const
{
  const auto override_path = getSessionParameter<std::string>("history_file_path", "");
  if (!override_path.empty()) {
    return override_path;
  }

  // unified_session_config.yaml と同様に package_share_directory/config から解決する。
  // その上で symlink-install 環境では実体パスへ解決し、src 側のファイルを直接更新する。
  const auto config_path =
    std::filesystem::path(
      ament_index_cpp::get_package_share_directory("crane_session_coordinator")) /
    "config" / "ball_placement_history.yaml";
  try {
    return std::filesystem::weakly_canonical(config_path);
  } catch (const std::exception &) {
    return config_path;
  }
}

std::optional<bool> BallPlacementSkillSession::determineAssignmentResult(
  const crane_msgs::msg::PlaySituation & current_play_situation)
{
  const auto explicit_result = extractPlacementResult(current_play_situation);
  if (explicit_result.has_value()) {
    initial_ball_placement_failures_.reset();
    return explicit_result;
  }

  const auto start_failures = initial_ball_placement_failures_.value_or(0);
  initial_ball_placement_failures_.reset();
  if (current_play_situation.our_team_info.ball_placement_failures > start_failures) {
    return false;
  }
  return std::nullopt;
}

void BallPlacementSkillSession::onSkillStarted()
{
  initial_ball_placement_failures_ = getCurrentBallPlacementFailures();
}

void BallPlacementSkillSession::onBeforeSkillRun()
{
  if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
    skill->setParameter("placement_x", target->x());
    skill->setParameter("placement_y", target->y());
  }
}

void BallPlacementSkillSession::onRobotsChangedHook()
{
  initial_ball_placement_failures_.reset();
}

std::uint32_t BallPlacementSkillSession::getCurrentBallPlacementFailures() const
{
  return world_model->getMsg().play_situation.our_team_info.ball_placement_failures;
}

std::optional<bool> BallPlacementSkillSession::extractPlacementResult(
  const crane_msgs::msg::PlaySituation & current_play_situation) const
{
  std::optional<bool> result;
  for (const auto & game_event : current_play_situation.referee_raw.game_events) {
    if (game_event.type.value == robocup_ssl_msgs::msg::GameEventType::PLACEMENT_SUCCEEDED) {
      result = true;
    } else if (game_event.type.value == robocup_ssl_msgs::msg::GameEventType::PLACEMENT_FAILED) {
      result = false;
    }
  }
  return result;
}

}  // namespace crane
