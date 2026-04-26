// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_sessions/ball_placement_skill_session.hpp>
#include <filesystem>
#include <robocup_ssl_msgs/msg/game_event_type.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace crane
{
auto BallPlacementSkillSession::calculatePositionCommand(
  const std::vector<RobotIdentifier> & robots)
  -> std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
{
  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (robots.empty()) {
    return {SessionBase::Status::RUNNING, {}};
  }
  if (not skill) {
    skill = std::make_shared<skills::SingleBallPlacement>(
      "ball_placement_skill_planner", robots.front().id, world_model);
    active_robot_id_ = robots.front().id;
    initial_ball_placement_failures_ = getCurrentBallPlacementFailures();
  }

  if (auto target = world_model->getBallPlacementTarget(); target.has_value()) {
    skill->setParameter("placement_x", target->x());
    skill->setParameter("placement_y", target->y());
  }
  auto status = static_cast<Status>(skill->run());

  return {status, {skill->getRobotCommand()}};
}

auto BallPlacementSkillSession::getRobotSuitabilityFunc() const
  -> std::function<double(const std::shared_ptr<RobotInfo> &)>
{
  ensureHistoryLoaded();

  auto wm = world_model;
  auto history = history_;

  // 重みは YAML の params: で上書き可能
  const double w_success = getSessionParameter<double>("success_weight", 1.0);
  const double w_failure = getSessionParameter<double>("failure_weight", 2.0);
  const double w_distance = getSessionParameter<double>("distance_weight", 0.01);
  const double w_rotation = getSessionParameter<double>("rotation_weight", 0.001);
  const int top_n = getSessionParameter<int>("top_n_candidates", DEFAULT_TOP_N);

  // 距離昇順で上位 N 台を事前計算する
  // （suitability_func は1ロボットずつ呼ばれるため、ランキング情報を lambda にキャプチャしておく）
  const auto goalie_id = wm->getOurGoalieId();
  const Point ball_pos = wm->ball().pos;

  std::vector<std::pair<std::uint8_t, double>> distances;
  for (const auto & r : wm->ours().robotsWhere().available().get()) {
    if (r->id == goalie_id) continue;
    distances.emplace_back(r->id, (r->pose.pos - ball_pos).norm());
  }
  std::ranges::sort(
    distances, [](const auto & a, const auto & b) { return a.second < b.second; });

  // 上位 N 台のID集合と、その中での last_seq 順のランク（0=最古=最優先）
  // 注: last_seq の絶対値を使うと長期的に W_R * last_seq が W_F を超えてしまい
  // 失敗履歴のあるロボットに流れてしまう。順位なら最大でも (top_n - 1) で上限が効く。
  std::unordered_set<std::uint8_t> top_ids;
  std::vector<std::pair<std::uint8_t, std::uint64_t>> top_with_seq;
  for (int i = 0; i < std::min<int>(top_n, distances.size()); ++i) {
    const auto id = distances[i].first;
    top_ids.insert(id);
    top_with_seq.emplace_back(id, history->get(id).last_seq);
  }
  std::ranges::stable_sort(
    top_with_seq, [](const auto & a, const auto & b) { return a.second < b.second; });
  std::unordered_map<std::uint8_t, int> rotation_rank;
  for (std::size_t i = 0; i < top_with_seq.size(); ++i) {
    rotation_rank[top_with_seq[i].first] = static_cast<int>(i);
  }

  return [history, top_ids, rotation_rank, ball_pos, goalie_id, w_success, w_failure, w_distance,
          w_rotation](const std::shared_ptr<RobotInfo> & robot) {
    if (robot->id == goalie_id) {
      return GOALIE_EXCLUSION_COST;
    }

    const double distance = (robot->pose.pos - ball_pos).norm();

    // 上位 N 台外は実用上選ばれない高コスト。distance を加算して安定なランキングを与える。
    if (top_ids.find(robot->id) == top_ids.end()) {
      return NON_CANDIDATE_COST + distance;
    }

    const auto entry = history->get(robot->id);
    const double raw =
      w_failure * static_cast<double>(entry.failure) -
      w_success * static_cast<double>(entry.success);
    const double clamped = std::max(0.0, raw);

    if (clamped > 0.0) {
      // 失敗が支配的: 失敗少 → 距離 の順
      return clamped + w_distance * distance;
    }
    // 全員「優秀」ゾーン: 上位N内で last_seq が小さい(=最古) ロボットを優先 → ローテ
    // rank は 0..(top_n-1) なので W_R * rank の上限は (top_n-1) * W_R で、W_F より小さく保てる
    const auto it = rotation_rank.find(robot->id);
    const int rank = (it != rotation_rank.end()) ? it->second : 0;
    return w_rotation * static_cast<double>(rank) + w_distance * distance;
  };
}

void BallPlacementSkillSession::onRobotsChanged()
{
  skill.reset();
  active_robot_id_.reset();
  initial_ball_placement_failures_.reset();
}

void BallPlacementSkillSession::onDeactivated(
  const crane_msgs::msg::PlaySituation & current_play_situation)
{
  if (!active_robot_id_.has_value()) {
    return;
  }

  const auto robot_id = *active_robot_id_;
  active_robot_id_.reset();
  skill.reset();

  const auto explicit_result = extractPlacementResult(current_play_situation);
  if (explicit_result.has_value()) {
    recordPlacementResult(robot_id, *explicit_result);
    initial_ball_placement_failures_.reset();
    return;
  }

  const auto start_failures = initial_ball_placement_failures_.value_or(0);
  initial_ball_placement_failures_.reset();
  if (current_play_situation.our_team_info.ball_placement_failures > start_failures) {
    recordPlacementResult(robot_id, false);
    return;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BallPlacementSkillSession"),
    "ロボット %d のボールプレイスメント終了を検知したが、明示的な成否シグナルがないため履歴更新をスキップ",
    static_cast<int>(robot_id));
}

void BallPlacementSkillSession::ensureHistoryLoaded() const
{
  if (history_) {
    return;
  }
  const auto path = resolveHistoryFilePath();
  history_ = std::make_shared<BallPlacementHistory>(path);
}

std::string BallPlacementSkillSession::resolveHistoryFilePath() const
{
  const auto override_path = getSessionParameter<std::string>("history_file_path", "");
  if (!override_path.empty()) {
    return override_path;
  }

  // unified_session_config.yaml と同様に package_share_directory/config から解決する。
  // その上で symlink-install 環境では実体パスへ解決し、src 側のファイルを直接更新する。
  const auto config_path = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("crane_session_coordinator")) /
                           "config" / "ball_placement_history.yaml";
  try {
    return std::filesystem::weakly_canonical(config_path).string();
  } catch (const std::exception &) {
    return config_path.string();
  }
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
    if (
      game_event.type.value ==
      robocup_ssl_msgs::msg::GameEventType::PLACEMENT_SUCCEEDED) {
      result = true;
    } else if (
      game_event.type.value ==
      robocup_ssl_msgs::msg::GameEventType::PLACEMENT_FAILED) {
      result = false;
    }
  }
  return result;
}

void BallPlacementSkillSession::recordPlacementResult(
  std::uint8_t robot_id, bool success) const
{
  ensureHistoryLoaded();
  const bool saved = success ? history_->recordSuccess(robot_id) : history_->recordFailure(robot_id);
  const char * result_text = success ? "成功" : "失敗";
  if (saved) {
    RCLCPP_INFO(
      rclcpp::get_logger("BallPlacementSkillSession"),
      "ロボット %d のプレイスメント%sを記録", static_cast<int>(robot_id), result_text);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("BallPlacementSkillSession"),
      "ロボット %d のプレイスメント%s履歴の保存に失敗", static_cast<int>(robot_id),
      result_text);
  }
}

}  // namespace crane
