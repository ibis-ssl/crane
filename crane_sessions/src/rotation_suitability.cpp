// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <crane_sessions/rotation_suitability.hpp>
#include <crane_sessions/session_base.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace crane
{
namespace
{
/// 候補外（ゴーリーや上位N台外）に乗せる大きなコスト。distance を加算して数値を分散させる。
constexpr double NON_CANDIDATE_COST = 1e6;
}  // namespace

std::function<double(const std::shared_ptr<RobotInfo> &)> makeRotationSuitabilityFunc(
  const WorldModelWrapper::SharedPtr & world_model,
  std::shared_ptr<SkillAssignmentHistory> history, const RotationSuitabilityConfig & config)
{
  // 距離昇順で上位 N 台を事前計算する
  // （suitability_func は1ロボットずつ呼ばれるため、ランキング情報を lambda にキャプチャしておく）
  const auto goalie_id = world_model->getOurGoalieId();
  const Point ball_pos = world_model->ball().pos;

  std::vector<std::pair<std::uint8_t, double>> distances;
  for (const auto & r : world_model->ours().robotsWhere().available().get()) {
    if (r->id == goalie_id) continue;
    distances.emplace_back(r->id, (r->pose.pos - ball_pos).norm());
  }
  std::ranges::sort(distances, [](const auto & a, const auto & b) { return a.second < b.second; });

  // 上位 N 台のID集合と、その中での last_seq 順のランク（0=最古=最優先）
  // 注: last_seq の絶対値を使うと長期的に W_R * last_seq が W_F を超えてしまい
  // 失敗履歴のあるロボットに流れてしまう。順位なら最大でも (top_n - 1) で上限が効く。
  std::unordered_set<std::uint8_t> top_ids;
  std::vector<std::pair<std::uint8_t, std::uint64_t>> top_with_seq;
  for (int i = 0; i < std::min<int>(config.top_n_candidates, distances.size()); ++i) {
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

  return [history, top_ids = std::move(top_ids), rotation_rank = std::move(rotation_rank), ball_pos,
          goalie_id, config](const std::shared_ptr<RobotInfo> & robot) {
    if (robot->id == goalie_id) {
      return SessionBase::GOALIE_EXCLUSION_COST;
    }

    const double distance = (robot->pose.pos - ball_pos).norm();

    // 上位 N 台外は実用上選ばれない高コスト。distance を加算して安定なランキングを与える。
    if (top_ids.find(robot->id) == top_ids.end()) {
      return NON_CANDIDATE_COST + distance;
    }

    const auto entry = history->get(robot->id);
    const double raw = config.failure_weight * static_cast<double>(entry.failure) -
                       config.success_weight * static_cast<double>(entry.success);
    const double clamped = std::max(0.0, raw);

    if (clamped > 0.0) {
      // 失敗が支配的: 失敗少 → 距離 の順
      return clamped + config.distance_weight * distance;
    }
    // 全員「優秀」ゾーン: 上位N内で last_seq が小さい(=最古) ロボットを優先 → ローテ
    // rank は 0..(top_n-1) なので W_R * rank の上限は (top_n-1) * W_R で、W_F より小さく保てる
    const auto it = rotation_rank.find(robot->id);
    const int rank = (it != rotation_rank.end()) ? it->second : 0;
    return config.rotation_weight * static_cast<double>(rank) + config.distance_weight * distance;
  };
}

}  // namespace crane
