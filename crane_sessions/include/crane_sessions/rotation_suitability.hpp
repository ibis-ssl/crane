// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__ROTATION_SUITABILITY_HPP_
#define CRANE_SESSIONS__ROTATION_SUITABILITY_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/skill_assignment_history.hpp>
#include <functional>
#include <memory>

namespace crane
{
/// ローテーション付き suitability 関数の重み・パラメータ。
///
/// スコア = clamped > 0 ? clamped + W_D * distance
///        : W_R * rotation_rank + W_D * distance     (clamped == 0 のローテ帯)
///   ただし clamped = max(0, W_F * failures - W_S * successes)
///
/// distance はボールからの直線距離（m）。
/// rotation_rank は上位 top_n_candidates 台内で last_seq 昇順の順位 (0=最古=最優先)。
struct RotationSuitabilityConfig
{
  double success_weight = 1.0;
  double failure_weight = 2.0;
  double distance_weight = 0.01;
  double rotation_weight = 0.001;
  int top_n_candidates = 3;
};

/// ローテーション付き suitability 関数を生成する。
///
/// - 距離昇順で上位 top_n_candidates 台のみ候補とし、それ以外は実用上選ばれない高コスト
///   (NON_CANDIDATE_COST + distance) を返す。
/// - 候補内で失敗履歴が支配的なら「失敗少 → 距離」順、そうでなければ
///   「last_seq 古い → 距離」順で選ぶことでローテーションする。
/// - ゴーリーは GOALIE_EXCLUSION_COST で除外。
///
/// `history` は遅延初期化された後の有効な shared_ptr を渡すこと。
std::function<double(const std::shared_ptr<RobotInfo> &)> makeRotationSuitabilityFunc(
  const WorldModelWrapper::SharedPtr & world_model,
  std::shared_ptr<SkillAssignmentHistory> history, const RotationSuitabilityConfig & config);

}  // namespace crane
#endif  // CRANE_SESSIONS__ROTATION_SUITABILITY_HPP_
