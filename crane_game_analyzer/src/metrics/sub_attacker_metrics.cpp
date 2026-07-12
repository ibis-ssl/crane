// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/sub_attacker_metrics.hpp"

#include <crane_geometry/ddps.hpp>
#include <crane_msg_wrappers/sub_attacker_evaluation.hpp>

namespace crane::metrics
{

SubAttackerPositionMetric::SubAttackerPositionMetric()
: MetricBase(MetricId::SUB_ATTACKER_POSITION, "SubAttackerPosition")
{
}

auto SubAttackerPositionMetric::compute(MetricContext & ctx) -> void
{
  auto & wm = ctx.world_model;
  auto & analysis = ctx.analysis;

  // 計算頻度スロットル: 前回計算からMIN_COMPUTE_INTERVAL_SEC未満なら前回結果を再送出する
  // NOTE: DPPS探索(候補点2560個)自体の所要時間が約130msあるため、
  // 間隔をそれより十分大きく取らないと実質的に間引けない（130ms未満の閾値では常に
  // elapsed >= 閾値になってしまう）。ヒステリシス閾値(0.5m)を踏まえ0.3秒(約3.3Hz)とする。
  constexpr double MIN_COMPUTE_INTERVAL_SEC = 0.3;
  auto now = ctx.clock->now();
  if (has_computed_ && (now - last_compute_time_).seconds() < MIN_COMPUTE_INTERVAL_SEC) {
    analysis.has_sub_attacker_position = last_has_position_;
    analysis.sub_attacker_position_score = last_score_;
    if (last_has_position_) {
      analysis.recommended_sub_attacker_position.x = last_position_.x();
      analysis.recommended_sub_attacker_position.y = last_position_.y();
    }
    return;
  }
  last_compute_time_ = now;
  has_computed_ = true;

  // デフォルトでは無効
  analysis.has_sub_attacker_position = false;
  analysis.sub_attacker_position_score = 0.0f;

  // ボールが自陣にある場合はSubAttacker不要
  if (wm->point_checker.isInOurHalf(wm->ball().pos)) {
    last_has_position_ = false;
    last_score_ = 0.0f;
    return;
  }

  // DPPS候補点を生成（旧SubAttackerSkillPlannerと同じパラメータ）
  // 半径0.25m刻み、最大10m、64方向
  auto candidates = getDPPSPoints(wm->ball().pos, 0.25, 10.0, 64);

  // フィルタリング：フィールド内かつペナルティエリア外
  std::vector<Point> valid_candidates;
  for (const auto & candidate : candidates) {
    if (wm->point_checker.isFieldInside(candidate) && !wm->point_checker.isPenaltyArea(candidate)) {
      valid_candidates.push_back(candidate);
    }
  }

  if (valid_candidates.empty()) {
    last_has_position_ = false;
    last_score_ = 0.0f;
    return;
  }

  // 各候補点のスコアを計算
  double best_score = -std::numeric_limits<double>::infinity();
  Point best_position{0.0, 0.0};

  for (const auto & candidate : valid_candidates) {
    double score = crane::evaluateSubAttackerPosition(candidate, *wm);

    if (score > best_score) {
      best_score = score;
      best_position = candidate;
    }
  }

  // ヒステリシス：前回位置から大きく変わる場合のみ更新
  constexpr double HYSTERESIS_THRESHOLD = 0.5;  // 0.5m以上動いたら更新
  if (has_last_position_) {
    double distance =
      std::hypot(best_position.x() - last_position_.x(), best_position.y() - last_position_.y());
    if (distance < HYSTERESIS_THRESHOLD) {
      // 前回位置を維持
      best_position = last_position_;
    }
  }

  // 結果を設定
  analysis.has_sub_attacker_position = true;
  analysis.recommended_sub_attacker_position.x = best_position.x();
  analysis.recommended_sub_attacker_position.y = best_position.y();
  analysis.sub_attacker_position_score = static_cast<float>(best_score);

  // 状態を更新
  last_position_ = best_position;
  has_last_position_ = true;
  last_has_position_ = true;
  last_score_ = static_cast<float>(best_score);
}

}  // namespace crane::metrics
