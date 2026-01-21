// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/sub_attacker_metrics.hpp"

#include <crane_geometry/ddps.hpp>
#include <crane_geometry/geometry_operations.hpp>

namespace crane::metrics
{

SubAttackerPositionMetric::SubAttackerPositionMetric()
: MetricBase(MetricId::SUB_ATTACKER_POSITION, "SubAttackerPosition")
{
}

namespace
{
// SubAttacker::getPointScoreのロジックを再実装
// TODO(Hans): 将来的にはcrane_geometryなど共通パッケージに移動すべき
double evaluateSubAttackerPosition(const Point & p, const WorldModelWrapper * world_model)
{
  Segment line{world_model->ball().pos, p};

  // 敵ロボットとの最短距離を計算
  double min_enemy_dist = std::numeric_limits<double>::max();
  Point closest_enemy_point = world_model->ball().pos;

  for (const auto & robot : world_model->theirs().getAvailableRobots()) {
    auto result = getClosestPointAndDistance(robot->pose.pos, line);
    if (result.distance < min_enemy_dist) {
      min_enemy_dist = result.distance;
      closest_enemy_point = result.closest_point;
    }
  }

  auto [angle, width] = world_model->getLargestGoalAngleRangeFromPoint(p);
  // ゴールが見える角度が大きいほどよい
  double score = width;

  // 敵が動いてボールをブロック出来るかどうか
  double enemy_closest_to_ball_dist = (closest_enemy_point - world_model->ball().pos).norm();
  double ratio = min_enemy_dist / enemy_closest_to_ball_dist;
  // ratioが大きいほどよい / 0.1以下は厳しい
  if (ratio < 0.1) {
    score = 0.;
  } else {
    score *= std::clamp(ratio * 5, 0.0, 1.0);
  }

  if (
    std::abs(world_model->ball().pos.x() - world_model->goal().x()) >
    std::abs(world_model->goal().x())) {
    // 反射角　小さいほどよい（敵ゴールに近い場合のみ）
    auto reflect_angle = std::abs(getAngleDiff(angle, getAngle(world_model->ball().pos - p)));
    score *= (1.0 - std::min(reflect_angle * 0.5, 1.0));
  }

  // 距離評価: 最小距離制約と適度な距離を評価
  const double dist = (world_model->ball().pos - p).norm();
  // 最小距離制約: 1.5m未満は大幅減点
  constexpr double MIN_PASS_DISTANCE = 1.5;
  if (dist < MIN_PASS_DISTANCE) {
    score *= dist / MIN_PASS_DISTANCE;  // 1.0mなら0.67倍、0.5mなら0.33倍
  }
  // 距離が遠すぎても減点（10m以上はスコア0）
  score = score * std::max(1.0 - dist / 10.0, 0.0);

  // シュートラインに近すぎる場所は避ける
  Segment shoot_line{world_model->getOurGoalCenter(), world_model->ball().pos};
  const auto dist_to_shoot_line = bg::distance(p, shoot_line);
  if (dist_to_shoot_line < 0.5) {
    score = 0.0;
  }

  // 自分とボールの延長線上にゴールがある場合は避ける
  Segment robot_to_ball_and_more{
    p, world_model->ball().pos + (world_model->ball().pos - p).normalized() * 10.0};
  Segment goal_line{
    world_model->getTheirGoalPosts().first, world_model->getTheirGoalPosts().second};
  if (double distance = bg::distance(robot_to_ball_and_more, goal_line); distance < 1.0) {
    score *= distance;
  }

  // ボールの後側にには行かない
  double dot = (world_model->getTheirGoalCenter() - world_model->ball().pos)
                 .normalized()
                 .dot((p - world_model->ball().pos).normalized());
  score *= std::max((dot + 0.5), 0.0);

  return score;
}
}  // namespace

auto SubAttackerPositionMetric::compute(MetricContext & ctx) -> void
{
  auto & wm = ctx.world_model;
  auto & analysis = ctx.analysis;

  // デフォルトでは無効
  analysis.has_sub_attacker_position = false;
  analysis.sub_attacker_position_score = 0.0f;

  // ボールが自陣にある場合はSubAttacker不要
  if (wm->point_checker.isInOurHalf(wm->ball().pos)) {
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
    return;
  }

  // 各候補点のスコアを計算
  double best_score = -std::numeric_limits<double>::infinity();
  Point best_position{0.0, 0.0};

  for (const auto & candidate : valid_candidates) {
    double score = evaluateSubAttackerPosition(candidate, wm);

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
}

}  // namespace crane::metrics
