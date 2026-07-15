// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__PASS_ORIGIN_HPP_
#define CRANE_GAME_ANALYZER__METRICS__PASS_ORIGIN_HPP_

#include <algorithm>

#include "metric_context.hpp"

namespace crane::metrics
{

/**
 * @brief パス起点を決定する（PassTargetMetric / PassPlanMetric 共有ヘルパ）
 *
 * ボールが検出かつ停止していれば現在位置、検出かつ移動中なら停止予測位置、
 * それ以外は履歴の直近検出→進行中キック起点→現在位置の順にフォールバックする。
 * 両メトリクスが同一の起点を用いることで、新旧パス評価の比較可視化が同じ前提に
 * 立てるようにするための共有関数（挙動は元の PassTargetMetric::computePassOrigin と同一）。
 */
[[nodiscard]] inline auto computePassOrigin(const MetricContext & ctx) -> Point
{
  const auto & ball = ctx.world_model->ball();
  // 検出かつ停止
  if (ball.isStopped() && ball.detected) {
    return ball.pos;
  }
  // 検出かつ移動
  if (ball.detected && ball.isMoving()) {
    return ball.getPredictedPosition(std::min(ball.getStopTime(), 1.0));
  }
  // 履歴から直近検出
  for (auto it = ctx.ball_history->begin(); it != ctx.ball_history->end(); ++it) {
    if (it->detected) {
      return Point(it->position.x, it->position.y);
    }
  }
  // キック起点
  if (not ctx.analysis.ongoing_kick.empty()) {
    const auto & k = ctx.analysis.ongoing_kick.front();
    return Point(k.origin_x, k.origin_y);
  }
  // フォールバック
  return ball.pos;
}

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__PASS_ORIGIN_HPP_
