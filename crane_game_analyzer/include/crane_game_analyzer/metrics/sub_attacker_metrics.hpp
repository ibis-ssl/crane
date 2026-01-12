// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__SUB_ATTACKER_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__SUB_ATTACKER_METRICS_HPP_

#include <crane_geometry/boost_geometry.hpp>

#include "metric_base.hpp"
#include "metric_context.hpp"

namespace crane::metrics
{

/**
 * @brief SubAttacker推奨位置を計算するメトリクス
 *
 * ボール周辺のDPPS候補点から最適なSubAttacker配置位置を計算し、
 * GameAnalysisに推奨位置を設定する
 */
class SubAttackerPositionMetric : public MetricBase
{
public:
  SubAttackerPositionMetric();

  /**
   * @brief このメトリクスは他のメトリクスに依存しない
   */
  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override { return {}; }

  /**
   * @brief SubAttacker推奨位置を計算
   *
   * 1. ボールが自陣にある場合は無効化
   * 2. getDPPSPoints() でボール周りの候補点を生成
   * 3. フィールド内・ペナルティエリア外でフィルタ
   * 4. SubAttacker::getPointScore() で各点のスコア計算
   * 5. 最高スコアの点を推奨位置として設定
   *
   * @param ctx 計算コンテキスト
   */
  auto compute(MetricContext & ctx) -> void override;

private:
  // ヒステリシス用状態（位置の急変を防ぐ）
  Point last_position_{0.0, 0.0};
  bool has_last_position_ = false;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__SUB_ATTACKER_METRICS_HPP_
