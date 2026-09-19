// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_

#include "crane_game_analyzer/selection_hysteresis.hpp"
#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief 推奨アタッカーメトリクス
 *
 * ボール距離とSlack時間を考慮して推奨Attackerを決定
 * OUR_SLACK, BALL_THREATに依存
 */
class AttackerCandidateMetric : public MetricBase
{
public:
  AttackerCandidateMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override
  {
    return {MetricId::OUR_SLACK, MetricId::THEIR_SLACK, MetricId::BALL_THREAT};
  }

  auto compute(MetricContext & ctx) -> void override;

private:
  // ヒステリシス管理
  SelectionHysteresis<int> attacker_hysteresis_{SelectionHysteresis<int>::Config{
    // 200ms → 500ms → 1.0s。即切替の近道を外した後も、保持時間が切れた直後に
    // 役割が入れ替わって PassPlanMetric の再計算が走り、成立していた計画が
    // 消えることがあった。パスは計画成立からキックまで約1秒かかるので、
    // その間は出し手を固定できる長さにする。
    // 急変時は force_switch_timeout(3秒) が最終的な逃げ道になる。
    .min_hold_duration_sec = 1.0,
    .min_improvement_ratio = 0.15,  // 1% → 15%: 僅差での無意味な切替を防止
    // 即切替の近道（emergency_switch_ratio / absolute_threshold）は使わない。
    //
    // このメトリクスのスコアは `10.0 / 到達距離` に二値条件の乗算
    // （インターセプト地点が無ければ ×0.3、敵が先着なら ×0.2）を掛けた値で、
    // 二値条件が反転すると 1フレームで 3〜5倍跳ねる。実測では 0.08 秒で
    // 2.02 → 9.42（4.7倍）という変化が出ており、EMA(α=0.3) でも吸収しきれない。
    // このためスコア差や倍率では「本当に優位になった」のか「条件が反転しただけ」
    // なのかを区別できず、どんな閾値を置いても素通りする。
    //
    // 結果として役割が 0.02〜0.51 秒間隔で振動し、PassPlanMetric が出した
    // score 1.23 の良い計画が 0.25 秒で壊れていた（出し手が 1→5→6 と飛ぶ）。
    // 判別できない以上は保持時間で守るしかない。急を要する場合は
    // force_switch_timeout が 3 秒で必ず切り替える。
    .emergency_switch_ratio = 0.0,
    .force_switch_timeout = 3.0,
    .absolute_threshold = 0.0}};

  // EMAスコア管理用マップ
  std::unordered_map<uint8_t, double> ema_scores_;

  // EMAスムージング係数
  static constexpr double EMA_ALPHA = 0.3;  // 新スコア30%、旧70%でスコア安定化
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_
