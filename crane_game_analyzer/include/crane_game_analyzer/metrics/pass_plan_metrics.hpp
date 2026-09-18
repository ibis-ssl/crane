// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__PASS_PLAN_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__PASS_PLAN_METRICS_HPP_

#include <crane_msgs/msg/pass_plan.hpp>
#include <crane_physics/pass_feasibility.hpp>
#include <crane_physics/pass_rating_math.hpp>
#include <crane_physics/slack_time_config.hpp>
#include <optional>
#include <vector>

#include "crane_game_analyzer/selection_hysteresis.hpp"
#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief 通常プレーの出し手と受け手が共有する直進パス計画
 *
 * 各味方（goalie/kicker 除く）の現在位置と周辺グリッド点を候補受領点とし、
 * feasibility ゲート（受け手先着）で安価に早期棄却→ratePassCandidate で採点→
 * ヒステリシス選定して PassPlan を生成する。キック検知後は受け手と受領点を固定し、
 * 停止・中断・タイムアウトで解除する。
 *
 * コスト: 本メトリクスは analyzer 共有の game_analysis publish に相乗りするため、
 * 重い再計算は recompute_interval_sec_ でデシメーションし、直近プランをキャッシュして
 * 毎フレーム完全再送する。これにより (a) GameAnalysis の int 既定値0による幻ロボット参照を
 * 防ぎ（未書込フィールドを残さない）、(b) 間引き tick 間の状態フリッカを防ぐ。
 */
class PassPlanMetric : public MetricBase
{
public:
  PassPlanMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override
  {
    return {MetricId::ONGOING_KICK, MetricId::ATTACKER_CANDIDATE};
  }

  auto compute(MetricContext & ctx) -> void override;
  auto visualize(MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer)
    -> void override;

  // パラメータ設定（crane_game_analyzer.cpp から注入）
  auto setRecomputeInterval(double sec) -> void { recompute_interval_sec_ = sec; }
  auto setFeasibilityParams(const ReceiveFeasibilityParams & params) -> void
  {
    feasibility_ = params;
  }
  auto setDppsParams(double r_resolution, double r_max, int theta_div) -> void
  {
    dpps_r_resolution_ = r_resolution;
    dpps_r_max_ = r_max;
    dpps_theta_div_ = theta_div;
  }
  auto setMaxCandidates(int n) -> void { max_candidates_ = n; }
  auto setMinPassScore(double s) -> void { min_pass_score_ = s; }
  /// 受領点に要求するフィールド境界からの余裕 [m]
  auto setReceivePointFieldMargin(double m) -> void { receive_point_field_margin_ = m; }
  /// 計画保持中に適用する下限の緩和率（1.0 で緩和なし）
  auto setMinPassScoreReleaseRatio(double r) -> void { min_pass_score_release_ratio_ = r; }
  auto setReceivePointImprovement(double ratio) -> void { receive_point_improvement_ = ratio; }
  auto setEnemySlackConfig(const SlackTimeConfig & config, double slack_scale = 1.0) -> void
  {
    enemy_slack_config_ = config;
    slack_scale_ = slack_scale;
  }

private:
  // 実際にプランを再計算する（デシメーションで間引かれる）。cached_plan_ を更新
  auto recomputePlan(MetricContext & ctx) -> void;

  // 有効プランが得られないときの非アクティブプランを cached_plan_ に書き込む
  auto writeInactivePlan(int kicker_id) -> void;

  // 受け手ID選定（第1レベルヒステリシス）
  SelectionHysteresis<int> receiver_hysteresis_{SelectionHysteresis<int>::Config{
    .min_hold_duration_sec = 0.5,
    .min_improvement_ratio = 0.2,
  }};

  // 直近プラン（毎フレーム完全再送するためのキャッシュ）
  crane_msgs::msg::PassPlan cached_plan_;

  // 受領点保持（第2レベル: 同一受け手内で改善が閾値未満なら点を動かさない）
  std::optional<Point> held_receive_point_;
  int held_receiver_id_ = -1;
  double receive_point_improvement_ = 0.2;  // 保持点を置換するのに必要な改善率

  // デシメーション
  double recompute_interval_sec_ = 0.1;  // 既定 10Hz
  std::optional<rclcpp::Time> last_recompute_time_;
  rclcpp::Clock::SharedPtr selection_clock_;
  std::optional<rclcpp::Time> flight_started_at_;
  Point planned_origin_ = Point::Zero();
  uint32_t plan_seq_ = 0;

  // 計測（コスト可視化: 直近再計算で feasibility+採点を試みた候補数）
  int last_evaluated_ = 0;
  // 棄却理由の内訳。評価候補が残っているのに計画が立たないとき、
  // feasibility とスコアのどちらで落ちたかはログに出ないと判別できない。
  int last_rejected_infeasible_ = 0;
  int last_rejected_low_score_ = 0;
  /// 味方の横取りで落とした候補数。受領点の競合と経路横断の両方を含む。
  int last_rejected_friendly_ = 0;
  double last_best_score_ = 0.0;
  /// 最良候補のスコア内訳。どの係数が下限割れの原因かをログで切り分ける。
  PassRating last_best_rating_{};
  Point last_best_point_ = Point::Zero();

  // 候補生成パラメータ（getDPPSPoints）
  double dpps_r_resolution_ = 0.3;
  double dpps_r_max_ = 2.5;
  int dpps_theta_div_ = 16;
  int max_candidates_ = 800;  // feasibility+採点の打ち切り上限（超過はログ）

  // 評価パラメータ
  ReceiveFeasibilityParams feasibility_{};
  SlackTimeConfig enemy_slack_config_{
    .robot_max_acceleration = 3.0,  // 敵は自チームより高め（安全マージン）
    .robot_max_velocity = 5.5,      // 敵は自チームより高め（安全マージン）
  };
  double slack_scale_ = 1.0;
  double min_pass_score_ = 0.5;
  /// 受領点をフィールド境界からどれだけ内側に限るか [m]。
  /// フィールド内であることだけを条件にすると、敵から最も遠いという理由で
  /// タッチライン際（実測ではラインまで 0.1m）の受領点が選ばれる。
  /// そこは受け手が後ろに回り込めず、Receive の software bumper が
  /// ボール進行方向へ目標をずらす先も場外になるため、受け損なって
  /// ボールがそのまま出る（実測: 9試行中5回が OUT_OF_PLAY）。
  /// 実行できない計画を立てないよう、境界に余裕を要求する。
  double receive_point_field_margin_ = 0.5;
  /// 計画を保持している間だけ下限を緩める比率（シュミットトリガ）。
  /// 受け手選定と受領点にはヒステリシスがあるのに、スコア判定には無かった。
  /// スコアが下限のすぐ上に居座る配置では、10Hz の再計算ごとに計画が
  /// 成立・消滅を繰り返す（実測: score 0.22〜0.37 / 下限 0.20 で、
  /// PLANNING が 0.06〜0.57 秒しか続かなかった）。
  /// 取得は min_pass_score_、保持はその release_ratio 倍で判定する。
  double min_pass_score_release_ratio_ = 0.6;

  // 可視化専用サブレイヤ（KickEventDetector と同様に自前ビルダーを所有・自己 flush）
  VisualizerMessageBuilder::SharedPtr viz_ =
    std::make_shared<VisualizerMessageBuilder>("analyzer/pass_plan");
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__PASS_PLAN_METRICS_HPP_
