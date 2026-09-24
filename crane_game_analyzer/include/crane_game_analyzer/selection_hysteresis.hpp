// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__SELECTION_HYSTERESIS_HPP_
#define CRANE_GAME_ANALYZER__SELECTION_HYSTERESIS_HPP_

#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace crane
{

/**
 * @brief 選択ヒステリシス管理テンプレートクラス
 *
 * 「前回ID保持 + 切替時刻 + 保持時間 + 改善判定」をカプセル化する。
 * 各フラグが0の場合はその条件をスキップする。
 */
template <typename IdType>
class SelectionHysteresis
{
public:
  struct Config
  {
    double min_hold_duration_sec = 0.0;   ///< 保持時間（秒）
    double min_improvement_ratio = 0.0;   ///< 保持期間後の切替に必要な最小改善率（0=制限なし）
    double emergency_switch_ratio = 0.0;  ///< 即切替に必要な倍率（0=この条件を課さない）
    double force_switch_timeout = 0.0;    ///< このタイムアウト後は小改善でも切替（0=未使用）
    double absolute_threshold = 0.0;      ///< 即切替に必要な絶対差（0=この条件を課さない）
  };

  explicit SelectionHysteresis(
    Config config, rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  : config_(config), clock_(std::move(clock))
  {
  }

  /**
   * @brief 切替判定を行い、内部状態を更新する
   * @param best_id 最高スコアのID
   * @param best_score 最高スコア値
   * @param current_score 現在選択中のIDのスコア（不明の場合は0.0）
   * @return true なら切替実施済み
   */
  auto shouldSwitch(IdType best_id, double best_score, double current_score) -> bool
  {
    const auto now = clock_->now();

    if (!current_id_.has_value()) {
      current_id_ = best_id;
      last_switch_time_ = now;
      return true;
    }

    if (best_id == *current_id_) {
      return false;
    }

    const double time_since_switch = (now - last_switch_time_).seconds();
    const double score_diff = best_score - current_score;
    const double improvement_ratio = score_diff / std::max(current_score, 0.1);

    // 即切替の2条件。未設定（0以下）の条件は課さない＝自動的に満たすとみなす。
    const bool ratio_satisfied = config_.emergency_switch_ratio <= 0.0 ||
                                 best_score >= current_score * config_.emergency_switch_ratio;
    const bool absolute_satisfied =
      config_.absolute_threshold <= 0.0 || score_diff >= config_.absolute_threshold;
    const bool immediate_switch_configured =
      config_.emergency_switch_ratio > 0.0 || config_.absolute_threshold > 0.0;

    bool should = false;

    if (config_.force_switch_timeout > 0.0 && time_since_switch >= config_.force_switch_timeout) {
      should = (score_diff > 0.1);
    } else if (immediate_switch_configured && ratio_satisfied && absolute_satisfied) {
      // min_hold_duration_sec を待たずに切り替えるのは「明らかに優位」なときだけ。
      //
      // 比率と絶対差は、設定されている方を**すべて**満たす必要がある。どちらか一方
      // だけで即切替を許すと、スコアが正規化されていない（AttackerMetric の実測で
      // 0.7〜30 まで振れる）ために、どちらの条件も単独では機能しない:
      //   - 比率だけ: 小スコア領域で 1.50→2.28（1.52倍・差 0.78）が通る
      //   - 絶対差だけ: 大スコア領域で 21.40→23.94（差 2.54・改善わずか 11.9%）が通る
      // 実測では、これらが 0.02〜0.32 秒間隔の役割振動を起こし、
      // PassPlanMetric の再計算を強制して計画が安定しなかった。
      //
      // 片方しか設定していない利用者の挙動は変わらない（未設定の条件は自動的に真）。
      should = true;
    } else if (time_since_switch >= config_.min_hold_duration_sec) {
      should = (config_.min_improvement_ratio <= 0.0) ||
               (improvement_ratio >= config_.min_improvement_ratio);
    }

    if (should) {
      current_id_ = best_id;
      last_switch_time_ = now;
    }

    return should;
  }

  [[nodiscard]] auto currentId() const -> std::optional<IdType> { return current_id_; }

  [[nodiscard]] auto timeSinceSwitch() const -> double
  {
    return (clock_->now() - last_switch_time_).seconds();
  }

  auto forceSwitch(IdType id) -> void
  {
    current_id_ = id;
    last_switch_time_ = clock_->now();
  }

  auto reset() -> void
  {
    current_id_ = std::nullopt;
    last_switch_time_ = rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME);
  }

  auto setConfig(Config config) -> void { config_ = config; }

private:
  Config config_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::optional<IdType> current_id_;
  rclcpp::Time last_switch_time_{static_cast<int64_t>(0), RCL_ROS_TIME};
};

}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__SELECTION_HYSTERESIS_HPP_
