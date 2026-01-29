// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__VELOCITY_PLAN_TRACKER_HPP_
#define CRANE_MSG_WRAPPERS__VELOCITY_PLAN_TRACKER_HPP_

#include <Eigen/Dense>
#include <crane_msgs/msg/velocity_plan_trace.hpp>
#include <string>

namespace crane
{

/**
 * @brief 速度計画の予実管理を行うユーティリティクラス
 *
 * 上流（スキル層）から下流（sender）まで一貫したトレースを提供し、
 * 計画された速度と実際の速度を比較・検証できるようにする。
 */
class VelocityPlanTracker
{
public:
  /**
   * @brief 新しいトレースを作成
   * @return 初期化されたVelocityPlanTrace
   */
  static auto createTrace() -> crane_msgs::msg::VelocityPlanTrace;

  /**
   * @brief 計画点を追加
   * @param trace トレースデータ
   * @param source 計画作成元 ("skill", "session", "local_planner", "sender")
   * @param predicted_pos 予測位置（フィールド座標 m）
   * @param predicted_vel 予測速度（フィールド座標 m/s）
   * @param target_time_us 予測対象時刻（基準からの相対時間 us）
   * @param estimated_arrival_time_us 目標到達予定時刻（us）
   */
  static void addPlanPoint(
    crane_msgs::msg::VelocityPlanTrace & trace, const std::string & source,
    const Eigen::Vector2d & predicted_pos, const Eigen::Vector2d & predicted_vel,
    int32_t target_time_us, int32_t estimated_arrival_time_us = 0);

  /**
   * @brief 速度修正を記録
   * @param trace トレースデータ
   * @param source 修正元 ("rvo2", "sender_accel_limit", "feedback_control")
   * @param before_vel 修正前の希望速度（m/s）
   * @param after_vel 修正後の実際の速度（m/s）
   */
  static void addCorrection(
    crane_msgs::msg::VelocityPlanTrace & trace, const std::string & source,
    const Eigen::Vector2d & before_vel, const Eigen::Vector2d & after_vel);

  /**
   * @brief 実績を記録し予実比較を実行
   * @param trace トレースデータ
   * @param actual_pos 実際の位置（フィールド座標 m）
   * @param actual_vel 実際の速度（フィールド座標 m/s）
   *
   * 最新の計画点と比較し、誤差を計算してactualsに追加する。
   * リングバッファとして動作し、最大10件まで保持する。
   */
  static void recordActual(
    crane_msgs::msg::VelocityPlanTrace & trace, const Eigen::Vector2d & actual_pos,
    const Eigen::Vector2d & actual_vel);

private:
  // トレースID生成用のカウンター
  static uint32_t trace_id_counter_;

  // リングバッファの最大サイズ
  static constexpr size_t MAX_ACTUALS = 10;
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__VELOCITY_PLAN_TRACKER_HPP_
