// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__BALL_PREDICTION_TRACKER_HPP_
#define CRANE_MSG_WRAPPERS__BALL_PREDICTION_TRACKER_HPP_

#include <Eigen/Dense>
#include <crane_msgs/msg/ball_prediction_trace.hpp>
#include <string>

namespace crane
{

/**
 * @brief ボール予測の予実管理を行うユーティリティクラス
 *
 * PassTargetSelector、Goalie、プランナーでのボール予測精度を検証し、
 * 物理モデルのパラメータ調整やデバッグに活用する。
 */
class BallPredictionTracker
{
public:
  /**
   * @brief 新しいトレースを作成
   * @return 初期化されたBallPredictionTrace
   */
  static auto createTrace() -> crane_msgs::msg::BallPredictionTrace;

  /**
   * @brief 予測点を追加
   * @param trace トレースデータ
   * @param source 予測作成元 ("pass_selector", "goalie", "planner", "world_model")
   * @param predicted_pos 予測位置（フィールド座標 m）
   * @param predicted_vel 予測速度（フィールド座標 m/s）
   * @param target_time_us 予測対象時刻（基準からの相対時間 us）
   * @param predicted_state 予測されたボール状態（STOPPED/ROLLING/FLYING）
   */
  static void addPredictionPoint(
    crane_msgs::msg::BallPredictionTrace & trace, const std::string & source,
    const Eigen::Vector3d & predicted_pos, const Eigen::Vector3d & predicted_vel,
    int32_t target_time_us, uint8_t predicted_state);

  /**
   * @brief 実績を記録し予実比較を実行
   * @param trace トレースデータ
   * @param actual_pos 実際の位置（フィールド座標 m）
   * @param actual_vel 実際の速度（フィールド座標 m/s）
   * @param actual_state 実際の状態（STOPPED/ROLLING/FLYING）
   *
   * 最新の予測点と比較し、誤差を計算してactualsに追加する。
   * リングバッファとして動作し、最大10件まで保持する。
   */
  static void recordActual(
    crane_msgs::msg::BallPredictionTrace & trace, const Eigen::Vector3d & actual_pos,
    const Eigen::Vector3d & actual_vel, uint8_t actual_state);

private:
  // トレースID生成用のカウンター
  static uint32_t trace_id_counter_;

  // リングバッファの最大サイズ
  static constexpr size_t MAX_ACTUALS = 10;
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__BALL_PREDICTION_TRACKER_HPP_
