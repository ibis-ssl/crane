// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__KICK_PREDICTION_TRACKER_HPP_
#define CRANE_MSG_WRAPPERS__KICK_PREDICTION_TRACKER_HPP_

#include <Eigen/Dense>
#include <crane_msgs/msg/kick_prediction_trace.hpp>
#include <string>

#include "tracker_base.hpp"

namespace crane
{

/**
 * @brief キック予測の予実管理を行うユーティリティクラス
 *
 * KickerModelのキック力とボール初速度・停止距離の予測精度を検証し、
 * 物理モデルのパラメータ調整やデバッグに活用する。
 */
class KickPredictionTracker
: public TrackerBase<KickPredictionTracker, crane_msgs::msg::KickPredictionTrace>
{
public:
  /**
   * @brief キック予測を記録
   * @param trace トレースデータ
   * @param source 予測作成元 ("skill", "session", "kick_event_detector")
   * @param kick_power キック力 [0.0-1.0]
   * @param is_chip_kick チップキックか
   * @param predicted_ball_speed 予測ボール初速度 [m/s]
   * @param predicted_stop_distance 予測停止距離 [m]
   * @param kick_position キック時のボール位置（フィールド座標 m）
   */
  static void recordPrediction(
    crane_msgs::msg::KickPredictionTrace & trace, const std::string & source, double kick_power,
    bool is_chip_kick, double predicted_ball_speed, double predicted_stop_distance,
    const Eigen::Vector2d & kick_position);

  /**
   * @brief 実績を記録し予実比較を実行
   * @param trace トレースデータ
   * @param actual_ball_speed 実際のボール初速度 [m/s]
   * @param actual_stop_distance 実際の停止距離 [m]
   *
   * 予測値と比較し、誤差を計算してactualに追加する。
   */
  static void recordActual(
    crane_msgs::msg::KickPredictionTrace & trace, double actual_ball_speed,
    double actual_stop_distance);
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__KICK_PREDICTION_TRACKER_HPP_
