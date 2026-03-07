// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/kick_prediction_tracker.hpp"

#include <chrono>
#include <cmath>

namespace crane
{

void KickPredictionTracker::recordPrediction(
  crane_msgs::msg::KickPredictionTrace & trace, const std::string & source, double kick_power,
  bool is_chip_kick, double predicted_ball_speed, double predicted_stop_distance,
  const Eigen::Vector2d & kick_position)
{
  crane_msgs::msg::KickPredictionPoint point;

  point.source = source;
  point.kick_power = static_cast<float>(kick_power);
  point.is_chip_kick = is_chip_kick;
  point.predicted_ball_speed = static_cast<float>(predicted_ball_speed);
  point.predicted_stop_distance = static_cast<float>(predicted_stop_distance);
  point.kick_pos_x = static_cast<float>(kick_position.x());
  point.kick_pos_y = static_cast<float>(kick_position.y());

  // キック時刻を記録（基準からのマイクロ秒）
  auto now = std::chrono::system_clock::now();
  auto now_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  point.kick_timestamp_us = (now_ns - trace.reference_timestamp_ns) / 1000;

  trace.prediction_point.push_back(point);
}

void KickPredictionTracker::recordActual(
  crane_msgs::msg::KickPredictionTrace & trace, double actual_ball_speed,
  double actual_stop_distance)
{
  // 予測点がない場合は何もしない
  if (trace.prediction_point.empty()) {
    return;
  }

  // 予測点を取得
  const auto & prediction = trace.prediction_point.front();

  crane_msgs::msg::KickPredictionActual actual;

  actual.predicted_ball_speed = prediction.predicted_ball_speed;
  actual.predicted_stop_distance = prediction.predicted_stop_distance;
  actual.actual_ball_speed = static_cast<float>(actual_ball_speed);
  actual.actual_stop_distance = static_cast<float>(actual_stop_distance);

  // 速度誤差を計算
  actual.speed_error = actual.actual_ball_speed - actual.predicted_ball_speed;
  actual.speed_error_percent = (actual.predicted_ball_speed > 0.0f)
                                 ? (actual.speed_error / actual.predicted_ball_speed * 100.0f)
                                 : 0.0f;

  // 距離誤差を計算
  actual.distance_error = actual.actual_stop_distance - actual.predicted_stop_distance;
  actual.distance_error_percent =
    (actual.predicted_stop_distance > 0.0f)
      ? (actual.distance_error / actual.predicted_stop_distance * 100.0f)
      : 0.0f;

  trace.actual.push_back(actual);
}

}  // namespace crane
