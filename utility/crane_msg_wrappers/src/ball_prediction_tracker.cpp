// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/ball_prediction_tracker.hpp"

#include <chrono>
#include <cmath>

namespace crane
{

// トレースID生成用のカウンター初期化
uint32_t BallPredictionTracker::trace_id_counter_ = 0;

auto BallPredictionTracker::createTrace() -> crane_msgs::msg::BallPredictionTrace
{
  crane_msgs::msg::BallPredictionTrace trace;

  // 基準タイムスタンプを設定（現在時刻のナノ秒）
  auto now = std::chrono::system_clock::now();
  trace.reference_timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  // トレースIDを割り当て
  trace.trace_id = ++trace_id_counter_;

  return trace;
}

void BallPredictionTracker::addPredictionPoint(
  crane_msgs::msg::BallPredictionTrace & trace, const std::string & source,
  const Eigen::Vector3d & predicted_pos, const Eigen::Vector3d & predicted_vel,
  int32_t target_time_us, uint8_t predicted_state)
{
  crane_msgs::msg::BallPredictionPoint point;

  point.source = source;
  point.target_time_us = target_time_us;
  point.predicted_pos_x = static_cast<float>(predicted_pos.x());
  point.predicted_pos_y = static_cast<float>(predicted_pos.y());
  point.predicted_pos_z = static_cast<float>(predicted_pos.z());
  point.predicted_vel_x = static_cast<float>(predicted_vel.x());
  point.predicted_vel_y = static_cast<float>(predicted_vel.y());
  point.predicted_vel_z = static_cast<float>(predicted_vel.z());
  point.predicted_state = predicted_state;

  trace.prediction_points.push_back(point);
}

void BallPredictionTracker::recordActual(
  crane_msgs::msg::BallPredictionTrace & trace, const Eigen::Vector3d & actual_pos,
  const Eigen::Vector3d & actual_vel, uint8_t actual_state)
{
  // 最新の予測点がない場合は何もしない
  if (trace.prediction_points.empty()) {
    return;
  }

  // 最新の予測点を取得
  const auto & latest_prediction = trace.prediction_points.back();

  crane_msgs::msg::BallPredictionActual actual;

  actual.prediction_time_us = latest_prediction.target_time_us;
  actual.predicted_pos_x = latest_prediction.predicted_pos_x;
  actual.predicted_pos_y = latest_prediction.predicted_pos_y;
  actual.predicted_pos_z = latest_prediction.predicted_pos_z;
  actual.predicted_vel_x = latest_prediction.predicted_vel_x;
  actual.predicted_vel_y = latest_prediction.predicted_vel_y;

  actual.actual_pos_x = static_cast<float>(actual_pos.x());
  actual.actual_pos_y = static_cast<float>(actual_pos.y());
  actual.actual_pos_z = static_cast<float>(actual_pos.z());
  actual.actual_vel_x = static_cast<float>(actual_vel.x());
  actual.actual_vel_y = static_cast<float>(actual_vel.y());
  actual.actual_state = actual_state;

  // 位置誤差を計算（3D距離）
  Eigen::Vector3d predicted_pos(
    latest_prediction.predicted_pos_x, latest_prediction.predicted_pos_y,
    latest_prediction.predicted_pos_z);
  Eigen::Vector3d pos_error = actual_pos - predicted_pos;
  actual.position_error = static_cast<float>(pos_error.norm());

  // 速度誤差を計算（2D速度のみ）
  Eigen::Vector2d predicted_vel(
    latest_prediction.predicted_vel_x, latest_prediction.predicted_vel_y);
  Eigen::Vector2d actual_vel_2d(actual_vel.x(), actual_vel.y());
  Eigen::Vector2d vel_error = actual_vel_2d - predicted_vel;
  actual.velocity_error = static_cast<float>(vel_error.norm());

  // リングバッファとして動作（最大10件まで）
  trace.actuals.push_back(actual);
  if (trace.actuals.size() > MAX_ACTUALS) {
    trace.actuals.erase(trace.actuals.begin());
  }
}

}  // namespace crane
