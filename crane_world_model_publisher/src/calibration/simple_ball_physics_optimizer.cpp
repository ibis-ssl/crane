// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/calibration/simple_ball_physics_optimizer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace crane
{

SimpleBallPhysicsOptimizer::SimpleBallPhysicsOptimizer()
{
  // デフォルト設定
}

auto SimpleBallPhysicsOptimizer::setConfig(const OptimizerConfig & config) -> void
{
  config_ = config;
}

auto SimpleBallPhysicsOptimizer::optimizeDecelerationParameter(const std::vector<KickDataPoint> & kick_data)
  -> OptimizationResult
{
  OptimizationResult result;
  
  if (kick_data.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SimpleBallPhysicsOptimizer"), 
                "キックデータが空です");
    return result;
  }
  
  // ストレートキックのデータのみを抽出
  std::vector<KickDataPoint> straight_kicks;
  std::copy_if(kick_data.begin(), kick_data.end(), std::back_inserter(straight_kicks),
               [](const KickDataPoint & kd) { return !kd.is_chip_kick; });
  
  if (straight_kicks.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SimpleBallPhysicsOptimizer"), 
                "ストレートキックデータが見つかりません");
    return result;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
              "最適化開始: %zu個のストレートキックデータを使用", straight_kicks.size());
  
  // 軌道データを時系列ポイントに変換
  std::vector<std::pair<double, Point>> all_trajectory_points;
  for (const auto & kick_point : straight_kicks) {
    if (kick_point.trajectory.size() < 3) continue;
    
    // 初期速度と方向の取得
    const Ball & initial_ball = kick_point.trajectory[0];
    double initial_speed = initial_ball.vel.norm();
    Point initial_direction = initial_ball.vel.normalized();
    
    if (initial_speed < 0.5) continue; // 低速キックはスキップ
    
    // 軌道点をフラット化
    for (size_t i = 0; i < kick_point.trajectory.size(); ++i) {
      const Ball & ball = kick_point.trajectory[i];
      double time_offset = i * 0.016; // 仮定: 60FPSデータ
      all_trajectory_points.emplace_back(time_offset, ball.pos);
    }
  }
  
  if (all_trajectory_points.size() < 10) {
    RCLCPP_WARN(rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
                "軌道データ点数が不足: %zu点", all_trajectory_points.size());
    return result;
  }
  
  // 最適化ループ（グリッドサーチ + 最小二乗法）
  double best_deceleration = 0.5;
  double min_error = std::numeric_limits<double>::max();
  
  // 粗い検索
  for (double decel = config_.min_deceleration; 
       decel <= config_.max_deceleration; 
       decel += 0.05) {
    
    double total_error = 0.0;
    size_t valid_points = 0;
    
    for (const auto & kick_point : straight_kicks) {
      if (kick_point.trajectory.size() < 3) continue;
      
      const Ball & initial_ball = kick_point.trajectory[0];
      double initial_speed = initial_ball.vel.norm();
      Point initial_position = initial_ball.pos;
      Point initial_direction = initial_ball.vel.normalized();
      
      // 予測軌道を計算
      std::vector<double> time_points;
      for (size_t i = 0; i < kick_point.trajectory.size(); ++i) {
        time_points.push_back(i * 0.016);
      }
      
      auto predicted_trajectory = predictRollingTrajectory(
        initial_position, initial_ball.vel, decel, time_points);
      
      // 実測軌道
      std::vector<std::pair<double, Point>> actual_trajectory;
      for (size_t i = 0; i < kick_point.trajectory.size(); ++i) {
        actual_trajectory.emplace_back(time_points[i], kick_point.trajectory[i].pos);
      }
      
      // 誤差計算
      double trajectory_error = calculateTrajectoryError(actual_trajectory, predicted_trajectory);
      total_error += trajectory_error;
      valid_points++;
    }
    
    if (valid_points > 0) {
      double avg_error = total_error / valid_points;
      if (avg_error < min_error) {
        min_error = avg_error;
        best_deceleration = decel;
      }
    }
  }
  
  // 細かい最適化（ベスト値周辺）
  double fine_step = 0.001;
  for (double decel = best_deceleration - 0.05; 
       decel <= best_deceleration + 0.05; 
       decel += fine_step) {
    
    if (decel < config_.min_deceleration || decel > config_.max_deceleration) continue;
    
    double total_error = 0.0;
    size_t valid_points = 0;
    
    for (const auto & kick_point : straight_kicks) {
      if (kick_point.trajectory.size() < 3) continue;
      
      const Ball & initial_ball = kick_point.trajectory[0];
      Point initial_position = initial_ball.pos;
      
      std::vector<double> time_points;
      for (size_t i = 0; i < kick_point.trajectory.size(); ++i) {
        time_points.push_back(i * 0.016);
      }
      
      auto predicted_trajectory = predictRollingTrajectory(
        initial_position, initial_ball.vel, decel, time_points);
      
      std::vector<std::pair<double, Point>> actual_trajectory;
      for (size_t i = 0; i < kick_point.trajectory.size(); ++i) {
        actual_trajectory.emplace_back(time_points[i], kick_point.trajectory[i].pos);
      }
      
      double trajectory_error = calculateTrajectoryError(actual_trajectory, predicted_trajectory);
      total_error += trajectory_error;
      valid_points++;
    }
    
    if (valid_points > 0) {
      double avg_error = total_error / valid_points;
      if (avg_error < min_error) {
        min_error = avg_error;
        best_deceleration = decel;
      }
    }
  }
  
  // 結果の設定
  result.success = true;
  result.optimized_deceleration = best_deceleration;
  result.residual_error = min_error;
  result.data_points_used = straight_kicks.size();
  result.mean_prediction_error = min_error;
  
  // R²の計算
  double total_variance = 0.0;
  double explained_variance = 0.0;
  size_t total_points = 0;
  
  for (const auto & kick_point : straight_kicks) {
    if (kick_point.trajectory.size() < 3) continue;
    
    std::vector<double> time_points;
    for (size_t i = 0; i < kick_point.trajectory.size(); ++i) {
      time_points.push_back(i * 0.016);
    }
    
    auto predicted_trajectory = predictRollingTrajectory(
      kick_point.trajectory[0].pos, kick_point.trajectory[0].vel, 
      best_deceleration, time_points);
    
    std::vector<std::pair<double, Point>> actual_trajectory;
    for (size_t i = 0; i < kick_point.trajectory.size(); ++i) {
      actual_trajectory.emplace_back(time_points[i], kick_point.trajectory[i].pos);
    }
    
    result.r_squared = calculateRSquared(actual_trajectory, predicted_trajectory);
    break; // 最初の軌道のみでR²を概算
  }
  
  RCLCPP_INFO(rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
              "最適化完了: 減速度=%.4f m/s², 誤差=%.4f m, R²=%.3f",
              result.optimized_deceleration, result.residual_error, result.r_squared);
  
  return result;
}

auto SimpleBallPhysicsOptimizer::predictRollingTrajectory(
  const Point & initial_position,
  const Point & initial_velocity,
  double deceleration,
  const std::vector<double> & time_points
) -> std::vector<std::pair<double, Point>>
{
  std::vector<std::pair<double, Point>> predicted_trajectory;
  
  double initial_speed = initial_velocity.norm();
  if (initial_speed < 1e-6) {
    // 初期速度が0の場合、全て初期位置
    for (double t : time_points) {
      predicted_trajectory.emplace_back(t, initial_position);
    }
    return predicted_trajectory;
  }
  
  Point direction = initial_velocity.normalized();
  double stop_time = initial_speed / deceleration;
  
  for (double t : time_points) {
    Point predicted_pos;
    
    if (t >= stop_time) {
      // 停止後
      double distance_at_stop = initial_speed * stop_time - 0.5 * deceleration * stop_time * stop_time;
      predicted_pos = initial_position + direction * distance_at_stop;
    } else {
      // 減速中
      double distance = initial_speed * t - 0.5 * deceleration * t * t;
      predicted_pos = initial_position + direction * distance;
    }
    
    predicted_trajectory.emplace_back(t, predicted_pos);
  }
  
  return predicted_trajectory;
}

auto SimpleBallPhysicsOptimizer::calculateTrajectoryError(
  const std::vector<std::pair<double, Point>> & actual_trajectory,
  const std::vector<std::pair<double, Point>> & predicted_trajectory
) -> double
{
  if (actual_trajectory.size() != predicted_trajectory.size()) {
    return std::numeric_limits<double>::max();
  }
  
  double total_squared_error = 0.0;
  for (size_t i = 0; i < actual_trajectory.size(); ++i) {
    Point actual_pos = actual_trajectory[i].second;
    Point predicted_pos = predicted_trajectory[i].second;
    double distance_error = (actual_pos - predicted_pos).norm();
    total_squared_error += distance_error * distance_error;
  }
  
  return std::sqrt(total_squared_error / actual_trajectory.size()); // RMSE
}

auto SimpleBallPhysicsOptimizer::calculateRSquared(
  const std::vector<std::pair<double, Point>> & actual_trajectory,
  const std::vector<std::pair<double, Point>> & predicted_trajectory
) -> double
{
  if (actual_trajectory.size() != predicted_trajectory.size() || actual_trajectory.empty()) {
    return 0.0;
  }
  
  // 平均位置の計算
  Point mean_actual = Point::Zero();
  for (const auto & [t, pos] : actual_trajectory) {
    mean_actual += pos;
  }
  mean_actual /= static_cast<double>(actual_trajectory.size());
  
  // 総変動と残差変動の計算
  double ss_tot = 0.0;  // 総変動
  double ss_res = 0.0;  // 残差変動
  
  for (size_t i = 0; i < actual_trajectory.size(); ++i) {
    Point actual_pos = actual_trajectory[i].second;
    Point predicted_pos = predicted_trajectory[i].second;
    
    ss_tot += (actual_pos - mean_actual).squaredNorm();
    ss_res += (actual_pos - predicted_pos).squaredNorm();
  }
  
  if (ss_tot < 1e-10) return 1.0; // 全て同じ位置の場合
  
  return 1.0 - (ss_res / ss_tot);
}

// SimpleKickerCalibrator の実装
SimpleKickerCalibrator::SimpleKickerCalibrator() {}

auto SimpleKickerCalibrator::calibrateKickerModel(const std::vector<KickDataPoint> & kick_data)
  -> CalibrationResult
{
  CalibrationResult result;
  
  // ストレートキックとチップキックのデータを分離
  std::vector<std::pair<double, double>> straight_power_speed;
  std::vector<std::pair<double, double>> chip_power_speed;
  
  for (const auto & kick_point : kick_data) {
    if (kick_point.trajectory.empty()) continue;
    
    double initial_speed = extractInitialSpeed(kick_point.trajectory);
    if (initial_speed < 0.1) continue; // 低速キックはスキップ
    
    if (kick_point.is_chip_kick) {
      chip_power_speed.emplace_back(kick_point.kick_power, initial_speed);
    } else {
      straight_power_speed.emplace_back(kick_point.kick_power, initial_speed);
    }
  }
  
  // ストレートキックモデルの最適化
  if (straight_power_speed.size() >= 3) {
    result.straight_kick_model = performLinearRegression(straight_power_speed);
    RCLCPP_INFO(rclcpp::get_logger("SimpleKickerCalibrator"),
                "ストレートキック: slope=%.2f, offset=%.2f, R²=%.3f (%zu points)",
                result.straight_kick_model.linear_coefficient,
                result.straight_kick_model.offset,
                result.straight_kick_model.r_squared,
                straight_power_speed.size());
  }
  
  // チップキックモデルの最適化
  if (chip_power_speed.size() >= 3) {
    result.chip_kick_model = performLinearRegression(chip_power_speed);
    RCLCPP_INFO(rclcpp::get_logger("SimpleKickerCalibrator"),
                "チップキック: slope=%.2f, offset=%.2f, R²=%.3f (%zu points)",
                result.chip_kick_model.linear_coefficient,
                result.chip_kick_model.offset,
                result.chip_kick_model.r_squared,
                chip_power_speed.size());
  }
  
  result.success = (straight_power_speed.size() >= 3) || (chip_power_speed.size() >= 3);
  
  return result;
}

auto SimpleKickerCalibrator::performLinearRegression(
  const std::vector<std::pair<double, double>> & power_speed_pairs
) -> KickerModel
{
  KickerModel model;
  
  if (power_speed_pairs.size() < 2) {
    return model;
  }
  
  size_t n = power_speed_pairs.size();
  double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0, sum_y2 = 0.0;
  
  for (const auto & [power, speed] : power_speed_pairs) {
    sum_x += power;
    sum_y += speed;
    sum_xy += power * speed;
    sum_x2 += power * power;
    sum_y2 += speed * speed;
  }
  
  double mean_x = sum_x / n;
  double mean_y = sum_y / n;
  
  // 線形回帰の計算
  double denominator = sum_x2 - n * mean_x * mean_x;
  if (std::abs(denominator) > 1e-10) {
    model.linear_coefficient = (sum_xy - n * mean_x * mean_y) / denominator;
    model.offset = mean_y - model.linear_coefficient * mean_x;
    
    // R²の計算
    double ss_tot = sum_y2 - n * mean_y * mean_y;
    double ss_res = 0.0;
    for (const auto & [power, speed] : power_speed_pairs) {
      double predicted = model.linear_coefficient * power + model.offset;
      ss_res += (speed - predicted) * (speed - predicted);
    }
    
    if (ss_tot > 1e-10) {
      model.r_squared = 1.0 - (ss_res / ss_tot);
    }
  }
  
  model.data_points_used = n;
  
  return model;
}

auto SimpleKickerCalibrator::extractInitialSpeed(const std::vector<Ball> & trajectory) -> double
{
  if (trajectory.empty()) return 0.0;
  
  // 最初の数点の最大速度を初期速度とする
  double max_speed = 0.0;
  size_t max_check = std::min(static_cast<size_t>(3), trajectory.size());
  
  for (size_t i = 0; i < max_check; ++i) {
    max_speed = std::max(max_speed, trajectory[i].vel.norm());
  }
  
  return max_speed;
}

auto SimpleKickerCalibrator::calculateRequiredPower(double target_speed, const KickerModel & model) -> double
{
  if (std::abs(model.linear_coefficient) < 1e-10) {
    return 0.5; // デフォルト値
  }
  
  double required_power = (target_speed - model.offset) / model.linear_coefficient;
  return std::clamp(required_power, 0.0, 1.0);
}

} // namespace crane