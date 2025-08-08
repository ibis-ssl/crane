// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/calibration/calibration_validator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace crane
{

CalibrationValidator::CalibrationValidator() {}

auto CalibrationValidator::setConfig(const ValidatorConfig & config) -> void
{
  config_ = config;
}

auto CalibrationValidator::validateCalibration(
  const std::vector<KickDataPoint> & kick_data,
  const BallPhysicsModel::Config & physics_config,
  const SimpleKickerCalibrator::KickerModel & kicker_model
) -> ValidationMetrics
{
  ValidationMetrics metrics;
  
  if (kick_data.size() < config_.min_validation_samples) {
    RCLCPP_WARN(rclcpp::get_logger("CalibrationValidator"),
                "検証データ不足: %zu個 (最小必要数: %zu)",
                kick_data.size(), config_.min_validation_samples);
    return metrics;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("CalibrationValidator"),
              "キャリブレーション検証開始: %zu個の軌道を検証", kick_data.size());
  
  // 物理モデルの生成
  BallPhysicsModel physics_model(physics_config);
  
  std::vector<double> position_errors;
  std::vector<double> velocity_errors;
  std::vector<double> stop_time_errors;
  std::vector<double> max_distance_errors;
  std::vector<double> speed_prediction_errors;
  
  metrics.validated_trajectories = 0;
  
  for (const auto & kick_point : kick_data) {
    if (kick_point.trajectory.size() < 3) continue;
    
    const Ball & initial_ball = kick_point.trajectory[0];
    
    // 軌道予測の評価
    double trajectory_error = evaluateTrajectoryPrediction(kick_point.trajectory, physics_model);
    if (trajectory_error < std::numeric_limits<double>::max()) {
      position_errors.push_back(trajectory_error);
    }
    
    // 停止時間予測の評価
    double predicted_stop_time = physics_model.getStopTime(
      initial_ball.vel, initial_ball.state, initial_ball.vel_z);
    double stop_time_error = compareStopTimes(kick_point.trajectory, predicted_stop_time);
    if (stop_time_error >= 0) {
      stop_time_errors.push_back(stop_time_error);
    }
    
    // 最大移動距離予測の評価
    double predicted_max_distance = physics_model.getMaxDistance(
      initial_ball.pos, initial_ball.vel, initial_ball.state, 
      initial_ball.pos_z, initial_ball.vel_z);
    double max_distance_error = compareMaxDistances(kick_point.trajectory, predicted_max_distance);
    if (max_distance_error >= 0) {
      max_distance_errors.push_back(max_distance_error);
    }
    
    // キッカーモデルの速度予測評価
    double predicted_speed = kicker_model.linear_coefficient * kick_point.kick_power + 
                            kicker_model.offset;
    double actual_initial_speed = initial_ball.vel.norm();
    double speed_error = std::abs(predicted_speed - actual_initial_speed);
    speed_prediction_errors.push_back(speed_error);
    
    metrics.validated_trajectories++;
  }
  
  // メトリクスの計算
  if (!position_errors.empty()) {
    double sum_squared = std::accumulate(position_errors.begin(), position_errors.end(), 0.0,
                                        [](double sum, double err) { return sum + err * err; });
    metrics.position_rmse = std::sqrt(sum_squared / position_errors.size());
    metrics.mean_absolute_error = std::accumulate(position_errors.begin(), position_errors.end(), 0.0) 
                                 / position_errors.size();
    metrics.max_absolute_error = *std::max_element(position_errors.begin(), position_errors.end());
  }
  
  if (!stop_time_errors.empty()) {
    metrics.stop_time_error = std::accumulate(stop_time_errors.begin(), stop_time_errors.end(), 0.0) 
                             / stop_time_errors.size();
  }
  
  if (!max_distance_errors.empty()) {
    metrics.max_distance_error = std::accumulate(max_distance_errors.begin(), max_distance_errors.end(), 0.0) 
                                / max_distance_errors.size();
  }
  
  if (!speed_prediction_errors.empty()) {
    metrics.speed_prediction_error = std::accumulate(speed_prediction_errors.begin(), speed_prediction_errors.end(), 0.0) 
                                    / speed_prediction_errors.size();
  }
  
  // 予測精度の計算
  size_t good_predictions = 0;
  if (metrics.position_rmse <= config_.max_position_rmse) good_predictions++;
  if (metrics.speed_prediction_error <= config_.max_speed_error) good_predictions++;
  if (metrics.stop_time_error <= config_.max_stop_time_error) good_predictions++;
  
  metrics.prediction_accuracy = static_cast<double>(good_predictions) / 3.0;
  metrics.validation_passed = (metrics.prediction_accuracy >= config_.accuracy_threshold);
  
  RCLCPP_INFO(rclcpp::get_logger("CalibrationValidator"),
              "検証完了: RMSE=%.3f m, 速度誤差=%.3f m/s, 精度=%.1f%% (%s)",
              metrics.position_rmse, metrics.speed_prediction_error,
              metrics.prediction_accuracy * 100.0,
              metrics.validation_passed ? "合格" : "不合格");
  
  return metrics;
}

auto CalibrationValidator::evaluateTrajectoryPrediction(
  const std::vector<Ball> & actual_trajectory,
  const BallPhysicsModel & physics_model
) -> double
{
  if (actual_trajectory.size() < 2) {
    return std::numeric_limits<double>::max();
  }
  
  const Ball & initial_ball = actual_trajectory[0];
  
  double total_squared_error = 0.0;
  size_t valid_predictions = 0;
  
  for (size_t i = 1; i < actual_trajectory.size(); ++i) {
    double time_ahead = i * 0.016; // 仮定: 60FPSデータ
    
    Point predicted_pos = physics_model.predictPosition(
      initial_ball.pos, initial_ball.vel, initial_ball.state,
      initial_ball.pos_z, initial_ball.vel_z, time_ahead);
    
    Point actual_pos = actual_trajectory[i].pos;
    double error = (predicted_pos - actual_pos).norm();
    
    total_squared_error += error * error;
    valid_predictions++;
  }
  
  if (valid_predictions == 0) {
    return std::numeric_limits<double>::max();
  }
  
  return std::sqrt(total_squared_error / valid_predictions); // RMSE
}

auto CalibrationValidator::compareStopTimes(
  const std::vector<Ball> & actual_trajectory,
  double predicted_stop_time
) -> double
{
  // 実際の停止時間を軌道から推定
  double actual_stop_time = -1.0;
  
  for (size_t i = 0; i < actual_trajectory.size(); ++i) {
    if (actual_trajectory[i].vel.norm() < 0.1) {
      actual_stop_time = i * 0.016; // 仮定: 60FPSデータ
      break;
    }
  }
  
  if (actual_stop_time < 0) {
    // 軌道データ内で停止しなかった場合
    actual_stop_time = actual_trajectory.size() * 0.016;
  }
  
  return std::abs(predicted_stop_time - actual_stop_time);
}

auto CalibrationValidator::compareMaxDistances(
  const std::vector<Ball> & actual_trajectory,
  double predicted_max_distance
) -> double
{
  if (actual_trajectory.empty()) return -1.0;
  
  const Point & initial_pos = actual_trajectory[0].pos;
  
  double actual_max_distance = 0.0;
  for (const auto & ball : actual_trajectory) {
    double distance = (ball.pos - initial_pos).norm();
    actual_max_distance = std::max(actual_max_distance, distance);
  }
  
  return std::abs(predicted_max_distance - actual_max_distance);
}

// CalibrationReportGenerator の実装
auto CalibrationReportGenerator::generateQualityReport(
  const ValidationMetrics & metrics,
  const SimpleBallPhysicsOptimizer::OptimizationResult & physics_result,
  const SimpleKickerCalibrator::CalibrationResult & kicker_result
) -> QualityReport
{
  QualityReport report;
  report.metrics = metrics;
  
  // 総合品質評価
  report.overall_quality_good = metrics.validation_passed && 
                                physics_result.r_squared > 0.8 && 
                                kicker_result.straight_kick_model.r_squared > 0.7;
  
  // 概要の生成
  std::stringstream summary;
  summary << std::fixed << std::setprecision(3);
  summary << "キャリブレーション品質評価\n";
  summary << "=========================\n";
  summary << "総合評価: " << (report.overall_quality_good ? "良好" : "要改善") << "\n";
  summary << "検証軌道数: " << metrics.validated_trajectories << "\n";
  summary << "位置予測RMSE: " << metrics.position_rmse << " m\n";
  summary << "速度予測誤差: " << metrics.speed_prediction_error << " m/s\n";
  summary << "物理モデルR²: " << physics_result.r_squared << "\n";
  summary << "キッカーモデルR²: " << kicker_result.straight_kick_model.r_squared << "\n";
  report.summary = summary.str();
  
  // 推奨事項の生成
  report.recommendations = generateRecommendations(metrics, physics_result, kicker_result);
  
  return report;
}

auto CalibrationReportGenerator::generateRecommendations(
  const ValidationMetrics & metrics,
  const SimpleBallPhysicsOptimizer::OptimizationResult & physics_result,
  const SimpleKickerCalibrator::CalibrationResult & kicker_result
) -> std::vector<std::string>
{
  std::vector<std::string> recommendations;
  
  // 位置予測精度の評価
  if (metrics.position_rmse > 0.15) {
    recommendations.push_back("位置予測精度が低下しています。より多くの高品質な軌道データを収集することを推奨します。");
  }
  
  // 物理モデルの決定係数評価
  if (physics_result.r_squared < 0.8) {
    recommendations.push_back("物理モデルの当てはまりが悪いです。データの外れ値除去や追加パラメータの調整を検討してください。");
  }
  
  // キッカーモデルの評価
  if (kicker_result.straight_kick_model.r_squared < 0.7) {
    recommendations.push_back("キッカーパワー-速度関係の線形性が低いです。より広範囲のパワー設定でのデータ収集が必要です。");
  }
  
  // データ量の評価
  if (physics_result.data_points_used < 20) {
    recommendations.push_back("最適化に使用されたデータ点数が少ないです。より多くのキックデータを収集してください。");
  }
  
  // 速度予測精度の評価
  if (metrics.speed_prediction_error > 0.5) {
    recommendations.push_back("キック速度の予測精度が低下しています。キッカーの機械的なばらつきや測定精度を確認してください。");
  }
  
  if (recommendations.empty()) {
    recommendations.push_back("キャリブレーション品質は良好です。現在のパラメータを本番環境に適用できます。");
  }
  
  return recommendations;
}

auto CalibrationReportGenerator::formatReportText(const QualityReport & report) -> std::string
{
  std::stringstream formatted;
  
  formatted << report.summary << "\n";
  
  formatted << "\n推奨事項:\n";
  formatted << "========\n";
  for (size_t i = 0; i < report.recommendations.size(); ++i) {
    formatted << (i + 1) << ". " << report.recommendations[i] << "\n";
  }
  
  formatted << "\n詳細メトリクス:\n";
  formatted << "============\n";
  formatted << std::fixed << std::setprecision(4);
  formatted << "平均絶対誤差: " << report.metrics.mean_absolute_error << " m\n";
  formatted << "最大絶対誤差: " << report.metrics.max_absolute_error << " m\n";
  formatted << "停止時間誤差: " << report.metrics.stop_time_error << " s\n";
  formatted << "予測精度: " << (report.metrics.prediction_accuracy * 100.0) << "%\n";
  
  return formatted.str();
}

} // namespace crane