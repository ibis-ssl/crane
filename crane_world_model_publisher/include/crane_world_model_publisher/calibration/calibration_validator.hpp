// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__CALIBRATION_VALIDATOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__CALIBRATION_VALIDATOR_HPP_

#include <crane_world_model_publisher/ball_physics_model.hpp>
#include <memory>

#include "ball_calibration_data_extractor.hpp"
#include "simple_ball_physics_optimizer.hpp"

namespace crane
{

/**
 * @brief キャリブレーション結果の検証器
 */
class CalibrationValidator
{
public:
  /**
   * @brief 検証メトリクス
   */
  struct ValidationMetrics
  {
    double position_rmse = 0.0;           // 位置予測RMSE [m]
    double velocity_rmse = 0.0;           // 速度予測RMSE [m/s]
    double stop_time_error = 0.0;         // 停止時間誤差 [s]
    double max_distance_error = 0.0;      // 最大移動距離誤差 [m]
    double speed_prediction_error = 0.0;  // 速度予測誤差 [m/s]

    // 統計情報
    size_t validated_trajectories = 0;  // 検証した軌道数
    double mean_absolute_error = 0.0;   // 平均絶対誤差 [m]
    double max_absolute_error = 0.0;    // 最大絶対誤差 [m]

    // 成功率
    double prediction_accuracy = 0.0;  // 予測精度 (0.0-1.0)
    bool validation_passed = false;    // 検証合格フラグ
  };

  /**
   * @brief 検証設定
   */
  struct ValidatorConfig
  {
    double max_position_rmse = 0.2;     // 許容位置RMSE [m]
    double max_velocity_rmse = 0.5;     // 許容速度RMSE [m/s]
    double max_stop_time_error = 0.3;   // 許容停止時間誤差 [s]
    double max_speed_error = 0.5;       // 許容速度誤差 [m/s]
    size_t min_validation_samples = 5;  // 最小検証サンプル数
    double accuracy_threshold = 0.8;    // 合格精度閾値
  };

  /**
   * @brief コンストラクタ
   */
  CalibrationValidator();

  /**
   * @brief 設定の更新
   */
  auto setConfig(const ValidatorConfig & config) -> void;

  /**
   * @brief キャリブレーション結果の検証
   * @param kick_data テストデータ
   * @param physics_config 物理モデル設定
   * @param kicker_model キッカーモデル
   * @return 検証メトリクス
   */
  auto validateCalibration(
    const std::vector<KickDataPoint> & kick_data, const BallPhysicsModel::Config & physics_config,
    const SimpleKickerCalibrator::KickerModel & kicker_model) -> ValidationMetrics;

  /**
   * @brief 個別軌道の予測精度評価
   * @param actual_trajectory 実測軌道
   * @param physics_model 物理モデル
   * @return 予測誤差 [m]
   */
  auto evaluateTrajectoryPrediction(
    const std::vector<Ball> & actual_trajectory, const BallPhysicsModel & physics_model) -> double;

  /**
   * @brief キッカーモデルの速度予測精度評価
   * @param kick_data キックデータ
   * @param kicker_model キッカーモデル
   * @return 速度予測誤差 [m/s]
   */
  auto evaluateKickerPrediction(
    const std::vector<KickDataPoint> & kick_data,
    const SimpleKickerCalibrator::KickerModel & kicker_model) -> double;

private:
  ValidatorConfig config_;

  /**
   * @brief 予測軌道と実測軌道の比較
   */
  auto compareTrajectories(
    const std::vector<Ball> & actual_trajectory, const std::vector<Point> & predicted_positions,
    const std::vector<Point> & predicted_velocities) -> ValidationMetrics;

  /**
   * @brief 停止時間の予測と実測の比較
   */
  auto compareStopTimes(const std::vector<Ball> & actual_trajectory, double predicted_stop_time)
    -> double;

  /**
   * @brief 最大移動距離の予測と実測の比較
   */
  auto compareMaxDistances(
    const std::vector<Ball> & actual_trajectory, double predicted_max_distance) -> double;
};

/**
 * @brief キャリブレーション品質レポート生成器
 */
class CalibrationReportGenerator
{
public:
  /**
   * @brief 品質レポート構造体
   */
  struct QualityReport
  {
    std::string summary;                              // 概要
    CalibrationValidator::ValidationMetrics metrics;  // 検証メトリクス
    std::vector<std::string> recommendations;         // 推奨事項
    bool overall_quality_good = false;                // 総合品質評価
  };

  /**
   * @brief 品質レポートの生成
   * @param metrics 検証メトリクス
   * @param physics_result 物理最適化結果
   * @param kicker_result キッカー最適化結果
   * @return 品質レポート
   */
  auto generateQualityReport(
    const CalibrationValidator::ValidationMetrics & metrics,
    const SimpleBallPhysicsOptimizer::OptimizationResult & physics_result,
    const SimpleKickerCalibrator::CalibrationResult & kicker_result) -> QualityReport;

  /**
   * @brief レポートのテキスト出力
   * @param report 品質レポート
   * @return フォーマットされたテキスト
   */
  auto formatReportText(const QualityReport & report) -> std::string;

private:
  /**
   * @brief 推奨事項の生成
   */
  auto generateRecommendations(
    const CalibrationValidator::ValidationMetrics & metrics,
    const SimpleBallPhysicsOptimizer::OptimizationResult & physics_result,
    const SimpleKickerCalibrator::CalibrationResult & kicker_result) -> std::vector<std::string>;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__CALIBRATION_VALIDATOR_HPP_
