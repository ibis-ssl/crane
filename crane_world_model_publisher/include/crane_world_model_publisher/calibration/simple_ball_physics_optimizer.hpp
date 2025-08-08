// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__SIMPLE_BALL_PHYSICS_OPTIMIZER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__SIMPLE_BALL_PHYSICS_OPTIMIZER_HPP_

#include "ball_calibration_data_extractor.hpp"
#include <crane_world_model_publisher/ball_physics_model.hpp>
#include <Eigen/Dense>
#include <memory>

namespace crane
{

/**
 * @brief 最小二乗法によるボール物理パラメータ最適化器
 */
class SimpleBallPhysicsOptimizer
{
public:
  /**
   * @brief 最適化結果構造体
   */
  struct OptimizationResult
  {
    bool success = false;
    double optimized_deceleration = 0.5;  // 最適化された減速度 [m/s²]
    double residual_error = 0.0;          // 残差誤差 (RMSE)
    size_t data_points_used = 0;          // 最適化に使用されたデータ点数
    size_t iterations = 0;                // 反復回数
    double r_squared = 0.0;               // 決定係数 (R²)
    
    // 詳細統計
    double mean_prediction_error = 0.0;   // 平均予測誤差
    double max_prediction_error = 0.0;    // 最大予測誤差  
    std::vector<double> position_errors;  // 各時点での位置誤差
  };
  
  /**
   * @brief 最適化設定
   */
  struct OptimizerConfig
  {
    double min_deceleration = 0.1;        // 最小減速度 [m/s²]
    double max_deceleration = 2.0;        // 最大減速度 [m/s²] 
    double convergence_threshold = 1e-6;  // 収束閾値
    size_t max_iterations = 100;          // 最大反復回数
    double min_trajectory_duration = 0.5; // 最小軌道継続時間 [s]
    double outlier_threshold = 3.0;       // 外れ値除去の標準偏差倍率
    bool remove_outliers = true;          // 外れ値除去を有効化
  };
  
  /**
   * @brief コンストラクタ
   */
  SimpleBallPhysicsOptimizer();
  
  /**
   * @brief 設定の更新
   */
  auto setConfig(const OptimizerConfig & config) -> void;
  
  /**
   * @brief キックデータからの減速度パラメータ最適化
   * @param kick_data キックデータポイント
   * @return 最適化結果
   */
  auto optimizeDecelerationParameter(const std::vector<KickDataPoint> & kick_data) 
    -> OptimizationResult;
  
  /**
   * @brief 単一軌道からの減速度推定（デバッグ用）
   * @param trajectory ボール軌道データ
   * @return 推定された減速度
   */
  auto estimateDecelerationFromTrajectory(const std::vector<Ball> & trajectory) -> double;

private:
  OptimizerConfig config_;
  
  /**
   * @brief 転がりボール軌道の最小二乗法による最適化
   */
  auto optimizeRollingTrajectory(
    const std::vector<std::pair<double, Point>> & trajectory_points,
    double initial_speed,
    const Point & initial_direction
  ) -> double;
  
  /**
   * @brief 予測軌道と実測軌道の誤差計算
   */
  auto calculateTrajectoryError(
    const std::vector<std::pair<double, Point>> & actual_trajectory,
    const std::vector<std::pair<double, Point>> & predicted_trajectory
  ) -> double;
  
  /**
   * @brief 外れ値の除去
   */
  auto removeOutliers(
    std::vector<std::pair<double, Point>> & trajectory_points,
    double deceleration
  ) -> size_t;
  
  /**
   * @brief 決定係数（R²）の計算
   */
  auto calculateRSquared(
    const std::vector<std::pair<double, Point>> & actual_trajectory,
    const std::vector<std::pair<double, Point>> & predicted_trajectory
  ) -> double;
  
  /**
   * @brief 転がり軌道予測
   */
  auto predictRollingTrajectory(
    const Point & initial_position,
    const Point & initial_velocity,
    double deceleration,
    const std::vector<double> & time_points
  ) -> std::vector<std::pair<double, Point>>;
};

/**
 * @brief 線形回帰によるキッカーパワー-速度関係最適化器
 */
class SimpleKickerCalibrator
{
public:
  /**
   * @brief キッカーモデル（線形関係）
   */
  struct KickerModel
  {
    double linear_coefficient = 2.0;  // 線形係数 [m/s per power]
    double offset = 0.0;              // オフセット [m/s]
    double r_squared = 0.0;           // 決定係数
    size_t data_points_used = 0;      // 使用データ点数
  };
  
  /**
   * @brief キャリブレーション結果
   */
  struct CalibrationResult
  {
    bool success = false;
    KickerModel straight_kick_model;  // ストレートキックモデル
    KickerModel chip_kick_model;      // チップキックモデル
    double residual_error = 0.0;      // 残差誤差
  };
  
  /**
   * @brief コンストラクタ
   */
  SimpleKickerCalibrator();
  
  /**
   * @brief キックデータからパワー-速度関係の最適化
   * @param kick_data キックデータポイント
   * @return キャリブレーション結果
   */
  auto calibrateKickerModel(const std::vector<KickDataPoint> & kick_data) 
    -> CalibrationResult;
  
  /**
   * @brief 速度からキックパワーの逆算
   * @param target_speed 目標速度 [m/s]
   * @param model キッカーモデル
   * @return 必要なキックパワー [0.0-1.0]
   */
  auto calculateRequiredPower(double target_speed, const KickerModel & model) -> double;

private:
  /**
   * @brief 線形回帰の実行
   */
  auto performLinearRegression(
    const std::vector<std::pair<double, double>> & power_speed_pairs
  ) -> KickerModel;
  
  /**
   * @brief 初期速度の抽出
   */
  auto extractInitialSpeed(const std::vector<Ball> & trajectory) -> double;
};

} // namespace crane

#endif // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__SIMPLE_BALL_PHYSICS_OPTIMIZER_HPP_