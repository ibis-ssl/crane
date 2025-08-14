// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__SIMPLE_BALL_PHYSICS_OPTIMIZER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__SIMPLE_BALL_PHYSICS_OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace crane
{
/**
 * @brief JSONベースボール物理パラメータ最適化器
 */
class SimpleBallPhysicsOptimizer
{
public:
  /**
   * @brief キックパワー-速度ペア構造体
   */
  struct KickPowerVelocityPair
  {
    size_t event_id;                                // イベントID
    double kick_power;                              // キックパワー [0.0-1.0]
    double estimated_initial_velocity;              // 推定初速度 [m/s]
    bool is_chip_kick;                              // チップキックフラグ
    double fitting_r_squared;                       // フィッティングR²値
    double trajectory_duration;                     // 軌道継続時間 [s]
    std::pair<double, double> confidence_interval;  // 信頼区間
  };

  /**
   * @brief 最適化結果構造体
   */
  struct OptimizationResult
  {
    bool success = false;
    double global_deceleration = 0.0;              // グローバル減速度 [m/s²]
    double global_rmse = 0.0;                      // グローバルRMSE
    double global_r_squared = 0.0;                 // グローバルR²値
    size_t trajectories_analyzed = 0;              // 解析した軌道数
    size_t trajectories_used = 0;                  // 有効軌道数
    std::vector<KickPowerVelocityPair> kick_data;  // キックデータ
  };

  /**
   * @brief 最適化設定
   */
  struct OptimizationConfig
  {
    std::string json_directory_path;             // JSONディレクトリパス
    double min_trajectory_duration = 0.5;        // 最小軌道継続時間 [s]
    double velocity_outlier_threshold = 2.0;     // 速度外れ値閾値 [σ]
    size_t min_data_points_per_trajectory = 10;  // 軌道あたり最小データ点数
    double min_fitting_r_squared = 0.6;          // 最小フィッティングR²値
    double min_deceleration = 0.1;               // 最小減速度 [m/s²]
    double max_deceleration = 2.0;               // 最大減速度 [m/s²]
  };

  /**
   * @brief コンストラクタ
   */
  SimpleBallPhysicsOptimizer();

  /**
   * @brief 設定の更新
   */
  auto setConfig(const OptimizationConfig & config) -> void;

  /**
   * @brief JSONディレクトリからの最適化実行
   * @param config 最適化設定
   * @return 最適化結果
   */
  auto optimizeFromJSONDirectory(const OptimizationConfig & config) -> OptimizationResult;

  /**
   * @brief キックパワー分析結果のJSON出力
   * @param output_path 出力ファイルパス
   * @param result 最適化結果
   * @return 成功フラグ
   */
  auto exportKickPowerAnalysis(const std::string & output_path, const OptimizationResult & result)
    -> bool;

private:
  OptimizationConfig config_;

  /**
   * @brief 軌道データ構造体
   */
  struct TrajectoryData
  {
    size_t event_id;
    double kick_power;
    bool is_chip_kick;
    std::vector<double> time_points;
    std::vector<double> velocities;
    std::vector<double> positions_x;
    std::vector<double> positions_y;

    // JSONのlinear_fit結果を保存
    struct LinearFitResult
    {
      double slope = 0.0;
      double intercept = 0.0;
      double r_squared = 0.0;
      size_t data_points = 0;
    } linear_fit;
  };

  /**
   * @brief JSONファイルからの軌道データ読み込み
   */
  auto loadTrajectoryDataFromJSON(const std::string & json_file_path) -> TrajectoryData;

  /**
   * @brief ディレクトリから全JSONファイルの軌道データを読み込み
   */
  auto loadAllTrajectoryData(const std::string & directory_path) -> std::vector<TrajectoryData>;

  /**
   * @brief グローバル減速度パラメータの最適化
   */
  auto optimizeGlobalDeceleration(const std::vector<TrajectoryData> & all_trajectories) -> double;

  /**
   * @brief 個別軌道の初速度推定（線形回帰）
   */
  auto estimateInitialVelocity(const TrajectoryData & trajectory, double deceleration)
    -> KickPowerVelocityPair;

  /**
   * @brief 線形回帰の実行
   */
  auto performLinearRegression(
    const std::vector<double> & x_data, const std::vector<double> & y_data)
    -> std::tuple<double, double, double>;  // slope, intercept, r_squared

  /**
   * @brief データ品質フィルタリング
   */
  auto filterQualityData(const std::vector<TrajectoryData> & trajectories)
    -> std::vector<TrajectoryData>;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__CALIBRATION__SIMPLE_BALL_PHYSICS_OPTIMIZER_HPP_
