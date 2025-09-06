// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/calibration/simple_ball_physics_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <regex>

namespace crane
{

SimpleBallPhysicsOptimizer::SimpleBallPhysicsOptimizer()
{
  // デフォルト設定
}

auto SimpleBallPhysicsOptimizer::setConfig(const OptimizationConfig & config) -> void
{
  config_ = config;
}

auto SimpleBallPhysicsOptimizer::optimizeFromJSONDirectory(const OptimizationConfig & config)
  -> OptimizationResult
{
  OptimizationResult result;
  config_ = config;

  RCLCPP_INFO(
    rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "JSONディレクトリから最適化開始: %s",
    config.json_directory_path.c_str());

  // 全軌道データの読み込み
  auto all_trajectories = loadAllTrajectoryData(config.json_directory_path);
  result.trajectories_analyzed = all_trajectories.size();

  if (all_trajectories.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
      "JSONファイルが見つからないか、データが読み込めませんでした");
    return result;
  }

  // データ品質フィルタリング
  auto filtered_trajectories = filterQualityData(all_trajectories);
  result.trajectories_used = filtered_trajectories.size();

  if (filtered_trajectories.size() < 3) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
      "有効な軌道データが不足: %zu個 (最低3個必要)", filtered_trajectories.size());
    return result;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "データフィルタリング完了: %zu/%zu軌道が有効",
    result.trajectories_used, result.trajectories_analyzed);

  // グローバル減速度パラメータの最適化
  double global_deceleration = optimizeGlobalDeceleration(filtered_trajectories);
  result.global_deceleration = global_deceleration;

  if (global_deceleration <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "グローバル減速度最適化に失敗");
    return result;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "グローバル減速度最適化完了: %.4f m/s²",
    global_deceleration);

  // 個別軌道の初速度推定
  std::vector<KickPowerVelocityPair> kick_data;
  double total_rmse = 0.0;
  size_t valid_estimations = 0;

  size_t rejected_count = 0;
  for (const auto & trajectory : filtered_trajectories) {
    auto kick_pair = estimateInitialVelocity(trajectory, global_deceleration);
    if (kick_pair.fitting_r_squared >= config.min_fitting_r_squared) {
      kick_data.push_back(kick_pair);
      total_rmse += (1.0 - kick_pair.fitting_r_squared);  // R²ベースのRMSE近似
      valid_estimations++;
    } else {
      rejected_count++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
    "個別軌道推定: 有効=%zu個, 除外=%zu個 (R²閾値=%.2f)", valid_estimations, rejected_count,
    config.min_fitting_r_squared);

  result.kick_data = kick_data;
  result.global_rmse = (valid_estimations > 0) ? total_rmse / valid_estimations : 1.0;
  result.global_r_squared = 1.0 - result.global_rmse;

  result.success = (kick_data.size() >= 3);

  RCLCPP_INFO(
    rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
    "最適化完了: 減速度=%.4f, RMSE=%.4f, キックデータ=%zu個", result.global_deceleration,
    result.global_rmse, result.kick_data.size());

  return result;
}

auto SimpleBallPhysicsOptimizer::loadTrajectoryDataFromJSON(const std::string & json_file_path)
  -> TrajectoryData
{
  TrajectoryData trajectory;

  try {
    std::ifstream file(json_file_path);
    if (!file.is_open()) {
      RCLCPP_WARN(
        rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "JSONファイルを開けませんでした: %s",
        json_file_path.c_str());
      return trajectory;
    }

    nlohmann::json json_data;
    file >> json_data;

    // ファイル名からイベントIDを抽出
    std::regex pattern(R"(kick_event_visualization_(\d+)_data\.json)");
    std::smatch matches;
    std::string filename = std::filesystem::path(json_file_path).filename().string();
    if (std::regex_search(filename, matches, pattern)) {
      trajectory.event_id = std::stoul(matches[1].str());
    }

    // キック情報の抽出
    if (json_data.contains("kick_info")) {
      const auto & kick_info = json_data["kick_info"];
      trajectory.kick_power = kick_info.value("kick_power", 0.0);
      trajectory.is_chip_kick = kick_info.value("is_chip_kick", false);
    }

    // 軌道データの抽出（実際のJSONファイル構造に対応）
    if (json_data.contains("data")) {
      const auto & data = json_data["data"];

      if (
        data.contains("timestamp_ns") && data.contains("position") && data.contains("ball_state")) {
        const auto & timestamp_array = data["timestamp_ns"];
        const auto & position = data["position"];
        const auto & ball_state_array = data["ball_state"];

        if (position.contains("x") && position.contains("y")) {
          const auto & pos_x_array = position["x"];
          const auto & pos_y_array = position["y"];

          size_t data_size = std::min(
            {timestamp_array.size(), pos_x_array.size(), pos_y_array.size(),
             ball_state_array.size()});

          // ball_state配列を使用して有効な軌道部分のみを抽出
          // STOPPED=0, ROLLING=1, FLYING=2, INVALID=3（フィールド外）
          std::vector<size_t> valid_indices;
          for (size_t i = 0; i < data_size; ++i) {
            int ball_state = ball_state_array[i].get<int>();
            if (ball_state == 1) {  // 1 = ROLLING（有効な軌道）のみ使用
              valid_indices.push_back(i);
            }
            // INVALID(3)やSTOPPED(0)は自動的に除外される
          }

          if (valid_indices.size() < config_.min_data_points_per_trajectory) {
            RCLCPP_WARN(
              rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
              "有効軌道データ点が不足: %zu (最小必要数: %zu)", valid_indices.size(),
              config_.min_data_points_per_trajectory);
            return trajectory;
          }

          trajectory.time_points.reserve(valid_indices.size());
          trajectory.positions_x.reserve(valid_indices.size());
          trajectory.positions_y.reserve(valid_indices.size());
          trajectory.velocities.reserve(valid_indices.size());

          // 有効なインデックスのみのデータを抽出
          std::vector<double> timestamps_sec;
          timestamps_sec.reserve(valid_indices.size());

          for (size_t idx : valid_indices) {
            double timestamp_ns = timestamp_array[idx].get<double>();
            double timestamp_sec = timestamp_ns * 1e-9;  // ナノ秒から秒に変換
            timestamps_sec.push_back(timestamp_sec);

            trajectory.positions_x.push_back(pos_x_array[idx].get<double>());
            trajectory.positions_y.push_back(pos_y_array[idx].get<double>());
          }

          // 相対時間に変換（最初のタイムスタンプを0とする）
          double start_time = timestamps_sec[0];
          for (double timestamp : timestamps_sec) {
            trajectory.time_points.push_back(timestamp - start_time);
          }

          // 位置データから速度を計算（有効データのみ）
          for (size_t i = 0; i < trajectory.time_points.size(); ++i) {
            if (i == 0) {
              // 最初の点では次の点との差分で速度を計算
              if (trajectory.time_points.size() > 1) {
                double dx = trajectory.positions_x[1] - trajectory.positions_x[0];
                double dy = trajectory.positions_y[1] - trajectory.positions_y[0];
                double dt = trajectory.time_points[1] - trajectory.time_points[0];
                double speed = (dt > 0) ? std::hypot(dx, dy) / dt : 0.0;
                trajectory.velocities.push_back(speed);
              } else {
                trajectory.velocities.push_back(0.0);
              }
            } else {
              // 前の点との差分で速度を計算
              double dx = trajectory.positions_x[i] - trajectory.positions_x[i - 1];
              double dy = trajectory.positions_y[i] - trajectory.positions_y[i - 1];
              double dt = trajectory.time_points[i] - trajectory.time_points[i - 1];
              double speed = (dt > 0) ? std::hypot(dx, dy) / dt : 0.0;
              trajectory.velocities.push_back(speed);
            }
          }
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "JSONファイル読み込みエラー %s: %s",
      json_file_path.c_str(), e.what());
  }

  return trajectory;
}

auto SimpleBallPhysicsOptimizer::loadAllTrajectoryData(const std::string & directory_path)
  -> std::vector<TrajectoryData>
{
  std::vector<TrajectoryData> all_trajectories;

  if (!std::filesystem::exists(directory_path)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "ディレクトリが存在しません: %s",
      directory_path.c_str());
    return all_trajectories;
  }

  std::regex pattern(R"(kick_event_visualization_\d+_data\.json)");

  for (const auto & entry : std::filesystem::directory_iterator(directory_path)) {
    if (entry.is_regular_file()) {
      std::string filename = entry.path().filename().string();
      if (std::regex_match(filename, pattern)) {
        auto trajectory = loadTrajectoryDataFromJSON(entry.path().string());
        if (!trajectory.time_points.empty()) {
          all_trajectories.push_back(trajectory);
        }
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
    "JSONファイル読み込み完了: %zu個のファイルから軌道データを抽出", all_trajectories.size());

  return all_trajectories;
}

auto SimpleBallPhysicsOptimizer::optimizeGlobalDeceleration(
  const std::vector<TrajectoryData> & all_trajectories) -> double
{
  double best_deceleration = 0.0;
  double min_global_rmse = std::numeric_limits<double>::max();

  // 各キックデータのslope値範囲を調査
  double min_slope = std::numeric_limits<double>::max();
  double max_slope = std::numeric_limits<double>::lowest();

  for (const auto & trajectory : all_trajectories) {
    if (trajectory.time_points.size() < config_.min_data_points_per_trajectory) continue;

    // 各軌道の実際のslope値を計算
    auto [slope, intercept, r_squared] =
      performLinearRegression(trajectory.time_points, trajectory.velocities);

    if (r_squared > 0.3) {                      // 最低限の品質チェック
      min_slope = std::min(min_slope, -slope);  // slope は負なので、-slope が減速度
      max_slope = std::max(max_slope, -slope);
    }
  }

  // slope値の範囲が取得できない場合はデフォルト範囲を使用
  if (min_slope == std::numeric_limits<double>::max()) {
    min_slope = 0.3;
    max_slope = 1.2;
  }

  // 範囲を拡張（±0.1）してグリッド境界に合わせる（0.01刻み）
  double grid_step = 0.01;
  double extended_min = min_slope - 0.1;
  double extended_max = max_slope + 0.1;

  // グリッド境界に合わせて調整
  min_slope = std::max(0.1, std::floor(extended_min / grid_step) * grid_step);
  max_slope = std::min(2.0, std::ceil(extended_max / grid_step) * grid_step);

  RCLCPP_INFO(
    rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
    "減速度探索範囲: %.2f - %.2f m/s² (slope値から決定、0.01刻み)", min_slope, max_slope);

  // グリッドサーチによる最適化（0.01刻みで探索）
  for (double decel = min_slope; decel <= max_slope; decel += grid_step) {
    double total_rmse = 0.0;
    size_t valid_trajectories = 0;

    // 最初の減速度候補での実際のslope値を確認
    if (std::abs(decel - min_slope) < 0.005) {
      RCLCPP_INFO(
        rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
        "減速度候補=%.2f での実際のslope値を確認:", decel);
    }

    for (const auto & trajectory : all_trajectories) {
      if (trajectory.time_points.size() < config_.min_data_points_per_trajectory) continue;

      // 固定減速度を仮定した初速度推定（v(t) = v0 - decel * t）
      // 線形回帰で v0 を求める: velocities = v0 - decel * time_points
      std::vector<double> expected_velocities;
      for (size_t i = 0; i < trajectory.time_points.size(); ++i) {
        expected_velocities.push_back(-decel * trajectory.time_points[i]);  // -decel * t の部分
      }

      // velocities - expected_velocities = v0 (定数) を線形回帰で求める
      auto [slope, intercept, r_squared] =
        performLinearRegression(expected_velocities, trajectory.velocities);

      // slope は 1 に近く、intercept が推定初速度 v0
      double estimated_v0 = intercept;

      // 固定減速度モデルでの予測誤差を計算
      double rmse = 0.0;
      for (size_t i = 0; i < trajectory.time_points.size(); ++i) {
        double predicted_velocity = estimated_v0 - decel * trajectory.time_points[i];
        double error = trajectory.velocities[i] - predicted_velocity;
        rmse += error * error;
      }
      rmse = std::sqrt(rmse / trajectory.time_points.size());

      // 最初の減速度候補での詳細を表示
      if (std::abs(decel - min_slope) < 0.005 && valid_trajectories < 3) {
        RCLCPP_INFO(
          rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
          "軌道%zu: 推定v0=%.3f, RMSE=%.3f (減速度=%.2f)", trajectory.event_id, estimated_v0, rmse,
          decel);
      }

      // 品質フィルタリング
      if (estimated_v0 > 0.1 && rmse < 2.0) {  // 合理的な初速度と誤差
        total_rmse += rmse;
        valid_trajectories++;
      }
    }

    // 各減速度候補での有効軌道数を表示（0.05刻みで間引き表示）
    if (std::fmod(decel - min_slope, 0.05) < 0.005) {
      RCLCPP_INFO(
        rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
        "減速度%.2f: 有効軌道数=%zu, 平均RMSE=%.4f", decel, valid_trajectories,
        valid_trajectories > 0 ? total_rmse / valid_trajectories : 999.0);
    }

    if (valid_trajectories >= 3) {
      double avg_rmse = total_rmse / valid_trajectories;
      if (avg_rmse < min_global_rmse) {
        min_global_rmse = avg_rmse;
        best_deceleration = decel;
        RCLCPP_INFO(
          rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "新しいベスト減速度: %.2f (RMSE: %.4f)",
          decel, avg_rmse);
      }
    }
  }

  if (best_deceleration > 0.0) {
    RCLCPP_INFO(
      rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
      "グローバル減速度最適化成功: %.4f m/s² (RMSE: %.4f)", best_deceleration, min_global_rmse);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("SimpleBallPhysicsOptimizer"),
      "グローバル減速度最適化失敗: 有効な軌道が見つかりませんでした (探索範囲: %.2f-%.2f m/s²)",
      min_slope, max_slope);
  }

  return best_deceleration;
}

auto SimpleBallPhysicsOptimizer::estimateInitialVelocity(
  const TrajectoryData & trajectory, double /* deceleration */) -> KickPowerVelocityPair
{
  KickPowerVelocityPair result;
  result.event_id = trajectory.event_id;
  result.kick_power = trajectory.kick_power;
  result.is_chip_kick = trajectory.is_chip_kick;
  result.trajectory_duration = (trajectory.time_points.empty())
                                 ? 0.0
                                 : trajectory.time_points.back() - trajectory.time_points.front();

  if (trajectory.time_points.size() < config_.min_data_points_per_trajectory) {
    result.fitting_r_squared = 0.0;
    return result;
  }

  // 速度vs時間の線形回帰: v(t) = v0 - deceleration * t
  // ここで固定された減速度を使用して初速度を推定
  auto [slope, intercept, r_squared] =
    performLinearRegression(trajectory.time_points, trajectory.velocities);

  result.estimated_initial_velocity = intercept;  // 切片 = 初速度
  result.fitting_r_squared = r_squared;

  // 信頼区間の計算（簡易版）
  double velocity_std = 0.0;
  for (size_t i = 0; i < trajectory.time_points.size(); ++i) {
    double predicted_velocity = intercept + slope * trajectory.time_points[i];
    double error = trajectory.velocities[i] - predicted_velocity;
    velocity_std += error * error;
  }
  velocity_std = std::sqrt(velocity_std / trajectory.time_points.size());

  double confidence_margin = 1.96 * velocity_std;  // 95%信頼区間
  result.confidence_interval = {intercept - confidence_margin, intercept + confidence_margin};

  return result;
}

auto SimpleBallPhysicsOptimizer::performLinearRegression(
  const std::vector<double> & x_data, const std::vector<double> & y_data)
  -> std::tuple<double, double, double>
{
  if (x_data.size() != y_data.size() || x_data.size() < 2) {
    return {0.0, 0.0, 0.0};
  }

  size_t n = x_data.size();
  double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0, sum_y2 = 0.0;

  for (size_t i = 0; i < n; ++i) {
    sum_x += x_data[i];
    sum_y += y_data[i];
    sum_xy += x_data[i] * y_data[i];
    sum_x2 += x_data[i] * x_data[i];
    sum_y2 += y_data[i] * y_data[i];
  }

  double mean_x = sum_x / n;
  double mean_y = sum_y / n;

  // 線形回帰の計算: y = slope * x + intercept
  double denominator = sum_x2 - n * mean_x * mean_x;
  double slope = 0.0, intercept = 0.0, r_squared = 0.0;

  if (std::abs(denominator) > 1e-10) {
    slope = (sum_xy - n * mean_x * mean_y) / denominator;
    intercept = mean_y - slope * mean_x;

    // R²の計算
    double ss_tot = sum_y2 - n * mean_y * mean_y;
    double ss_res = 0.0;
    for (size_t i = 0; i < n; ++i) {
      double predicted = slope * x_data[i] + intercept;
      ss_res += (y_data[i] - predicted) * (y_data[i] - predicted);
    }

    if (ss_tot > 1e-10) {
      r_squared = 1.0 - (ss_res / ss_tot);
    }
  }

  return {slope, intercept, r_squared};
}

auto SimpleBallPhysicsOptimizer::filterQualityData(const std::vector<TrajectoryData> & trajectories)
  -> std::vector<TrajectoryData>
{
  std::vector<TrajectoryData> filtered;

  for (const auto & trajectory : trajectories) {
    // 基本的な品質チェック
    if (trajectory.time_points.size() < config_.min_data_points_per_trajectory) continue;
    if (trajectory.kick_power <= 0.0 || trajectory.kick_power > 1.0) continue;

    // 軌道継続時間チェック
    double duration = (trajectory.time_points.empty())
                        ? 0.0
                        : trajectory.time_points.back() - trajectory.time_points.front();
    if (duration < config_.min_trajectory_duration) continue;

    // 速度データの妥当性チェック
    bool valid_velocities = true;
    double max_velocity = 0.0;
    for (double velocity : trajectory.velocities) {
      if (velocity < 0.0 || velocity > 20.0) {  // 物理的上限チェック
        valid_velocities = false;
        break;
      }
      max_velocity = std::max(max_velocity, velocity);
    }

    if (!valid_velocities || max_velocity < 0.5) continue;  // 最小速度チェック

    // ストレートキックのみをフィルタ（チップキックは複雑すぎるため除外）
    if (!trajectory.is_chip_kick) {
      filtered.push_back(trajectory);
    }
  }

  return filtered;
}

auto SimpleBallPhysicsOptimizer::exportKickPowerAnalysis(
  const std::string & output_path, const OptimizationResult & result) -> bool
{
  try {
    nlohmann::json output_json;

    // グローバルキャリブレーション結果
    output_json["global_calibration"]["deceleration_parameter"] = result.global_deceleration;
    output_json["global_calibration"]["global_rmse"] = result.global_rmse;
    output_json["global_calibration"]["global_r_squared"] = result.global_r_squared;
    output_json["global_calibration"]["trajectories_analyzed"] = result.trajectories_analyzed;
    output_json["global_calibration"]["valid_trajectories"] = result.trajectories_used;

    // キックパワーデータ
    auto & kick_power_data = output_json["kick_power_data"];
    for (const auto & kick : result.kick_data) {
      nlohmann::json kick_json;
      kick_json["event_id"] = kick.event_id;
      kick_json["kick_power"] = kick.kick_power;
      kick_json["estimated_initial_velocity"] = kick.estimated_initial_velocity;
      kick_json["is_chip_kick"] = kick.is_chip_kick;
      kick_json["fitting_r_squared"] = kick.fitting_r_squared;
      kick_json["trajectory_duration"] = kick.trajectory_duration;
      kick_json["confidence_interval"] = {
        kick.confidence_interval.first, kick.confidence_interval.second};
      kick_power_data.push_back(kick_json);
    }

    // パワー別統計サマリー（0.0-1.0を0.1刻み）
    std::vector<double> target_powers = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    auto & power_summary = output_json["power_velocity_summary"];

    for (double target_power : target_powers) {
      std::vector<double> velocities_for_power;
      for (const auto & kick : result.kick_data) {
        if (!kick.is_chip_kick && std::abs(kick.kick_power - target_power) < 0.05) {
          velocities_for_power.push_back(kick.estimated_initial_velocity);
        }
      }

      nlohmann::json power_entry;
      power_entry["power"] = target_power;
      power_entry["sample_count"] = velocities_for_power.size();

      if (!velocities_for_power.empty()) {
        double mean_velocity =
          std::accumulate(velocities_for_power.begin(), velocities_for_power.end(), 0.0) /
          velocities_for_power.size();
        power_entry["mean_velocity"] = mean_velocity;

        // 標準偏差計算
        double variance = 0.0;
        for (double vel : velocities_for_power) {
          variance += (vel - mean_velocity) * (vel - mean_velocity);
        }
        variance /= velocities_for_power.size();
        power_entry["std_deviation"] = std::sqrt(variance);
      } else {
        power_entry["mean_velocity"] = 0.0;
        power_entry["std_deviation"] = 0.0;
      }

      power_summary.push_back(power_entry);
    }

    // ファイル出力
    std::ofstream output_file(output_path);
    if (!output_file.is_open()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "出力ファイルを開けませんでした: %s",
        output_path.c_str());
      return false;
    }

    output_file << output_json.dump(2);  // インデント付きで出力
    output_file.close();

    RCLCPP_INFO(
      rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "キックパワー分析結果を出力しました: %s",
      output_path.c_str());

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("SimpleBallPhysicsOptimizer"), "JSON出力エラー: %s", e.what());
    return false;
  }
}

}  // namespace crane
