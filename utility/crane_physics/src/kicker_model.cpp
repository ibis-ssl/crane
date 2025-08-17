// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_physics/kicker_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

// TODO: PR5でBallPhysicsModelが実装されたらincludeを有効化
// #include "crane_physics/ball_physics_model.hpp"

namespace crane
{

// ===== コンストラクタ・デストラクタ =====

KickerModel::KickerModel() : config_(), ball_physics_model_(nullptr) {}

KickerModel::KickerModel(const Config & config) : config_(config), ball_physics_model_(nullptr)
{
  if (!validateConfig()) {
    throw std::runtime_error("Invalid KickerModel configuration");
  }
}

KickerModel::KickerModel(const Config & config, std::shared_ptr<BallPhysicsModel> ball_physics)
: config_(config), ball_physics_model_(ball_physics)
{
  if (!validateConfig()) {
    throw std::runtime_error("Invalid KickerModel configuration");
  }
}

// ===== YAML設定ファイル読み込み =====

auto KickerModel::loadConfigFromYAML(const std::string & yaml_file_path) -> Config
{
  // TODO: YAML-cppライブラリを使用したYAMLファイル読み込み実装
  // 現在は基本的な実装のみ提供

  std::ifstream file(yaml_file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open YAML file: " + yaml_file_path);
  }

  Config config;
  // 簡易的なYAMLパース実装
  // 実際の実装では yaml-cpp または rclcpp::Parameter を使用

  std::string line;
  while (std::getline(file, line)) {
    // 基本的なパースロジック（実際の実装では改善が必要）
    if (line.find("straight_kick_powers:") != std::string::npos) {
      // 配列パース処理
      // デフォルト値を使用
      break;
    }
  }

  return config;  // デフォルト設定を返す
}

auto KickerModel::createWithYAMLConfig(const std::string & yaml_file_path) -> KickerModel
{
  auto config = loadConfigFromYAML(yaml_file_path);
  return KickerModel(config);
}

// ===== キック力 ↔ ボール初速度変換 =====

auto KickerModel::predictStraightKickSpeed(double kick_power) const -> double
{
  if (!isValidKickPower(kick_power)) {
    throw std::runtime_error("Invalid kick power: " + std::to_string(kick_power));
  }

  return getLinearInterpolation(
    kick_power, config_.straight_kick_powers, config_.straight_kick_speeds);
}

auto KickerModel::predictChipKickDistance(double kick_power) const -> double
{
  if (!isValidKickPower(kick_power)) {
    throw std::runtime_error("Invalid kick power: " + std::to_string(kick_power));
  }

  return getLinearInterpolation(
    kick_power, config_.chip_kick_powers, config_.chip_kick_distances);
}

auto KickerModel::calculateStraightKickPower(double target_speed) const -> double
{
  if (target_speed < 0.0) {
    throw std::runtime_error("Target speed must be non-negative");
  }

  return getInverseLinearInterpolation(
    target_speed, config_.straight_kick_powers, config_.straight_kick_speeds);
}

auto KickerModel::calculateChipKickPower(double target_distance) const -> double
{
  if (target_distance < 0.0) {
    throw std::runtime_error("Target distance must be non-negative");
  }

  return getInverseLinearInterpolation(
    target_distance, config_.chip_kick_powers, config_.chip_kick_distances);
}

// ===== BallPhysicsModel連携による高度な計算 =====

auto KickerModel::calculateKickPowerForStopDistance(double target_stop_distance) const -> double
{
  if (!ball_physics_model_) {
    throw std::runtime_error("BallPhysicsModel is not set");
  }

  if (target_stop_distance < 0.0) {
    throw std::runtime_error("Target stop distance must be non-negative");
  }

  // TODO: PR5でBallPhysicsModel実装後に詳細な物理計算を実装
  // 現在は簡易的な計算

  // 簡易的な物理モデル: v₀ = sqrt(2 * deceleration * distance)
  constexpr double estimated_deceleration = 0.7;  // m/s²
  double required_initial_speed = std::sqrt(2.0 * estimated_deceleration * target_stop_distance);

  return calculateStraightKickPower(required_initial_speed);
}

auto KickerModel::predictStopDistance(double kick_power) const -> double
{
  if (!ball_physics_model_) {
    throw std::runtime_error("BallPhysicsModel is not set");
  }

  if (!isValidKickPower(kick_power)) {
    throw std::runtime_error("Invalid kick power: " + std::to_string(kick_power));
  }

  // TODO: PR5でBallPhysicsModel実装後に詳細な物理計算を実装
  // 現在は簡易的な計算

  double initial_speed = predictStraightKickSpeed(kick_power);

  // 簡易的な物理モデル: distance = v₀² / (2 * deceleration)
  constexpr double estimated_deceleration = 0.7;  // m/s²
  return (initial_speed * initial_speed) / (2.0 * estimated_deceleration);
}

auto KickerModel::predictChipKickTotalDistance(double kick_power) const -> double
{
  if (!ball_physics_model_) {
    throw std::runtime_error("BallPhysicsModel is not set");
  }

  if (!isValidKickPower(kick_power)) {
    throw std::runtime_error("Invalid kick power: " + std::to_string(kick_power));
  }

  // チップキック飛行距離
  double flight_distance = predictChipKickDistance(kick_power);

  // TODO: PR5でBallPhysicsModel実装後に着地後の転がり距離を計算
  // 現在は簡易的な計算

  // 着地後の転がり距離（飛行距離の20%と仮定）
  double roll_distance = flight_distance * 0.2;

  return flight_distance + roll_distance;
}

// ===== BallPhysicsModel管理 =====

auto KickerModel::setBallPhysicsModel(std::shared_ptr<BallPhysicsModel> ball_physics) -> void
{
  ball_physics_model_ = ball_physics;
}

auto KickerModel::getBallPhysicsModel() const -> std::shared_ptr<BallPhysicsModel>
{
  return ball_physics_model_;
}

// ===== 設定アクセサ =====

auto KickerModel::getConfig() const -> const Config &
{
  return config_;
}

auto KickerModel::setConfig(const Config & config) -> void
{
  if (!validateArrays(
        config.straight_kick_powers, config.straight_kick_speeds, "straight_kick")) {
    throw std::runtime_error("Invalid straight kick configuration");
  }

  if (!validateArrays(config.chip_kick_powers, config.chip_kick_distances, "chip_kick")) {
    throw std::runtime_error("Invalid chip kick configuration");
  }

  config_ = config;
}

// ===== 検証関数 =====

auto KickerModel::validateConfig() const -> bool
{
  return validateArrays(
           config_.straight_kick_powers, config_.straight_kick_speeds, "straight_kick") &&
         validateArrays(config_.chip_kick_powers, config_.chip_kick_distances, "chip_kick");
}

auto KickerModel::isValidKickPower(double kick_power) -> bool
{
  return kick_power >= 0.0 && kick_power <= 1.0;
}

// ===== 内部ヘルパー関数 =====

auto KickerModel::getLinearInterpolation(
  double x, const std::vector<double> & x_array, const std::vector<double> & y_array) const
  -> double
{
  if (x_array.empty() || y_array.empty()) {
    throw std::runtime_error("Empty arrays for interpolation");
  }

  if (x_array.size() != y_array.size()) {
    throw std::runtime_error("Array size mismatch for interpolation");
  }

  // 範囲外の場合は端点の値を返す
  if (x <= x_array.front()) {
    return y_array.front();
  }
  if (x >= x_array.back()) {
    return y_array.back();
  }

  // 線形補間
  for (size_t i = 0; i < x_array.size() - 1; ++i) {
    if (x >= x_array[i] && x <= x_array[i + 1]) {
      double ratio = (x - x_array[i]) / (x_array[i + 1] - x_array[i]);
      return y_array[i] + ratio * (y_array[i + 1] - y_array[i]);
    }
  }

  // 到達しない点（理論上）
  return y_array.back();
}

auto KickerModel::getInverseLinearInterpolation(
  double y, const std::vector<double> & x_array, const std::vector<double> & y_array) const
  -> double
{
  if (x_array.empty() || y_array.empty()) {
    throw std::runtime_error("Empty arrays for inverse interpolation");
  }

  if (x_array.size() != y_array.size()) {
    throw std::runtime_error("Array size mismatch for inverse interpolation");
  }

  // 範囲外の場合は端点の値を返す
  if (y <= y_array.front()) {
    return x_array.front();
  }
  if (y >= y_array.back()) {
    return x_array.back();
  }

  // 逆線形補間
  for (size_t i = 0; i < y_array.size() - 1; ++i) {
    if (y >= y_array[i] && y <= y_array[i + 1]) {
      if (std::abs(y_array[i + 1] - y_array[i]) < 1e-9) {
        // 除算ゼロ回避
        return x_array[i];
      }
      double ratio = (y - y_array[i]) / (y_array[i + 1] - y_array[i]);
      return x_array[i] + ratio * (x_array[i + 1] - x_array[i]);
    }
  }

  // 到達しない点（理論上）
  return x_array.back();
}

auto KickerModel::validateArrays(
  const std::vector<double> & x_array, const std::vector<double> & y_array,
  const std::string & array_name) const -> bool
{
  if (x_array.empty() || y_array.empty()) {
    return false;
  }

  if (x_array.size() != y_array.size()) {
    return false;
  }

  // X配列が昇順であることを確認
  for (size_t i = 1; i < x_array.size(); ++i) {
    if (x_array[i] <= x_array[i - 1]) {
      return false;
    }
  }

  // Y配列が非負であることを確認
  for (double value : y_array) {
    if (value < 0.0) {
      return false;
    }
  }

  return true;
}

// ===== ファクトリー関数 =====

auto createDefaultKickerModel() -> std::shared_ptr<KickerModel>
{
  return std::make_shared<KickerModel>();
}

auto createKickerModelFromYAML(const std::string & yaml_file_path)
  -> std::shared_ptr<KickerModel>
{
  try {
    auto config = KickerModel::loadConfigFromYAML(yaml_file_path);
    return std::make_shared<KickerModel>(config);
  } catch (const std::exception & e) {
    // YAML読み込み失敗時はデフォルト設定を使用
    return createDefaultKickerModel();
  }
}

auto createIntegratedKickerModel(
  const std::string & yaml_file_path, std::shared_ptr<BallPhysicsModel> ball_physics)
  -> std::shared_ptr<KickerModel>
{
  auto kicker_model = createKickerModelFromYAML(yaml_file_path);
  kicker_model->setBallPhysicsModel(ball_physics);
  return kicker_model;
}

}  // namespace crane