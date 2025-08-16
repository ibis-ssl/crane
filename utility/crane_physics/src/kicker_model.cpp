// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_physics/kicker_model.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

#include "crane_physics/ball_info.hpp"
#include "crane_physics/ball_physics_model.hpp"

namespace crane
{

// ===== コンストラクタ =====

KickerModel::KickerModel() : config_(), ball_physics_model_(nullptr) {}

KickerModel::KickerModel(const Config & config) : config_(config), ball_physics_model_(nullptr)
{
  if (!validateConfig()) {
    throw std::runtime_error("KickerModel: 無効な設定が指定されました");
  }
}

KickerModel::KickerModel(const Config & config, std::shared_ptr<BallPhysicsModel> ball_physics)
: config_(config), ball_physics_model_(ball_physics)
{
  if (!validateConfig()) {
    throw std::runtime_error("KickerModel: 無効な設定が指定されました");
  }
}

// ===== YAML設定ファイル読み込み =====

auto KickerModel::loadConfigFromYAML(const std::string & yaml_file_path) -> Config
{
  Config config;

  try {
    if (!std::filesystem::exists(yaml_file_path)) {
      throw std::runtime_error("YAMLファイルが見つかりません: " + yaml_file_path);
    }

    YAML::Node root = YAML::LoadFile(yaml_file_path);

    if (!root["kicker_model"]) {
      throw std::runtime_error("'kicker_model'セクションがYAMLファイルに見つかりません");
    }

    const YAML::Node & kicker_model = root["kicker_model"];

    // ストレートキック設定の読み込み
    if (kicker_model["straight_kick_powers"]) {
      config.straight_kick_powers = kicker_model["straight_kick_powers"].as<std::vector<double>>();
    }
    if (kicker_model["straight_kick_speeds"]) {
      config.straight_kick_speeds = kicker_model["straight_kick_speeds"].as<std::vector<double>>();
    }

    // チップキック設定の読み込み
    if (kicker_model["chip_kick_powers"]) {
      config.chip_kick_powers = kicker_model["chip_kick_powers"].as<std::vector<double>>();
    }
    if (kicker_model["chip_kick_distances"]) {
      config.chip_kick_distances = kicker_model["chip_kick_distances"].as<std::vector<double>>();
    }

  } catch (const YAML::Exception & e) {
    throw std::runtime_error("YAML解析エラー: " + std::string(e.what()));
  } catch (const std::exception & e) {
    throw std::runtime_error("設定ファイル読み込みエラー: " + std::string(e.what()));
  }

  return config;
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
    throw std::runtime_error("無効なキック力: " + std::to_string(kick_power));
  }

  return getLinearInterpolation(
    kick_power, config_.straight_kick_powers, config_.straight_kick_speeds);
}

auto KickerModel::predictChipKickDistance(double kick_power) const -> double
{
  if (!isValidKickPower(kick_power)) {
    throw std::runtime_error("無効なキック力: " + std::to_string(kick_power));
  }

  return getLinearInterpolation(kick_power, config_.chip_kick_powers, config_.chip_kick_distances);
}

auto KickerModel::calculateStraightKickPower(double target_speed) const -> double
{
  if (target_speed < 0.0) {
    throw std::runtime_error("無効な目標速度: " + std::to_string(target_speed));
  }

  return getInverseLinearInterpolation(
    target_speed, config_.straight_kick_powers, config_.straight_kick_speeds);
}

auto KickerModel::calculateChipKickPower(double target_distance) const -> double
{
  if (target_distance < 0.0) {
    throw std::runtime_error("無効な目標距離: " + std::to_string(target_distance));
  }

  return getInverseLinearInterpolation(
    target_distance, config_.chip_kick_powers, config_.chip_kick_distances);
}

// ===== BallPhysicsModel連携による高度な計算 =====

auto KickerModel::calculateKickPowerForStopDistance(double target_stop_distance) const -> double
{
  if (!ball_physics_model_) {
    throw std::runtime_error("BallPhysicsModelが設定されていません");
  }

  if (target_stop_distance < 0.0) {
    throw std::runtime_error("無効な目標停止距離: " + std::to_string(target_stop_distance));
  }

  // 二分探索でキック力を求める
  double power_min = 0.0;
  double power_max = 1.0;
  constexpr double tolerance = 0.01;  // 1cm の精度
  constexpr int max_iterations = 100;

  for (int i = 0; i < max_iterations; ++i) {
    double power_mid = (power_min + power_max) / 2.0;
    double predicted_distance = predictStopDistance(power_mid);

    if (std::abs(predicted_distance - target_stop_distance) < tolerance) {
      return power_mid;
    }

    if (predicted_distance < target_stop_distance) {
      power_min = power_mid;
    } else {
      power_max = power_mid;
    }
  }

  // 収束しない場合は最良の近似値を返す
  return (power_min + power_max) / 2.0;
}

auto KickerModel::predictStopDistance(double kick_power) const -> double
{
  if (!ball_physics_model_) {
    throw std::runtime_error("BallPhysicsModelが設定されていません");
  }

  if (!isValidKickPower(kick_power)) {
    throw std::runtime_error("無効なキック力: " + std::to_string(kick_power));
  }

  // キック力から初速度を予測
  double initial_speed = predictStraightKickSpeed(kick_power);

  // BallPhysicsModelを使って停止距離を計算
  Point initial_velocity(initial_speed, 0.0);  // X方向にキック
  return ball_physics_model_->getMaxDistance(
    Point(0.0, 0.0), initial_velocity, Ball::State::ROLLING, 0.0, 0.0);
}

auto KickerModel::predictChipKickTotalDistance(double kick_power) const -> double
{
  if (!ball_physics_model_) {
    throw std::runtime_error("BallPhysicsModelが設定されていません");
  }

  if (!isValidKickPower(kick_power)) {
    throw std::runtime_error("無効なキック力: " + std::to_string(kick_power));
  }

  // チップキックの飛行距離を予測
  double flight_distance = predictChipKickDistance(kick_power);

  // 着地後の転がり距離を計算するために、着地時の速度を推定
  // 簡単化のため、初速度の一定割合が着地時に残ると仮定
  double initial_speed = predictStraightKickSpeed(kick_power);
  double landing_speed = initial_speed * 0.3;  // 経験的な値（30%が残る）

  Point landing_velocity(landing_speed, 0.0);
  double rolling_distance = ball_physics_model_->getMaxDistance(
    Point(0.0, 0.0), landing_velocity, Ball::State::ROLLING, 0.0, 0.0);

  return flight_distance + rolling_distance;
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

auto KickerModel::getConfig() const -> const Config & { return config_; }

auto KickerModel::setConfig(const Config & config) -> void
{
  if (!validateArrays(config.straight_kick_powers, config.straight_kick_speeds, "straight_kick")) {
    throw std::runtime_error("無効なストレートキック設定");
  }

  if (!validateArrays(config.chip_kick_powers, config.chip_kick_distances, "chip_kick")) {
    throw std::runtime_error("無効なチップキック設定");
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
  if (x_array.size() != y_array.size()) {
    std::stringstream what;
    what << "x_arrayとy_arrayのサイズは等しい必要があります: ";
    what << "size(x_array): " << static_cast<int>(x_array.size());
    what << ", size(y_array): " << static_cast<int>(y_array.size());
    throw std::runtime_error(what.str());
  }

  if (x_array.empty()) {
    throw std::runtime_error("x_arrayが空です");
  }

  if (x_array.size() == 1) {
    return y_array[0];
  }

  // 範囲外の値は境界値でクランプ
  if (x <= x_array[0]) {
    return y_array[0];
  }
  if (x >= x_array.back()) {
    return y_array.back();
  }

  // 線形補間の実行
  for (size_t i = 1; i < x_array.size(); ++i) {
    if (x <= x_array[i]) {
      double x_diff = x_array[i] - x_array[i - 1];
      double y_diff = y_array[i] - y_array[i - 1];
      return y_array[i - 1] + (y_diff / x_diff) * (x - x_array[i - 1]);
    }
  }

  // ここに到達することはないはずだが、安全のため
  return y_array.back();
}

auto KickerModel::getInverseLinearInterpolation(
  double y, const std::vector<double> & x_array, const std::vector<double> & y_array) const
  -> double
{
  if (x_array.size() != y_array.size()) {
    throw std::runtime_error("配列サイズが一致しません");
  }

  if (x_array.empty()) {
    throw std::runtime_error("配列が空です");
  }

  if (x_array.size() == 1) {
    return x_array[0];
  }

  // Y配列の最小値と最大値を確認
  auto [min_y, max_y] = std::minmax_element(y_array.begin(), y_array.end());

  // 範囲外の値は境界値でクランプ
  if (y <= *min_y) {
    return x_array[std::distance(y_array.begin(), min_y)];
  }
  if (y >= *max_y) {
    return x_array[std::distance(y_array.begin(), max_y)];
  }

  // 逆線形補間の実行
  for (size_t i = 1; i < y_array.size(); ++i) {
    double y_prev = y_array[i - 1];
    double y_curr = y_array[i];

    // Y値が単調でない場合への対応
    if ((y_prev <= y && y <= y_curr) || (y_curr <= y && y <= y_prev)) {
      double y_diff = y_curr - y_prev;
      if (std::abs(y_diff) < 1e-10) {
        // Y値がほぼ同じ場合は中点を返す
        return (x_array[i - 1] + x_array[i]) / 2.0;
      }

      double x_diff = x_array[i] - x_array[i - 1];
      return x_array[i - 1] + (x_diff / y_diff) * (y - y_prev);
    }
  }

  // 補間点が見つからない場合は最も近いX値を返す
  size_t closest_idx = 0;
  double min_diff = std::abs(y - y_array[0]);
  for (size_t i = 1; i < y_array.size(); ++i) {
    double diff = std::abs(y - y_array[i]);
    if (diff < min_diff) {
      min_diff = diff;
      closest_idx = i;
    }
  }

  return x_array[closest_idx];
}

auto KickerModel::validateArrays(
  const std::vector<double> & x_array, const std::vector<double> & y_array,
  const std::string & array_name) const -> bool
{
  if (x_array.size() != y_array.size()) {
    return false;
  }

  if (x_array.empty()) {
    return false;
  }

  // X配列が単調増加であることを確認
  for (size_t i = 1; i < x_array.size(); ++i) {
    if (x_array[i] <= x_array[i - 1]) {
      return false;
    }
  }

  // Y配列が非負であることを確認
  for (double y : y_array) {
    if (y < 0.0) {
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

auto createKickerModelFromYAML(const std::string & yaml_file_path) -> std::shared_ptr<KickerModel>
{
  auto config = KickerModel::loadConfigFromYAML(yaml_file_path);
  return std::make_shared<KickerModel>(config);
}

auto createIntegratedKickerModel(
  const std::string & yaml_file_path, std::shared_ptr<BallPhysicsModel> ball_physics)
  -> std::shared_ptr<KickerModel>
{
  auto config = KickerModel::loadConfigFromYAML(yaml_file_path);
  return std::make_shared<KickerModel>(config, ball_physics);
}

}  // namespace crane
