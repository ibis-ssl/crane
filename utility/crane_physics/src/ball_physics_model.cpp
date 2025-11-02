// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_physics/ball_physics_model.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace crane
{
BallPhysicsModel::BallPhysicsModel() : config_({}) {}

BallPhysicsModel::BallPhysicsModel(const Config & config) : config_(config) {}

auto BallPhysicsModel::getStateTransitionMatrix(Ball::State state, double dt) const
  -> Eigen::Matrix<double, 6, 6>
{
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();

  // 位置更新 (x = x + v*dt)
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  switch (state) {
    case Ball::State::STOPPED:
      // 停止状態では速度は0のまま（デフォルトの単位行列）
      break;

    case Ball::State::ROLLING:
      // 転がっているときは減速
      {
        double deceleration_factor = std::exp(-config_.deceleration * dt);
        F(3, 3) = deceleration_factor;  // vx減速
        F(4, 4) = deceleration_factor;  // vy減速
        F(5, 5) = 0.0;                  // vzは0
      }
      break;

    case Ball::State::FLYING:
      // 飛んでいるときはXY速度は一定、Z速度は重力の影響
      F(5, 5) = 1.0;  // Z速度は連続（重力は制御入力で処理）
      break;
  }

  return F;
}

auto BallPhysicsModel::getControlInput(Ball::State state, double dt) const
  -> Eigen::Matrix<double, 6, 1>
{
  Eigen::Matrix<double, 6, 1> control_input = Eigen::Matrix<double, 6, 1>::Zero();

  switch (state) {
    case Ball::State::STOPPED:
    case Ball::State::ROLLING:
      // 停止・転がり状態では制御入力なし（地面に接触）
      break;

    case Ball::State::FLYING:
      // 飛行状態では重力の影響
      {
        double gravity_pos_effect = 0.5 * config_.gravity * dt * dt;
        double gravity_vel_effect = config_.gravity * dt;

        control_input(2) = gravity_pos_effect;  // Z位置への重力効果
        control_input(5) = gravity_vel_effect;  // Z速度への重力効果
      }
      break;
  }

  return control_input;
}

auto BallPhysicsModel::estimateStateFromMeasurement(
  const Vector3 & position, const Vector3 & velocity) const -> Ball::State
{
  double height = position(2);
  double speed = velocity.head<2>().norm();

  if (height > config_.height_threshold) {
    return Ball::State::FLYING;
  } else if (speed > config_.speed_threshold) {
    return Ball::State::ROLLING;
  } else {
    return Ball::State::STOPPED;
  }
}

auto BallPhysicsModel::checkStateTransition(
  Ball::State current_state, const Vector3 & position, const Vector3 & velocity) const
  -> Ball::State
{
  double height = position(2);
  double speed = velocity.head<2>().norm();

  switch (current_state) {
    case Ball::State::FLYING:
      if (height <= 0.0) {
        return Ball::State::ROLLING;
      }
      break;

    case Ball::State::ROLLING:
      if (speed < config_.stop_threshold) {
        return Ball::State::STOPPED;
      }
      break;

    case Ball::State::STOPPED:
      if (speed > config_.speed_threshold) {
        return Ball::State::ROLLING;
      }
      break;
  }

  return current_state;  // 状態変化なし
}

auto BallPhysicsModel::predictPosition(
  const Point & position, const Point & velocity, Ball::State state, double pos_z, double vel_z,
  double time_ahead) const -> Point
{
  switch (state) {
    case Ball::State::STOPPED:
      return position;

    case Ball::State::ROLLING:
      return getRollingPredictedPosition(position, velocity, time_ahead);

    case Ball::State::FLYING: {
      // 独立した放物運動計算（Ballクラスに依存しない）
      Point3D initial_pos(position.x(), position.y(), pos_z);
      Point3D initial_vel(velocity.x(), velocity.y(), vel_z);

      // 着地時間計算
      double a = 0.5 * config_.gravity;
      double b = vel_z;
      double c = pos_z;

      double discriminant = b * b - 4 * a * c;
      double landing_time = 0.0;

      if (discriminant >= 0) {
        double sqrt_discriminant = std::sqrt(discriminant);
        double t1 = (-b + sqrt_discriminant) / (2 * a);
        double t2 = (-b - sqrt_discriminant) / (2 * a);

        if (t1 > 1e-6 && t2 > 1e-6) {
          landing_time = std::min(t1, t2);
        } else if (t1 > 1e-6) {
          landing_time = t1;
        } else if (t2 > 1e-6) {
          landing_time = t2;
        }
      }

      if (time_ahead <= landing_time) {
        // まだ空中
        Point predicted_pos;
        predicted_pos.x() = initial_pos.x() + initial_vel.x() * time_ahead;
        predicted_pos.y() = initial_pos.y() + initial_vel.y() * time_ahead;
        return predicted_pos;
      } else {
        // 着地後転がり
        Point landing_pos;
        landing_pos.x() = initial_pos.x() + initial_vel.x() * landing_time;
        landing_pos.y() = initial_pos.y() + initial_vel.y() * landing_time;

        Point landing_vel;
        landing_vel.x() = initial_vel.x();
        landing_vel.y() = initial_vel.y();

        double time_after_landing = time_ahead - landing_time;
        return getRollingPredictedPosition(landing_pos, landing_vel, time_after_landing);
      }
    }
  }
  return position;
}

auto BallPhysicsModel::predictVelocity(
  [[maybe_unused]] const Point & position, const Point & velocity, Ball::State state, double pos_z,
  double vel_z, double time_ahead) const -> Point
{
  switch (state) {
    case Ball::State::STOPPED:
      return {0, 0};

    case Ball::State::ROLLING:
      return getRollingPredictedVelocity(velocity, time_ahead);

    case Ball::State::FLYING: {
      // 独立した放物運動計算
      double a = 0.5 * config_.gravity;
      double b = vel_z;
      double c = pos_z;

      double discriminant = b * b - 4 * a * c;
      double landing_time = 0.0;

      if (discriminant >= 0) {
        double sqrt_discriminant = std::sqrt(discriminant);
        double t1 = (-b + sqrt_discriminant) / (2 * a);
        double t2 = (-b - sqrt_discriminant) / (2 * a);

        if (t1 > 1e-6 && t2 > 1e-6) {
          landing_time = std::min(t1, t2);
        } else if (t1 > 1e-6) {
          landing_time = t1;
        } else if (t2 > 1e-6) {
          landing_time = t2;
        }
      }

      if (time_ahead <= landing_time) {
        // まだ空中（XY速度は一定）
        return velocity;
      } else {
        // 着地後転がり
        double time_after_landing = time_ahead - landing_time;
        return getRollingPredictedVelocity(velocity, time_after_landing);
      }
    }
  }
  return {0, 0};
}

auto BallPhysicsModel::getStopTime(const Point & velocity, Ball::State state, double vel_z) const
  -> double
{
  switch (state) {
    case Ball::State::STOPPED:
      return 0.0;

    case Ball::State::ROLLING:
      return getRollingStopTime(velocity);

    case Ball::State::FLYING: {
      // 簡単な着地時間計算（pos_z=0と仮定）
      double landing_time = 0.0;
      if (vel_z != 0) {
        landing_time = std::max(0.0, -2.0 * vel_z / config_.gravity);
      }

      double rolling_stop_time = getRollingStopTime(velocity);
      return landing_time + rolling_stop_time;
    }
  }
  return 0.0;
}

auto BallPhysicsModel::getMaxDistance(
  const Point & position, const Point & velocity, Ball::State state, [[maybe_unused]] double pos_z,
  double vel_z) const -> double
{
  switch (state) {
    case Ball::State::STOPPED:
      return 0.0;

    case Ball::State::ROLLING:
      return getRollingMaxDistance(velocity);

    case Ball::State::FLYING: {
      // 着地位置計算
      double landing_time = 0.0;
      if (vel_z != 0) {
        landing_time = std::max(0.0, -2.0 * vel_z / config_.gravity);
      }

      Point landing_pos;
      landing_pos.x() = position.x() + velocity.x() * landing_time;
      landing_pos.y() = position.y() + velocity.y() * landing_time;

      double distance_to_landing = (landing_pos - position).norm();
      double rolling_distance = getRollingMaxDistance(velocity);

      return distance_to_landing + rolling_distance;
    }
  }
  return 0.0;
}

auto BallPhysicsModel::getRollingStopTime(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed == 0) return 0;
  return speed / config_.deceleration;
}

auto BallPhysicsModel::getRollingMaxDistance(const Point & velocity) const -> double
{
  double speed = velocity.norm();
  if (speed == 0) return 0;
  double stop_time = getRollingStopTime(velocity);
  return speed * stop_time - 0.5 * config_.deceleration * stop_time * stop_time;
}

auto BallPhysicsModel::getRollingPredictedPosition(
  const Point & position, const Point & velocity, double time_ahead) const -> Point
{
  double speed = velocity.norm();
  if (speed == 0) {
    return position;
  }

  Point direction = velocity.normalized();
  double stop_time = getRollingStopTime(velocity);

  if (time_ahead >= stop_time) {
    double max_distance = getRollingMaxDistance(velocity);
    return position + direction * max_distance;
  } else {
    double distance_traveled =
      speed * time_ahead - 0.5 * config_.deceleration * time_ahead * time_ahead;
    return position + direction * distance_traveled;
  }
}

auto BallPhysicsModel::getRollingPredictedVelocity(const Point & velocity, double time_ahead) const
  -> Point
{
  double speed = velocity.norm();
  if (speed == 0) {
    return {0, 0};
  }

  Point direction = velocity.normalized();
  double stop_time = getRollingStopTime(velocity);

  if (time_ahead >= stop_time) {
    return {0, 0};
  } else {
    double current_speed = speed - config_.deceleration * time_ahead;
    return direction * current_speed;
  }
}

auto BallPhysicsModel::loadConfigFromYAML(const std::string & yaml_file_path) -> Config
{
  Config config;

  try {
    // ファイルの存在確認
    if (!std::filesystem::exists(yaml_file_path)) {
      throw std::runtime_error("YAML file not found: " + yaml_file_path);
    }

    // YAMLファイル読み込み
    YAML::Node root = YAML::LoadFile(yaml_file_path);

    // ball_physics_modelセクションの存在確認
    if (!root["ball_physics_model"]) {
      throw std::runtime_error("'ball_physics_model' section not found in YAML file");
    }

    const YAML::Node & physics_model = root["ball_physics_model"];

    // パラメータの読み込み（デフォルト値を使用）
    config.deceleration = physics_model["deceleration"].as<double>(config.deceleration);
    config.gravity = physics_model["gravity"].as<double>(config.gravity);
    config.air_resistance = physics_model["air_resistance"].as<double>(config.air_resistance);
    config.height_threshold = physics_model["height_threshold"].as<double>(config.height_threshold);
    config.speed_threshold = physics_model["speed_threshold"].as<double>(config.speed_threshold);
    config.stop_threshold = physics_model["stop_threshold"].as<double>(config.stop_threshold);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("YAML parsing error: " + std::string(e.what()));
  } catch (const std::exception & e) {
    throw std::runtime_error("Error loading ball physics config: " + std::string(e.what()));
  }

  return config;
}

// 静的ファクトリーメソッドの実装
auto BallPhysicsModel::getDefaultConfig() -> Config
{
  Config config;
  config.deceleration = 0.7;       // 転がり時の減速度 (m/s²)
  config.gravity = -9.81;          // 重力加速度 (m/s²)
  config.air_resistance = 0.0;     // 空気抵抗係数
  config.height_threshold = 0.05;  // 飛行判定の高度閾値 (m)
  config.speed_threshold = 0.1;    // 移動判定の速度閾値 (m/s)
  config.stop_threshold = 0.05;    // 停止判定の速度閾値 (m/s)
  return config;
}

auto BallPhysicsModel::createDefault() -> BallPhysicsModel
{
  return BallPhysicsModel(getDefaultConfig());
}

// ファクトリーの実装
std::shared_ptr<BallPhysicsModel> BallPhysicsModelFactory::instance_ = nullptr;

auto BallPhysicsModelFactory::getInstance() -> std::shared_ptr<BallPhysicsModel>
{
  if (!instance_) {
    instance_ = std::make_shared<BallPhysicsModel>();
  }
  return instance_;
}

auto BallPhysicsModelFactory::createWithConfig(const BallPhysicsModel::Config & config)
  -> std::shared_ptr<BallPhysicsModel>
{
  instance_ = std::make_shared<BallPhysicsModel>(config);
  return instance_;
}

auto BallPhysicsModelFactory::createWithYAMLConfig(const std::string & yaml_file_path)
  -> std::shared_ptr<BallPhysicsModel>
{
  try {
    auto config = BallPhysicsModel::loadConfigFromYAML(yaml_file_path);
    instance_ = std::make_shared<BallPhysicsModel>(config);
    return instance_;
  } catch (const std::exception & e) {
    // YAML読み込み失敗時はデフォルト設定を使用
    instance_ = std::make_shared<BallPhysicsModel>();
    throw;  // 呼び出し側でエラーハンドリング可能
  }
}
}  // namespace crane
