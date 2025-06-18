// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__BALL_PHYSICS_MODEL_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__BALL_PHYSICS_MODEL_HPP_

#include <Eigen/Dense>
#include <crane_basics/ball_info.hpp>
#include <memory>

namespace crane
{
class BallPhysicsModel
{
public:
  struct Config
  {
    double deceleration = 0.5;        // 転がり時の減速度 (m/s²)
    double gravity = -9.81;           // 重力加速度 (m/s²)
    double air_resistance = 0.0;     // 空気抵抗係数
    double height_threshold = 0.05;  // 飛行判定の高度閾値 (m)
    double speed_threshold = 0.1;    // 移動判定の速度閾値 (m/s)
    double stop_threshold = 0.05;    // 停止判定の速度閾値 (m/s)
  };

  BallPhysicsModel();
  explicit BallPhysicsModel(const Config & config);

  ~BallPhysicsModel() = default;

  // EKF用の物理計算
  [[nodiscard]] auto getStateTransitionMatrix(Ball::State state, double dt) const -> Eigen::Matrix<double, 6, 6>;

  [[nodiscard]] auto getControlInput(Ball::State state, double dt) const -> Eigen::Matrix<double, 6, 1>;

  // 状態推定
  [[nodiscard]] auto estimateStateFromMeasurement(const Eigen::Vector3d & position, const Eigen::Vector3d & velocity) const -> Ball::State;

  [[nodiscard]] auto checkStateTransition(Ball::State current_state, const Eigen::Vector3d & position, const Eigen::Vector3d & velocity) const -> Ball::State;

  // 予測計算（Ballクラス用）
  [[nodiscard]] auto predictPosition(const Ball & ball, double time_ahead) const -> Point;

  [[nodiscard]] auto predictVelocity(const Ball & ball, double time_ahead) const -> Point;

  [[nodiscard]] auto getStopTime(const Ball & ball) const -> double;

  [[nodiscard]] auto getMaxDistance(const Ball & ball) const -> double;

  // 設定アクセサ
  [[nodiscard]] auto getConfig() const -> const Config & { return config_; }

  auto setConfig(const Config & config) -> void { config_ = config; }

  // 物理定数アクセサ
  [[nodiscard]] auto getDeceleration() const -> double { return config_.deceleration; }

  [[nodiscard]] auto getGravity() const -> double { return config_.gravity; }

  [[nodiscard]] auto getAirResistance() const -> double { return config_.air_resistance; }

private:
  Config config_;

  // ヘルパー関数
  [[nodiscard]] auto getRollingStopTime(const Point & velocity) const -> double;

  [[nodiscard]] auto getRollingMaxDistance(const Point & velocity) const -> double;

  [[nodiscard]] auto getRollingPredictedPosition(const Point & position, const Point & velocity, double time_ahead) const -> Point;

  [[nodiscard]] auto getRollingPredictedVelocity(const Point & velocity, double time_ahead) const -> Point;
};

// シングルトンファクトリー（設定の一元管理用）
class BallPhysicsModelFactory
{
public:
  static auto getInstance() -> std::shared_ptr<BallPhysicsModel>;

  static auto createWithConfig(const BallPhysicsModel::Config & config) -> std::shared_ptr<BallPhysicsModel>;

private:
  static std::shared_ptr<BallPhysicsModel> instance_;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__BALL_PHYSICS_MODEL_HPP_