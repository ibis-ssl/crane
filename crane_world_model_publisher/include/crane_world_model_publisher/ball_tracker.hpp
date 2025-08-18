// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_physics/ball_info.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace crane
{
class BallTracker
{
public:
  explicit BallTracker(
    const Vector3 & initial_position, Ball::State initial_state = Ball::State::ROLLING,
    std::shared_ptr<BallPhysicsModel> physics_model = BallPhysicsModelFactory::getInstance(),
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>());

  ~BallTracker() = default;

  auto predict(double dt) -> void;

  auto update(const Vector3 & measurement, Ball::State observed_state = Ball::State::ROLLING)
    -> void;

  [[nodiscard]] auto getState() const -> crane_msgs::msg::BallInfo;

  [[nodiscard]] auto getBall() const -> Ball;

  [[nodiscard]] auto getPosition() const -> Vector3;

  [[nodiscard]] auto getVelocity() const -> Vector3;

  [[nodiscard]] auto getCovariance() const -> Eigen::Matrix<double, 6, 6>;

  [[nodiscard]] auto getMahalanobisDistance(const Vector3 & measurement) const -> double;

  [[nodiscard]] auto isValidMeasurement(const Vector3 & measurement, double threshold = 9.0) const
    -> bool;

  [[nodiscard]] auto getLastUpdateTime() const -> rclcpp::Time { return last_update_time_; }

  auto setLastUpdateTime(const rclcpp::Time & time) -> void { last_update_time_ = time; }

  [[nodiscard]] auto getTrackingConfidence() const -> double { return tracking_confidence_; }

  [[nodiscard]] auto getBallState() const -> Ball::State { return ball_state_; }

  auto resetTracker(const Vector3 & position, Ball::State state = Ball::State::ROLLING) -> void;

  [[nodiscard]] auto getPhysicsModel() const -> std::shared_ptr<BallPhysicsModel>
  {
    return physics_model_;
  }

private:
  Eigen::Matrix<double, 6, 1> state_;
  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Matrix<double, 6, 6> process_noise_;
  Eigen::Matrix<double, 3, 3> measurement_noise_;

  Ball::State ball_state_;
  rclcpp::Time last_update_time_;
  double tracking_confidence_;

  std::shared_ptr<BallPhysicsModel> physics_model_;
  std::shared_ptr<rclcpp::Clock> clock_;

  auto initializeMatrices() -> void;

  auto getMeasurementMatrix() const -> Eigen::Matrix<double, 3, 6>;

  auto updateStateTransition() -> void;
};

class BallTrackerManager
{
public:
  explicit BallTrackerManager(
    std::shared_ptr<rclcpp::Clock> clock,
    std::shared_ptr<BallPhysicsModel> physics_model = BallPhysicsModelFactory::getInstance());

  ~BallTrackerManager() = default;

  auto processVisionDetection(const Vector3 & ball_position, const rclcpp::Time & timestamp)
    -> crane_msgs::msg::BallInfo;

  auto processVisionDetectionWithState(
    const Vector3 & ball_position, Ball::State observed_state, const rclcpp::Time & timestamp)
    -> crane_msgs::msg::BallInfo;

  auto predict(double dt) -> void;

  [[nodiscard]] auto getBestTracker() const -> std::shared_ptr<BallTracker>;

  auto removeOldTrackers(double max_age_seconds = 1.0) -> void;

private:
  std::vector<std::shared_ptr<BallTracker>> trackers_;
  std::shared_ptr<BallPhysicsModel> physics_model_;
  std::shared_ptr<rclcpp::Clock> clock_;

  static constexpr double OUTLIER_THRESHOLD = 9.0;
  static constexpr double MIN_TRACKING_CONFIDENCE = 0.3;

  auto findBestMatchingTracker(const Vector3 & measurement) -> std::shared_ptr<BallTracker>;

  auto createNewTracker(const Vector3 & position) -> std::shared_ptr<BallTracker>;

  auto createNewTracker(const Vector3 & position, Ball::State state)
    -> std::shared_ptr<BallTracker>;

  auto updateTrackingConfidences() -> void;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_
