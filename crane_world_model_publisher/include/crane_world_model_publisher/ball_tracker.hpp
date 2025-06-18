// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_

#include <Eigen/Dense>
#include <crane_basics/ball_info.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_world_model_publisher/ball_physics_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

namespace crane
{
class BallTracker
{
public:
  explicit BallTracker(
    const Eigen::Vector3d & initial_position, 
    Ball::State initial_state = Ball::State::ROLLING,
    std::shared_ptr<BallPhysicsModel> physics_model = BallPhysicsModelFactory::getInstance());

  ~BallTracker() = default;

  auto predict(double dt) -> void;

  auto update(const Eigen::Vector3d & measurement, Ball::State observed_state = Ball::State::ROLLING) -> void;

  [[nodiscard]] auto getState() const -> crane_msgs::msg::BallInfo;

  [[nodiscard]] auto getBall() const -> Ball;

  [[nodiscard]] auto getPosition() const -> Eigen::Vector3d;

  [[nodiscard]] auto getVelocity() const -> Eigen::Vector3d;

  [[nodiscard]] auto getCovariance() const -> Eigen::Matrix<double, 6, 6>;

  [[nodiscard]] auto getMahalanobisDistance(const Eigen::Vector3d & measurement) const -> double;

  [[nodiscard]] auto isValidMeasurement(const Eigen::Vector3d & measurement, double threshold = 9.0) const -> bool;

  [[nodiscard]] auto getLastUpdateTime() const -> rclcpp::Time { return last_update_time_; }

  auto setLastUpdateTime(const rclcpp::Time & time) -> void { last_update_time_ = time; }

  [[nodiscard]] auto getTrackingConfidence() const -> double { return tracking_confidence_; }

  [[nodiscard]] auto getBallState() const -> Ball::State { return ball_state_; }

  auto resetTracker(const Eigen::Vector3d & position, Ball::State state = Ball::State::ROLLING) -> void;

  [[nodiscard]] auto getPhysicsModel() const -> std::shared_ptr<BallPhysicsModel> { return physics_model_; }

private:
  Eigen::Matrix<double, 6, 1> state_;
  Eigen::Matrix<double, 6, 6> covariance_;
  Eigen::Matrix<double, 6, 6> process_noise_;
  Eigen::Matrix<double, 3, 3> measurement_noise_;

  Ball::State ball_state_;
  rclcpp::Time last_update_time_;
  double tracking_confidence_;
  
  std::shared_ptr<BallPhysicsModel> physics_model_;

  auto initializeMatrices() -> void;

  auto getMeasurementMatrix() const -> Eigen::Matrix<double, 3, 6>;

  auto updateStateTransition() -> void;
};

class BallTrackerManager
{
public:
  explicit BallTrackerManager(std::shared_ptr<BallPhysicsModel> physics_model = BallPhysicsModelFactory::getInstance());

  ~BallTrackerManager() = default;

  auto processVisionDetection(const Eigen::Vector3d & ball_position, const rclcpp::Time & timestamp) -> crane_msgs::msg::BallInfo;

  auto processVisionDetectionWithState(const Eigen::Vector3d & ball_position, Ball::State observed_state, const rclcpp::Time & timestamp) -> crane_msgs::msg::BallInfo;

  auto predict(double dt) -> void;

  [[nodiscard]] auto getBestTracker() const -> std::shared_ptr<BallTracker>;

  auto removeOldTrackers(double max_age_seconds = 1.0) -> void;

private:
  std::vector<std::shared_ptr<BallTracker>> trackers_;
  std::shared_ptr<BallPhysicsModel> physics_model_;
  
  static constexpr double OUTLIER_THRESHOLD = 9.0;
  static constexpr double MIN_TRACKING_CONFIDENCE = 0.3;

  auto findBestMatchingTracker(const Eigen::Vector3d & measurement) -> std::shared_ptr<BallTracker>;

  auto createNewTracker(const Eigen::Vector3d & position) -> std::shared_ptr<BallTracker>;

  auto createNewTracker(const Eigen::Vector3d & position, Ball::State state) -> std::shared_ptr<BallTracker>;

  auto updateTrackingConfidences() -> void;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_