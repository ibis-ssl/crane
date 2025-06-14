#ifndef CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace crane
{

class BallTracker
{
public:
  BallTracker();

  void init(
    const rclcpp::Time & t_start, double initial_x, double initial_y, double initial_vx = 0.0,
    double initial_vy = 0.0);
  void predict(const rclcpp::Time & t_now);
  void update(const Eigen::Vector2d & measurement, const rclcpp::Time & t_measurement);

  Eigen::Vector4d getState() const { return x_k_; }  // x, y, vx, vy
  Eigen::Matrix4d getCovariance() const { return P_k_; }
  bool isInitialized() const { return initialized_; }

private:
  bool initialized_;
  rclcpp::Time last_update_time_;

  // Kalman Filter matrices
  Eigen::Vector4d x_k_;  // State estimate [x, y, vx, vy]'
  Eigen::Matrix4d P_k_;  // State covariance matrix
  Eigen::Matrix4d F_k_;  // State transition matrix
  Eigen::Matrix2d R_k_;  // Measurement noise covariance matrix
  Eigen::MatrixXd H_k_;  // Measurement matrix (4x2 -> 2x4)
  Eigen::Matrix4d Q_k_;  // Process noise covariance matrix
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__BALL_TRACKER_HPP_
