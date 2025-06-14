#include "crane_world_model_publisher/ball_tracker.hpp"

namespace crane
{

BallTracker::BallTracker() : initialized_(false)
{
  // Initialize matrices (dimensions)
  x_k_ = Eigen::Vector4d::Zero();
  P_k_ = Eigen::Matrix4d::Identity();  // Initial uncertainty
  F_k_ = Eigen::Matrix4d::Identity();
  H_k_ = Eigen::MatrixXd::Zero(2, 4);
  Q_k_ = Eigen::Matrix4d::Identity();
  R_k_ = Eigen::Matrix2d::Identity();

  // Measurement matrix H_k_ (maps state to measurement [x, y])
  H_k_(0, 0) = 1.0;  // x = x
  H_k_(1, 1) = 1.0;  // y = y

  // Initial Process Noise Covariance Q_k_ (tune these values)
  // Assumes some uncertainty in constant velocity model
  double q_pos = 0.05;  // position variance
  double q_vel = 0.1;   // velocity variance
  Q_k_ << q_pos, 0, 0, 0, 0, q_pos, 0, 0, 0, 0, q_vel, 0, 0, 0, 0, q_vel;

  // Initial Measurement Noise Covariance R_k_ (tune these values)
  // Depends on sensor accuracy
  double r_pos = 0.1;  // measurement variance for position
  R_k_ << r_pos, 0, 0, r_pos;
}

void BallTracker::init(
  const rclcpp::Time & t_start, double initial_x, double initial_y, double initial_vx,
  double initial_vy)
{
  x_k_ << initial_x, initial_y, initial_vx, initial_vy;

  // Set initial covariance. Large if initial state is uncertain, smaller if more certain.
  P_k_ = Eigen::Matrix4d::Identity() * 0.1;  // Example: moderate initial uncertainty
  P_k_(2, 2) = 1.0;  // Higher uncertainty for initial velocities if not measured
  P_k_(3, 3) = 1.0;

  last_update_time_ = t_start;
  initialized_ = true;
  RCLCPP_INFO(
    rclcpp::get_logger("BallTracker"),
    "Kalman Filter initialized at t=%.4f with x=%.2f, y=%.2f, vx=%.2f, vy=%.2f", t_start.seconds(),
    initial_x, initial_y, initial_vx, initial_vy);
}

void BallTracker::predict(const rclcpp::Time & t_now)
{
  if (!initialized_) {
    RCLCPP_WARN(rclcpp::get_logger("BallTracker"), "Predict called before initialization.");
    return;
  }

  double dt = (t_now - last_update_time_).seconds();
  if (dt < 0) {
    RCLCPP_WARN(rclcpp::get_logger("BallTracker"), "Negative dt (%.4f) in predict. Skipping.", dt);
    dt = 0;  // Or handle as an error
  }

  // Update State Transition Matrix F_k_ for current dt
  F_k_ = Eigen::Matrix4d::Identity();
  F_k_(0, 2) = dt;  // x = x_prev + vx_prev * dt
  F_k_(1, 3) = dt;  // y = y_prev + vy_prev * dt

  // Predict state: x_k = F_k * x_k_{k-1}
  x_k_ = F_k_ * x_k_;
  // Predict covariance: P_k = F_k * P_{k-1} * F_k' + Q_k
  P_k_ = F_k_ * P_k_ * F_k_.transpose() + Q_k_;

  last_update_time_ = t_now;  // Update time for next prediction or update
}

void BallTracker::update(const Eigen::Vector2d & measurement, const rclcpp::Time & t_measurement)
{
  if (!initialized_) {
    RCLCPP_WARN(
      rclcpp::get_logger("BallTracker"),
      "Update called before initialization. Initializing with this measurement.");
    // If not initialized, use this measurement to initialize (optional, could also require explicit init)
    // init(t_measurement, measurement(0), measurement(1));
    // For now, let's assume init must be called first.
    return;
  }

  // If t_measurement is older than last_update_time_, it could be an old packet.
  // A more robust system might handle out-of-order measurements.
  // For simplicity, we can predict up to t_measurement if it's newer than last_update_time_
  // but not if predict() was just called with a t_now equal to t_measurement.
  // This check ensures we don't double-predict or use stale data.
  if (
    (t_measurement - last_update_time_).seconds() >
    1e-9) {  // if t_measurement is significantly newer
    //RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "Update time is newer, predicting up to measurement time.");
    predict(t_measurement);  // Predict up to the measurement time
  } else if (
    (t_measurement - last_update_time_).seconds() <
    -1e-9) {  // if t_measurement is significantly older
    RCLCPP_WARN(
      rclcpp::get_logger("BallTracker"), "Received old measurement (%.4fs older). Skipping update.",
      (last_update_time_ - t_measurement).seconds());
    return;
  }

  // Measurement residual (innovation): y_k = z_k - H_k * x_k_predicted
  Eigen::Vector2d y_k = measurement - H_k_ * x_k_;

  // Residual (innovation) covariance: S_k = H_k * P_k_predicted * H_k' + R_k
  Eigen::Matrix2d S_k = H_k_ * P_k_ * H_k_.transpose() + R_k_;

  // Optimal Kalman gain: K_k = P_k_predicted * H_k' * S_k^-1
  Eigen::MatrixXd K_k = P_k_ * H_k_.transpose() * S_k.inverse();

  // Update state estimate: x_k = x_k_predicted + K_k * y_k
  x_k_ = x_k_ + K_k * y_k;

  // Update state covariance: P_k = (I - K_k * H_k) * P_k_predicted
  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  P_k_ = (I - K_k * H_k_) * P_k_;

  // last_update_time_ was already updated in predict if called, or should be t_measurement if predict wasn't called.
  // Ensure last_update_time_ reflects the time of this data.
  last_update_time_ = t_measurement;
}

}  // namespace crane
