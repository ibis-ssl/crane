// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LATENCY_ESTIMATOR__LATENCY_ESTIMATOR_HPP_
#define CRANE_LATENCY_ESTIMATOR__LATENCY_ESTIMATOR_HPP_

#include <crane_msgs/msg/latency_estimation_array.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <deque>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>

namespace crane
{

class LatencyEstimator : public rclcpp::Node
{
public:
  explicit LatencyEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct RobotBuffer
  {
    std::deque<std::pair<double, double>> cmd;        // (t_sec, target_theta)
    std::deque<std::pair<double, double>> obs_world;  // (t_sec, pose.theta from world_model)
    std::deque<std::pair<double, double>> obs_fb;     // (t_sec, yaw_angle from robot_feedback)
    double last_world_ms{std::numeric_limits<double>::quiet_NaN()};
    double last_world_corr{0.0};
    double last_fb_ms{std::numeric_limits<double>::quiet_NaN()};
    double last_fb_corr{0.0};
    double ema_world_ms{std::numeric_limits<double>::quiet_NaN()};
    double ema_fb_ms{std::numeric_limits<double>::quiet_NaN()};
  };

  static double interpolate(const std::deque<std::pair<double, double>> & data, double t);
  static std::pair<double, double> estimateLagMs(
    const std::deque<std::pair<double, double>> & cmd,
    const std::deque<std::pair<double, double>> & obs, double max_lag_ms, double resample_dt_ms,
    double min_correlation, double min_cmd_stddev);

  void onRobotCommands(const crane_msgs::msg::RobotCommands::SharedPtr msg);
  void onWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg);
  void onRobotFeedback(const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg);
  void onEstimationTimer();

  void trimBuffer(std::deque<std::pair<double, double>> & buf, double oldest_t);
  double applyEma(double prev, double next) const;

  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commands_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr sub_feedback_;
  rclcpp::Publisher<crane_msgs::msg::LatencyEstimationArray>::SharedPtr pub_estimation_;
  rclcpp::TimerBase::SharedPtr estimation_timer_;

  std::unordered_map<uint8_t, RobotBuffer> buffers_;

  double buffer_seconds_{5.0};
  double resample_dt_ms_{10.0};
  double max_lag_ms_{500.0};
  double min_correlation_{0.3};
  double min_cmd_stddev_rad_{0.05};
  double estimation_interval_sec_{1.0};
  double ema_alpha_{0.3};
  double log_interval_sec_{1.0};

  rclcpp::Time last_log_time_;
};

}  // namespace crane

#endif  // CRANE_LATENCY_ESTIMATOR__LATENCY_ESTIMATOR_HPP_
