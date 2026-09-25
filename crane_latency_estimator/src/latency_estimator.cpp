// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <crane_latency_estimator/latency_estimator.hpp>
#include <crane_utils/parameter.hpp>
#include <numeric>
#include <string>
#include <vector>

namespace crane
{

LatencyEstimator::LatencyEstimator(const rclcpp::NodeOptions & options)
: Node("latency_estimator", options)
{
  buffer_seconds_ = crane::get_or_declare_parameter(this, "buffer_seconds", 5.0);
  resample_dt_ms_ = crane::get_or_declare_parameter(this, "resample_dt_ms", 10.0);
  max_lag_ms_ = crane::get_or_declare_parameter(this, "max_lag_ms", 500.0);
  min_correlation_ = crane::get_or_declare_parameter(this, "min_correlation", 0.3);
  min_cmd_stddev_rad_ = crane::get_or_declare_parameter(this, "min_cmd_stddev_rad", 0.05);
  estimation_interval_sec_ = crane::get_or_declare_parameter(this, "estimation_interval_sec", 1.0);
  ema_alpha_ = crane::get_or_declare_parameter(this, "ema_alpha", 0.3);

  sub_commands_ = create_subscription<crane_msgs::msg::RobotCommands>(
    "/robot_commands", 10,
    [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) { onRobotCommands(msg); });

  sub_world_model_ = create_subscription<crane_msgs::msg::WorldModel>(
    "/world_model", 10,
    [this](const crane_msgs::msg::WorldModel::SharedPtr msg) { onWorldModel(msg); });

  sub_feedback_ = create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 10,
    [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) { onRobotFeedback(msg); });

  pub_estimation_ = create_publisher<crane_msgs::msg::LatencyEstimationArray>(
    "/latency_estimation", rclcpp::QoS(1));

  const auto interval_ms =
    std::chrono::milliseconds(static_cast<int>(estimation_interval_sec_ * 1000.0));
  estimation_timer_ = create_wall_timer(interval_ms, [this]() { onEstimationTimer(); });
}

double LatencyEstimator::interpolate(const std::deque<std::pair<double, double>> & data, double t)
{
  if (data.empty()) return 0.0;
  if (t <= data.front().first) return data.front().second;
  if (t >= data.back().first) return data.back().second;
  auto it = std::lower_bound(
    data.begin(), data.end(), t,
    [](const std::pair<double, double> & a, double v) { return a.first < v; });
  auto it_prev = std::prev(it);
  double alpha = (t - it_prev->first) / (it->first - it_prev->first);
  return it_prev->second + alpha * (it->second - it_prev->second);
}

LatencyEstimator::LagEstimate LatencyEstimator::estimateLagMs(
  const std::deque<std::pair<double, double>> & cmd,
  const std::deque<std::pair<double, double>> & obs, double max_lag_ms, double resample_dt_ms,
  double min_correlation, double min_cmd_stddev)
{
  constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

  if (cmd.size() < 10 || obs.size() < 10) return {NaN, 0.0, NaN};

  double t_start = std::max(cmd.front().first, obs.front().first);
  double t_end = std::min(cmd.back().first, obs.back().first);
  const double dt_sec = resample_dt_ms / 1000.0;
  int N = static_cast<int>((t_end - t_start) / dt_sec);
  if (N < 20) return {NaN, 0.0, NaN};

  std::vector<double> c(N), o(N);
  for (int i = 0; i < N; ++i) {
    double t = t_start + i * dt_sec;
    c[i] = interpolate(cmd, t);
    o[i] = interpolate(obs, t);
  }

  double c_mean = std::accumulate(c.begin(), c.end(), 0.0) / N;
  double o_mean = std::accumulate(o.begin(), o.end(), 0.0) / N;
  for (int i = 0; i < N; ++i) {
    c[i] -= c_mean;
    o[i] -= o_mean;
  }

  double c_std = std::sqrt(std::inner_product(c.begin(), c.end(), c.begin(), 0.0) / N);
  double o_std = std::sqrt(std::inner_product(o.begin(), o.end(), o.begin(), 0.0) / N);
  if (c_std < min_cmd_stddev || o_std < 1e-6) return {NaN, 0.0, c_std};

  int L = static_cast<int>(max_lag_ms / resample_dt_ms);
  double best_corr = -2.0;
  int best_lag = 0;
  for (int lag = -L; lag <= L; ++lag) {
    double corr = 0.0;
    int count = 0;
    for (int i = 0; i < N; ++i) {
      int j = i + lag;
      if (j >= 0 && j < N) {
        corr += c[i] * o[j];
        count++;
      }
    }
    if (count > 0) {
      corr /= static_cast<double>(count) * c_std * o_std;
      if (corr > best_corr) {
        best_corr = corr;
        best_lag = lag;
      }
    }
  }

  if (best_corr < min_correlation) return {NaN, best_corr, c_std};
  return {best_lag * resample_dt_ms, best_corr, c_std};
}

void LatencyEstimator::trimBuffer(std::deque<std::pair<double, double>> & buf, double oldest_t)
{
  while (!buf.empty() && buf.front().first < oldest_t) {
    buf.pop_front();
  }
}

double LatencyEstimator::applyEma(double prev, double next) const
{
  if (std::isnan(prev)) return next;
  if (std::isnan(next)) return prev;
  return ema_alpha_ * next + (1.0 - ema_alpha_) * prev;
}

void LatencyEstimator::onRobotCommands(const crane_msgs::msg::RobotCommands::SharedPtr msg)
{
  const double t = rclcpp::Time(msg->header.stamp).seconds();
  const double cutoff = t - buffer_seconds_;
  for (const auto & cmd : msg->robot_commands) {
    auto & buf = buffers_[cmd.robot_id];
    buf.cmd.emplace_back(t, static_cast<double>(cmd.target_theta));
    trimBuffer(buf.cmd, cutoff);
  }
}

void LatencyEstimator::onWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
{
  const double t = rclcpp::Time(msg->header.stamp).seconds();
  const double cutoff = t - buffer_seconds_;
  for (const auto & info : msg->robot_info_ours) {
    auto & buf = buffers_[info.id];
    buf.obs_world.emplace_back(t, info.pose.theta);
    trimBuffer(buf.obs_world, cutoff);
  }
}

void LatencyEstimator::onRobotFeedback(const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg)
{
  for (const auto & fb : msg->feedback) {
    const double t = rclcpp::Time(fb.received_stamp).seconds();
    const double cutoff = t - buffer_seconds_;
    auto & buf = buffers_[fb.robot_id];
    buf.obs_fb.emplace_back(t, static_cast<double>(fb.yaw_angle));
    trimBuffer(buf.obs_fb, cutoff);
  }
}

void LatencyEstimator::onEstimationTimer()
{
  crane_msgs::msg::LatencyEstimationArray out;
  out.header.stamp = now();

  for (auto & [robot_id, buf] : buffers_) {
    if (buf.cmd.empty()) continue;

    // world_model と robot_feedback は観測バッファ・EMA・source 名だけが違う
    const auto estimate =
      [&](const std::deque<std::pair<double, double>> & obs, double & ema_ms, const char * source) {
        const auto result = estimateLagMs(
          buf.cmd, obs, max_lag_ms_, resample_dt_ms_, min_correlation_, min_cmd_stddev_rad_);
        ema_ms = applyEma(ema_ms, result.lag_ms);
        if (std::isnan(result.lag_ms)) return;

        crane_msgs::msg::LatencyEstimation est;
        est.robot_id = robot_id;
        est.source = source;
        est.latency_ms = static_cast<float>(ema_ms);
        est.correlation = static_cast<float>(result.correlation);
        est.samples_used = static_cast<uint32_t>(buf.cmd.size());
        est.cmd_stddev = static_cast<float>(result.cmd_stddev);
        out.estimations.push_back(est);
      };
    estimate(buf.obs_world, buf.ema_world_ms, "world_model");
    estimate(buf.obs_fb, buf.ema_fb_ms, "robot_feedback");
  }

  if (!out.estimations.empty()) {
    pub_estimation_->publish(out);
  }
}

}  // namespace crane
