// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__LATENCY_MEASUREMENT_SESSION_HPP_
#define CRANE_SESSIONS__LATENCY_MEASUREMENT_SESSION_HPP_

#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/session_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
// Sends sinusoidal yaw commands to assigned robots to excite theta variation.
// Latency estimation from the resulting signals is handled by crane_latency_estimator_node.
class LatencyMeasurementSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC explicit LatencyMeasurementSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    return [](const std::shared_ptr<RobotInfo> & robot) { return static_cast<double>(robot->id); };
  }

  int getDesiredRobotNumber(int max_robots) const override { return max_robots; }

  void onRobotsChanged() override {}

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

private:
  static constexpr double AMPLITUDE = 0.5;     // rad
  static constexpr double FREQUENCY = 0.7;     // Hz
  static constexpr double OMEGA_LIMIT = 20.0;  // rad/s

  struct RobotState
  {
    double base_theta{0.0};
    double phase{0.0};
  };

  std::unordered_map<uint8_t, RobotState> states_;
  std::shared_ptr<rclcpp::Clock> clock_;
  rclcpp::Time start_time_;
};

}  // namespace crane

#endif  // CRANE_SESSIONS__LATENCY_MEASUREMENT_SESSION_HPP_
