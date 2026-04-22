// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <crane_sessions/latency_measurement_session.hpp>
#include <string>

namespace crane
{

LatencyMeasurementSession::LatencyMeasurementSession(
  WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: SessionBase("latency_measurement", world_model)
{
  clock_ = node.get_clock();
  start_time_ = clock_->now();
}

std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
LatencyMeasurementSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  if (robots.empty()) {
    return {Status::RUNNING, robot_commands};
  }

  const double t_sec = (clock_->now() - start_time_).seconds();
  const size_t n = robots.size();

  for (size_t idx = 0; idx < n; ++idx) {
    const uint8_t robot_id = robots[idx].id;
    auto robot = world_model->getOurRobot(robot_id);

    if (states_.find(robot_id) == states_.end()) {
      RobotState state;
      state.base_theta = robot->pose.theta;
      state.phase = static_cast<double>(idx) * M_PI / static_cast<double>(n);
      states_.emplace(robot_id, std::move(state));
    }

    const auto & state = states_[robot_id];
    const double target_theta =
      state.base_theta + AMPLITUDE * std::sin(2.0 * M_PI * FREQUENCY * t_sec + state.phase);

    RobotCommandWrapper cmd("latency_measurement", robot_id, world_model);
    cmd.stopHere().setTargetTheta(target_theta).setOmegaLimit(OMEGA_LIMIT);

    visualizer->drawDebugLabel(robot->pose.pos, "id" + std::to_string(robot_id) + ":sin");

    robot_commands.emplace_back(cmd.getMsg());
  }

  return {Status::RUNNING, robot_commands};
}

}  // namespace crane
