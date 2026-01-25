// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__TEST_TACTIC_HPP_
#define CRANE_SESSIONS__TEST_TACTIC_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class TestSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC explicit TestSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

private:
  struct Waypoint
  {
    Point pos;
    std::optional<double> theta;  // 目標角度（ラジアン）
    std::optional<double> max_velocity;
    std::optional<double> max_acceleration;
    double dwell_sec{0.0};
  };

  double default_max_velocity = 2.0;
  double default_max_acceleration = 2.5;
  double default_sleep_sec = 0.0;
  std::vector<Waypoint> waypoints;
  uint8_t target_robot_id = 0;
  std::shared_ptr<crane::PositionCommandWrapper> command = nullptr;

  size_t current_index = 0;
  std::optional<rclcpp::Time> sleep_until = std::nullopt;

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface = nullptr;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reload_sub = nullptr;
  std::string config_file_path;
  bool reload_requested = false;

  auto loadConfigFromFile(const std::string & path) -> bool;
  auto applyLegLimits(crane::PositionCommandWrapper & cmd, const Waypoint & wp) -> void;
};

}  // namespace crane
#endif  // CRANE_SESSIONS__TEST_TACTIC_HPP_
