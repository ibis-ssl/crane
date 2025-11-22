// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TEST_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TEST_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/planner_base.hpp>
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
class TestPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC explicit TestPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;

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
  std::shared_ptr<crane::RobotCommandWrapper> command = nullptr;

  size_t current_index = 0;
  std::optional<rclcpp::Time> sleep_until = std::nullopt;

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface = nullptr;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reload_sub = nullptr;
  std::string config_file_path;
  bool reload_requested = false;

  auto loadConfigFromFile(const std::string & path) -> bool;
  auto applyLegLimits(crane::RobotCommandWrapper & cmd, const Waypoint & wp) -> void;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__TEST_PLANNER_HPP_
