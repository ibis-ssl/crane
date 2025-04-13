// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SANDWICH_BALL_PALCEMENT_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SANDWICH_BALL_PALCEMENT_PLANNER_HPP_

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class SandwichBallPlacementPlanner : public PlannerBase
{
  enum class State {
    PREPARE,
    APPROACH,
    MOVE,
    LEAVE,
  };

public:
  COMPOSITION_PUBLIC
  explicit SandwichBallPlacementPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : PlannerBase("sandwich_ball_placement", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override;

private:
  std::unordered_map<uint8_t, Pose2D> stop_poses;

  std::pair<
    std::shared_ptr<RobotCommandWrapper>, std::shared_ptr<RobotCommandWrapper>>
    placers;

  State state = State::PREPARE;

  Point last_ball;

  Vector2 sandwich_direction;
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SANDWICH_BALL_PALCEMENT_PLANNER_HPP_
