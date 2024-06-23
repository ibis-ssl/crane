// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TIGERS_GOALIE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TIGERS_GOALIE_PLANNER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class TigersGoaliePlanner : public PlannerBase
{
public:
  enum class State {
    STOP,
    DEFEND,
    INTERCEPT,
    PASS,
    MOVE_TO_PENALTY_AREA,
    RAMBO,
    MOVE_IN_FRONT_OF_BALL,
    GET_BALL_CONTACT,
    MOVE_WITH_BALL,
    PREPARE_PENALTY,
  };

  COMPOSITION_PUBLIC
  explicit TigersGoaliePlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("tigers_goalie", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  Status doCriticalKeeper(const std::shared_ptr<RobotInfo> & robot, RobotCommandWrapper & command)
  {
    return Status::SUCCESS;
  }

  bool isBallMoveToweredTo(Point point)
  {
    double dot =
      (point - world_model->ball.pos).normalized().dot(world_model->ball.vel.normalized());
    return dot > 0.5 or (point - world_model->ball.pos).norm() < 0.2;
  }

  bool isBallAimedForGoal()
  {
    Segment goal_line{world_model->getOurGoalPosts().first, world_model->getOurGoalPosts().second};
    Segment ball_line{
      world_model->ball.pos, world_model->ball.pos + world_model->ball.vel.normalized() * 10.0};
    return boost::geometry::intersects(goal_line, ball_line);
  }

  bool isStopped() const { return false; }

  bool isPreparePenalty() const { return false; }

  bool isKeeperWellInsidePenaltyArea() const { return false; }

  bool ballCanBePassedOutOfPenaltyArea() const { return false; }

  bool canGoOut() const { return false; }

  bool isBallBetweenGoalieAndGoal() const { return false; }

  bool isOutsidePenaltyArea() const { return false; }

  bool canInterceptSafely()
  {
    return false;
    //    return world_model->isDefenseArea(world_model->ball.pos) && (not isBallAimedForGoal());
  }

  bool isBallMoving() const { return false; }

  bool isBallPlacementRequired() const { return false; }
  bool hasInterceptionFailed(const std::shared_ptr<RobotInfo> & robot)
  {
    return isBallMoveToweredTo(robot->pose.pos) or
           not world_model->isDefenseArea(world_model->ball.pos);
  }

  bool isGoalKick() const { return false; }

  bool isBallPlaced() const { return false; }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;

  State state = State::DEFEND;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__TIGERS_GOALIE_PLANNER_HPP_
