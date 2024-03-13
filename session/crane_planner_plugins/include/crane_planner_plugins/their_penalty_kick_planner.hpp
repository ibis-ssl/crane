// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__THEIR_PENALTY_KICK_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__THEIR_PENALTY_KICK_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class TheirPenaltyKickPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::Goalie> goalie = nullptr;

  std::vector<std::shared_ptr<RobotCommandWrapper>> other_robots;

public:
  COMPOSITION_PUBLIC
  explicit TheirPenaltyKickPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("TheirPenaltyKickPlanner", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;

    for (auto & robot_command : other_robots) {
      // 関係ないロボットはボールより1m以上下がる(ルール5.3.5.3)
      Point target{};
      target << (world_model->getTheirGoalCenter().x() + world_model->ball.pos.x()) / 2,
        robot_command->robot->pose.pos.y();
      robot_command->setTargetPosition(target);
      robot_command->setMaxVelocity(0.5);
      robot_commands.push_back(robot_command->getMsg());
    }
    if (goalie) {
      if (
        world_model->play_situation.getSituationCommandID() ==
        crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION) {
        goalie->commander()->setTargetPosition(world_model->getOurGoalCenter());
        goalie->commander()->lookAtBall();
        goalie->commander()->setMaxVelocity(0.5);
      } else {
        auto status = goalie->run(visualizer);
      }
      robot_commands.emplace_back(goalie->getRobotCommand());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    goalie = std::make_shared<skills::Goalie>(world_model->getOurGoalieId(), world_model);
    auto robots_sorted = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [&](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほうが先頭
        return 100. / robot->getDistance(world_model->ball.pos);
      });
    for (auto it = robots_sorted.begin(); it != robots_sorted.end(); it++) {
      if (*it != world_model->getOurGoalieId()) {
        other_robots.emplace_back(std::make_shared<RobotCommandWrapper>(*it, world_model));
      }
    }
    return robots_sorted;
  }
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__THEIR_PENALTY_KICK_PLANNER_HPP_
