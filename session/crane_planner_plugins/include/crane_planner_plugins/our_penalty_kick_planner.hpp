// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__OUR_PENALTY_KICK_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__OUR_PENALTY_KICK_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/penalty_kick.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class OurPenaltyKickPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::PenaltyKick> kicker = nullptr;

  std::vector<std::shared_ptr<RobotCommandWrapper>> other_robots;

public:
  COMPOSITION_PUBLIC
  explicit OurPenaltyKickPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("OurPenaltyKickPlanner", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;

    for (auto & robot_command : other_robots) {
      // 関係ないロボットはボールより1m以上下がる(ルール5.3.5.3)
      Point target{};
      target << (world_model->getOurGoalCenter().x() + world_model->ball.pos.x()) / 2,
        robot_command->robot->pose.pos.y();
      robot_command->setTargetPosition(target);
      robot_command->setMaxVelocity(0.5);
      robot_commands.push_back(robot_command->getMsg());
    }
    if (kicker) {
      auto status = kicker->run(visualizer);
      robot_commands.emplace_back(kicker->getRobotCommand());
      if (status == skills::Status::SUCCESS) {
        return {PlannerBase::Status::SUCCESS, robot_commands};
      }
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    auto robots_sorted = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [&](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほうが先頭
        return 100. / robot->getDistance(world_model->ball.pos);
      });
    if (robots_sorted.size() > 0) {
      // 一番ボールに近いロボットがキッカー
      kicker = std::make_shared<skills::PenaltyKick>(robots_sorted.front(), world_model);
    }
    if (robots_sorted.size() > 1) {
      for (auto it = robots_sorted.begin() + 1; it != robots_sorted.end(); it++) {
        other_robots.emplace_back(std::make_shared<RobotCommandWrapper>(*it, world_model));
      }
    }
    return robots_sorted;
  }
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__OUR_PENALTY_KICK_PLANNER_HPP_
