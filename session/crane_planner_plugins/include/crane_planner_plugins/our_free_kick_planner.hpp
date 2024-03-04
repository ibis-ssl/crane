// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__OUR_FREE_KICK_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__OUR_FREE_KICK_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/simple_attacker.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class OurDirectFreeKickPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::SimpleAttacker> kicker = nullptr;

  std::vector<std::shared_ptr<RobotCommandWrapper>> other_robots;

public:
  COMPOSITION_PUBLIC
  explicit OurDirectFreeKickPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("OurDirectFreeKickPlanner", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;

    for (auto & robot_command : other_robots) {
      robot_command->stopHere();
      robot_commands.push_back(robot_command->getMsg());
    }
    if (kicker) {
      auto command = kicker->commander();
      auto [best_angle, goal_angle_width] =
        world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
      Point best_target = world_model->ball.pos + getNormVec(best_angle) * 0.3;

      // シュートの隙がないときは仲間へパス
      if (goal_angle_width < 0.07) {
        auto our_robots = world_model->ours.getAvailableRobots(command->robot->id);
        our_robots.erase(
          std::remove_if(
            our_robots.begin(), our_robots.end(),
            [&](const auto & robot) {
              bool erase_flag = false;
              if (auto role = PlannerBase::robot_roles->find(robot->id);
                  role != PlannerBase::robot_roles->end()) {
                if (role->second.planner_name == "defender") {
                  // defenderにはパスしない
                  erase_flag = true;
                } else if (role->second.planner_name.find("goalie") != std::string::npos) {
                  // キーパーにもパスしない
                  erase_flag = true;
                }
              }
              return erase_flag;
            }),
          our_robots.end());

        auto nearest_robot =
          world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, our_robots);
        best_target = nearest_robot.first->pose.pos;
      }

      // 経由ポイント

      Point intermediate_point =
        world_model->ball.pos + (world_model->ball.pos - best_target).normalized() * 0.2;

      double dot = (command->robot->pose.pos - world_model->ball.pos)
                     .normalized()
                     .dot((world_model->ball.pos - best_target).normalized());
      double target_theta = getAngle(best_target - world_model->ball.pos);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (dot < 0.95 || std::abs(getAngleDiff(target_theta, command->robot->pose.theta)) > 0.05) {
        command->setTargetPosition(intermediate_point);
        command->enableCollisionAvoidance();
      } else {
        command->setTargetPosition(world_model->ball.pos);
        command->kickStraight(0.7).disableCollisionAvoidance();
        command->enableCollisionAvoidance();
        command->disableBallAvoidance();
      }

      command->setTargetTheta(getAngle(best_target - world_model->ball.pos));

      bool is_in_defense = world_model->isDefenseArea(world_model->ball.pos);
      bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

      if ((not is_in_field) or is_in_defense) {
        // stop here
        command->stopHere();
      }
      robot_commands.emplace_back(kicker->getRobotCommand());
    }
    return {Status::RUNNING, robot_commands};
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
    // ゴールキーパーはキッカーに含めない(ロボットがキーパーのみの場合は除く)
    if (robots_sorted.size() > 1 && robots_sorted.front()->id == world_model->getOurGoalieId()) {
      robots_sorted.erase(robots_sorted.begin());
    }
    if (robots_sorted.size() > 0) {
      // 一番ボールに近いロボットがキッカー
      kicker = std::make_shared<skills::SimpleAttacker>(robots_sorted.front(), world_model);
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
#endif  // CRANE_PLANNER_PLUGINS__OUR_FREE_KICK_PLANNER_HPP_
