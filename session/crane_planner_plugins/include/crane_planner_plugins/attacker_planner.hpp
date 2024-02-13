// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/control_target.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class AttackerPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit AttackerPlanner(
    WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)
  : PlannerBase("attacker", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id : robots) {
      crane::RobotCommandWrapper target(robot_id.robot_id, world_model);
      auto robot = world_model->getRobot(robot_id);

      auto [best_target, goal_angle_width] = getBestShootTargetWithWidth();

      // シュートの隙がないときは仲間へパス
      if (goal_angle_width < 0.07) {
        auto our_robots = world_model->ours.getAvailableRobots();
        our_robots.erase(
          std::remove_if(
            our_robots.begin(), our_robots.end(),
            [&](const auto & robot) { return robot->id == robot_id.robot_id; }),
          our_robots.end());
        auto nearest_robot =
          world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, our_robots);
        best_target = nearest_robot.first->pose.pos;
      }

      // 経由ポイント

      Point intermediate_point =
        world_model->ball.pos + (world_model->ball.pos - best_target).normalized() * 0.2;

      double dot = (robot->pose.pos - world_model->ball.pos)
                     .normalized()
                     .dot((world_model->ball.pos - best_target).normalized());
      double target_theta = getAngle(best_target - world_model->ball.pos);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (dot < 0.95 || std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.05) {
        target.setTargetPosition(intermediate_point);
        target.enableCollisionAvoidance();
      } else {
        target.setTargetPosition(world_model->ball.pos);
        target.kickStraight(0.7).disableCollisionAvoidance();
        target.enableCollisionAvoidance();
        target.disableBallAvoidance();
      }

      target.setTargetTheta(getAngle(best_target - world_model->ball.pos));

      bool is_in_defense = world_model->isDefenseArea(world_model->ball.pos);
      bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

      if ((not is_in_field) or is_in_defense) {
        // stop here
        target.stopHere();
      }
      control_targets.emplace_back(target.getMsg());
    }
    return {PlannerBase::Status::RUNNING, control_targets};
  }

protected:
  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほどスコアが高い
        return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
      });
  }

  auto getBestShootTargetWithWidth() -> std::pair<Point, double>
  {
    const auto & ball = world_model->ball.pos;

    Interval goal_range;

    auto goal_posts = world_model->getTheirGoalPosts();
    goal_range.append(getAngle(goal_posts.first - ball), getAngle(goal_posts.second - ball));

    for (auto & enemy : world_model->theirs.robots) {
      double distance = (ball - enemy->pose.pos).norm();
      constexpr double MACHINE_RADIUS = 0.1;

      double center_angle = getAngle(enemy->pose.pos - ball);
      double diff_angle =
        atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

      goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
    }

    auto largest_interval = goal_range.getLargestInterval();

    double target_angle = (largest_interval.first + largest_interval.second) / 2.0;

    return {
      ball + getNormVec(target_angle) * 0.5, largest_interval.second - largest_interval.first};
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_
