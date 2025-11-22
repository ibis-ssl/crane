// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PASSABLE_BALL_PLACEMENT_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__PASSABLE_BALL_PLACEMENT_PLANNER_HPP_

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class PassableBallPlacementPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit PassableBallPlacementPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("passable_ball_placement", world_model)
  {
  }

  auto calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override
  {
    if (ball_placement) {
      ball_placement->run();
      return {PlannerBase::Status::RUNNING, {ball_placement->getRobotCommand()}};
    } else {
      return {PlannerBase::Status::RUNNING, {}};
    }
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    if (selectable_robots.empty()) {
      return {};
    } else {
      std::vector<uint8_t> selected_robots;
      // ボールに近いロボットを1台選択
      auto ball_selected_robots = this->getSelectedRobotsByScore(
        1, selectable_robots,
        [this](const std::shared_ptr<RobotInfo> & robot) {
          if (robot->id == world_model->getOurGoalieId()) {
            // ゴールキーパーは選出しない
            return -100.;
          } else {
            // ボールに近いほどスコアが高い
            return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
          }
        },
        prev_roles);

      if (ball_selected_robots.empty()) {
        return {};
      }

      ball_placement = std::make_shared<skills::SingleBallPlacement>(
        "ball_placement", selected_robots.front(), world_model);
      ball_placement->setParameter("pass_enable", true);
      return {ball_selected_robots.front()};
    }
  }

  std::shared_ptr<skills::SingleBallPlacement> ball_placement;
};

class PlacementTargetPlacerPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit PlacementTargetPlacerPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("placement_target_placer", world_model)
  {
  }

  auto calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override
  {
    if (placer) {
      placer->lookAtBall();
      placer->setDribblerTargetPosition(target);
      return {PlannerBase::Status::RUNNING, {placer->getMsg()}};
    } else {
      return {PlannerBase::Status::RUNNING, {}};
    }
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    if (selectable_robots.empty()) {
      return {};
    } else {
      std::vector<uint8_t> selected_robots;
      // ボールに近いロボットを1台選択
      if (auto placement_target = world_model->getBallPlacementTarget(); placement_target) {
        auto selected = this->getSelectedRobotsByScore(
          1, selectable_robots,
          [&](const std::shared_ptr<RobotInfo> & robot) {
            if (robot->id == world_model->getOurGoalieId()) {
              // ゴールキーパーは選出しない
              return -100.;
            } else {
              return 100.0 / std::max(robot->getDistance(placement_target.value()), 0.01);
            }
          },
          prev_roles);
        if (selected.empty()) {
          return {};
        } else {
          placer = std::make_shared<RobotCommandWrapper>("placer", selected.front(), world_model);
          target = placement_target.value();
          return {selected.front()};
        }
      } else {
        return {};
      }
    }
  }

  std::shared_ptr<RobotCommandWrapper> placer = nullptr;

  Point target;
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__PASSABLE_BALL_PLACEMENT_PLANNER_HPP_
