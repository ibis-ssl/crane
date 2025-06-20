// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_

#include <algorithm>
#include <crane_comm/ddps.hpp>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/receive.hpp>
#include <crane_robot_skills/robot_command_as_skill.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class PassReceiverPlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Receive> receive_skill = nullptr;

  int pass_receiver_id = 0;

  Point pass_target;

  COMPOSITION_PUBLIC explicit PassReceiverPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("PassReceiver", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext &) override
  {
    if (receive_skill) {
      if (world_model->ball().isMoving(1.0)) {
        auto command = receive_skill->getRobotCommand();
        return {PlannerBase::Status::RUNNING, {command}};
      } else {
        auto robot_pos = receive_skill->commander()->getRobot()->pose.pos;
        auto points = crane::getDPPSPoints(robot_pos, 0.1, 1.0, 16);
        auto points_with_score =
          points | ranges::views::filter([&](const Point & p) {
            return world_model->point_checker.isFieldInside(p) &&
                   not world_model->point_checker.isPenaltyArea(p, 0.2 + 0.1);
            // 8.4.1 試合を再開する前の、停止、フリーキックの間、
            // すべてのロボットは相手のディフェンスエリアから少なくとも0.2m以上離れていなければならない。
          }) |
          ranges::views::filter([&](const Point & p) {
            if (auto nearest_enemy = world_model->getNearestRobotWithDistanceFromSegment(
                  {robot_pos, p}, world_model->theirs().getAvailableRobots());
                nearest_enemy.has_value()) {
              return nearest_enemy->distance > 0.2;
            } else {
              return true;
            }
          }) |
          ranges::views::transform([&](const Point & p) {
            if (auto nearest_enemy = world_model->getNearestRobotWithDistanceFromSegment(
                  {p, world_model->ball().pos}, world_model->theirs().getAvailableRobots());
                nearest_enemy.has_value()) {
              return std::make_pair(nearest_enemy->distance, p);
            } else {
              return std::make_pair(0.0, robot_pos);
            }
          }) |
          ranges::to<std::vector>();

        auto [min_score, max_score] = ranges::minmax_element(
          points_with_score, [](const auto & a, const auto & b) { return a.first < b.first; });
        if (min_score != points_with_score.end() && max_score != points_with_score.end()) {
          for (const auto & [score, point] : points_with_score) {
            visualizer->circle()
              .center(point)
              .radius(0.05)
              .fill("red", (score - min_score->first) / (max_score->first - min_score->first))
              .build();
          }
          visualizer->circle()
            .center(max_score->second)
            .radius(0.05)
            .fill("red")
            .stroke("black")
            .strokeWidth(20)
            .build();
          receive_skill->commander()->setTargetPosition(max_score->second).lookAtBall();
        } else {
          receive_skill->commander()->stopHere().lookAtBall();
        }
        return {PlannerBase::Status::SUCCESS, {receive_skill->getRobotCommand()}};
      }
    } else {
      return {PlannerBase::Status::RUNNING, {}};
    }
  }

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override
  {
    pass_receiver_id = -1;
    if (auto planner_context = context.find("AttackerSkill"); planner_context != context.end()) {
      if (auto pass_receiver = planner_context->second.find("pass_receiver");
          pass_receiver != planner_context->second.end()) {
        pass_receiver_id = static_cast<int>(pass_receiver->second);
      }
    }

    if (std::ranges::count(selectable_robots, pass_receiver_id) == 0) {
      receive_skill = nullptr;
      return {};
    } else {
      receive_skill =
        std::make_shared<skills::Receive>("pass_receiver", pass_receiver_id, world_model);
      return {pass_receiver_id};
    }
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__PASS_RECEIVER_PLANNER_HPP_
