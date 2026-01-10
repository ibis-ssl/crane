// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__PLACEMENT_AVOIDANCE_TACTIC_HPP_
#define CRANE_TACTICS__PLACEMENT_AVOIDANCE_TACTIC_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_tactics/tactic_base.hpp>
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
class BallPlacementAvoidanceTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC explicit BallPlacementAvoidanceTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("BallPlacementAvoidance", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override
  {
    std::vector<crane_msgs::msg::PositionCommand> robot_commands;

    auto isInPlacementArea = [this](const Point & point, double offset) {
      if (auto placement_area = world_model->getBallPlacementArea(); placement_area) {
        return bg::distance(point, placement_area.value()) <=
               placement_area.value().radius + offset;
      } else {
        return false;
      }
    };

    for (const auto & robot_id : robots) {
      auto robot = world_model->getOurRobot(robot_id.id);
      if (!robot) {
        continue;
      }

      Point current_position = robot->pose.pos;
      auto command = std::make_shared<PositionCommandWrapper>(
        "ball_placement_avoidance", robot_id.id, world_model);

      Point target_position = current_position;

      if (isInPlacementArea(current_position, 0.2)) {
        auto [distance, closest_point] = getClosestPointAndDistance(
          world_model->getBallPlacementArea().value().segment, current_position);
        // 0.8m離れる
        target_position = closest_point + (current_position - closest_point).normalized() * 0.8;

        if (not world_model->point_checker.isFieldInside(target_position, 0.2)) {
          // 一番近いフィールド外のポイントがだめなので逆方向に0.8m離れる
          target_position = closest_point + (closest_point - current_position).normalized() * 0.8;

          if (auto segment = world_model->getBallPlacementArea().value().segment;
              (closest_point == segment.first || closest_point == segment.second)) {
            // 一番近い点が端点の場合は単純に反対側の点を選択するだけではだめなので、
            // 垂直方向に0.8m離れた点を複数選択して、フィールド内かつ配置エリア外の点を選択する
            std::vector<Point> target_candidates;
            Vector2 vertical_vec =
              getVerticalVec((segment.second - segment.first).normalized()) * 0.8;
            target_candidates.push_back(closest_point + vertical_vec);
            target_candidates.push_back(closest_point - vertical_vec);

            if (auto target = std::ranges::find_if(
                  target_candidates,
                  [&](const auto & target_candidate) {
                    return (
                      world_model->point_checker.isFieldInside(target_candidate, 0.2) &&
                      not isInPlacementArea(target_candidate, 0.1));
                  });
                target != target_candidates.end()) {
              target_position = *target;
            } else {
              // どの候補もだめな場合は移動しない
              target_position = current_position;
            }
          }
        }

        visualizer->line()
          .start(current_position)
          .end(target_position)
          .stroke("yellow")
          .strokeWidth(20)
          .build();
      }

      command->setTargetPosition(target_position);
      command->disableGoalAreaAvoidance().setTargetTheta(robot->pose.theta);
      robot_commands.push_back(command->getMsg());
    }
    return {Status::RUNNING, robot_commands};
  }
};

}  // namespace crane
#endif  // CRANE_TACTICS__PLACEMENT_AVOIDANCE_TACTIC_HPP_
