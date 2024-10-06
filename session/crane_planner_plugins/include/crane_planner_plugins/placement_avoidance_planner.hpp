// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PLACEMENT_AVOIDANCE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__PLACEMENT_AVOIDANCE_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
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
struct CommandWithOriginalPosition
{
  std::shared_ptr<RobotCommandWrapperPosition> command;
  Point original_position;
};
class BallPlacementAvoidancePlanner : public PlannerBase
{
private:
  std::vector<CommandWithOriginalPosition> commands;

public:
  //  std::shared_ptr<skills::Attacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit BallPlacementAvoidancePlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("BallPlacementAvoidance", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;

    auto isInPlacementArea = [&](const Point & point) {
      if (auto placement_area = world_model->getBallPlacementArea(); placement_area) {
        return bg::distance(point, placement_area.value()) <= placement_area.value().radius;
      } else {
        return false;
      }
    };

    for (auto & command : commands) {
      if (isInPlacementArea(command.original_position)) {
        auto [distance, closest_point] = getClosestPointAndDistance(
          world_model->getBallPlacementArea().value().segment, command.original_position);
        // 0.6m離れる
        Point target_position =
          closest_point + (command.original_position - closest_point).normalized() * 0.6;
        if (not world_model->point_checker.isFieldInside(target_position, 0.2)) {
          // 一番近いフィールド外のポイントがだめなので逆方向に0.6m離れる
          target_position =
            closest_point + (closest_point - command.original_position).normalized() * 0.6;

          if (auto segment = world_model->getBallPlacementArea().value().segment;
              (closest_point == segment.first || closest_point == segment.second)) {
            // 一番近い点が端点の場合は単純に反対側の点を選択するだけではだめなので、
            // 垂直方向に0.6m離れた点を複数選択して、フィールド外かつ配置エリア外の点を選択する
            std::vector<Point> target_candidates;
            Vector2 vertical_vec =
              getVerticalVec((segment.second - segment.first).normalized()) * 0.6;
            target_candidates.push_back(closest_point + vertical_vec);
            target_candidates.push_back(closest_point - vertical_vec);

            if (auto target = std::ranges::find_if(
                  target_candidates,
                  [&](const auto & target_candidate) {
                    return (
                      not world_model->point_checker.isFieldInside(target_candidate, 0.2) &&
                      not isInPlacementArea(target_candidate));
                  });
                target != target_candidates.end()) {
              target_position = *target;
            } else {
              // どの候補もだめな場合は移動しない
              target_position = command.original_position;
            }
          }
        }
        command.command->setTargetPosition(target_position);
      } else {
        command.command->setTargetPosition(command.original_position);
      }
      robot_commands.push_back(command.command->getMsg());
    }
    return {Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    commands.clear();
    for (size_t index = 0; const auto & robot_id : selectable_robots) {
      if (index >= selectable_robots_num) {
        break;
      }

      commands.emplace_back(CommandWithOriginalPosition{
        std::make_shared<RobotCommandWrapperPosition>(
          "ball_placement_avoidance_planner", robot_id, world_model),
        world_model->getOurRobot(robot_id)->pose.pos});
      ++index;
    }
    // commandsからrobot_idのリストを作成
    return commands | ranges::views::transform([](const auto & command) {
             return command.command->getRobot()->id;
           }) |
           ranges::to<std::vector<uint8_t>>();
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__PLACEMENT_AVOIDANCE_PLANNER_HPP_
