// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__PLACEMENT_AVOIDANCE_SESSION_HPP_
#define CRANE_SESSIONS__PLACEMENT_AVOIDANCE_SESSION_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_sessions/session_base.hpp>
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
class BallPlacementAvoidanceSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC explicit BallPlacementAvoidanceSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("placement_avoidance", world_model)
  {
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      // ボール配置エリアへの距離（近いほど優先）
      if (auto placement_area = wm->getBallPlacementArea(); placement_area) {
        return bg::distance(robot->pose.pos, placement_area.value());
      }
      return 0.0;
    };
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;

    const auto placement_area_opt = world_model->getBallPlacementArea();

    auto isInPlacementArea = [&placement_area_opt](const Point & point, double offset) -> bool {
      if (!placement_area_opt) return false;
      return bg::distance(point, placement_area_opt.value()) <=
             placement_area_opt.value().radius + offset;
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
        auto [distance, closest_point] =
          getClosestPointAndDistance(placement_area_opt.value().segment, current_position);
        // 0.8m離れる
        target_position = closest_point + (current_position - closest_point).normalized() * 0.8;

        if (not world_model->point_checker.isFieldInside(target_position, 0.2)) {
          // 一番近いフィールド外のポイントがだめなので逆方向に0.8m離れる
          target_position = closest_point + (closest_point - current_position).normalized() * 0.8;

          if (
            const auto & segment = placement_area_opt.value().segment;
            (closest_point == segment.first || closest_point == segment.second)) {
            // 一番近い点が端点の場合は単純に反対側の点を選択するだけではだめなので、
            // 垂直方向に0.8m離れた点を複数選択して、フィールド内かつ配置エリア外の点を選択する
            Vector2 vertical_vec =
              getVerticalVec((segment.second - segment.first).normalized()) * 0.8;
            std::array<Point, 2> target_candidates = {
              closest_point + vertical_vec, closest_point - vertical_vec};

            if (
              auto target = std::ranges::find_if(
                target_candidates,
                [&](const auto & target_candidate) {
                  return (
                    world_model->point_checker.isFieldInside(target_candidate, 0.2) &&
                    not isInPlacementArea(target_candidate, 0.1));
                });
              target != target_candidates.end()) {
              target_position = *target;
            } else {
              // 垂直方向の2候補もだめな場合は放射状8方向で有効点を探索する
              std::array<Point, 8> radial_candidates;
              for (int i = 0; i < 8; i++) {
                double angle = i * M_PI / 4.0;
                radial_candidates[i] =
                  closest_point + Point(std::cos(angle), std::sin(angle)) * 0.8;
              }
              auto valid = std::ranges::find_if(radial_candidates, [&](const auto & c) {
                return world_model->point_checker.isFieldInside(c, 0.2) &&
                       not isInPlacementArea(c, 0.1);
              });
              if (valid != radial_candidates.end()) {
                target_position = *valid;
              } else {
                // 最終フォールバック: 移動しない
                target_position = current_position;
              }
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
#endif  // CRANE_SESSIONS__PLACEMENT_AVOIDANCE_SESSION_HPP_
