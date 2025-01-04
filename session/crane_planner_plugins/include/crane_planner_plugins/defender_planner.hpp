// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/position_assignments.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_planner_plugins/defense_functions.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class DefenderPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit DefenderPlanner(WorldModelWrapper::SharedPtr & world_model)
  : PlannerBase("defender", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override;

  std::vector<Point> getDefenseArcPoints(const int robot_num, const Segment & ball_line) const;

  // defense_pointを中心にrobot_num台のロボットをdefense_line上に等間隔に配置する
  std::vector<Point> getDefenseLinePoints(const int robot_num, const Segment & ball_line) const;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override
  {
    Segment ball_line{world_model->goal, world_model->ball.pos};
    auto parameter = getDefenseLinePointParameter(ball_line, world_model);
    if (not parameter) {
      return {};
    }
    const auto defense_point = getDefenseLinePoint(parameter.value(), world_model);
    auto selected = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this, defense_point](const std::shared_ptr<RobotInfo> & robot) {
        // defense pointに近いほどスコアが高い
        return 100. - world_model->getSquareDistanceFromRobot(robot->id, defense_point);
      },
      prev_roles, context);

    return selected;
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
