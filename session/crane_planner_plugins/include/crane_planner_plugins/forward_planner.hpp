// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__FORWARD_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__FORWARD_PLANNER_HPP_

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/forward.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class ForwardPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit ForwardPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("forward", world_model)
  {
  }

  auto createForwardLines() const -> std::vector<Segment>
  {
    std::vector<Segment> forward_lines;

    const double goal_line_x = world_model->field_size.x() * 0.5;
    const double field_half_width = world_model->field_size.y() * 0.5;
    const double penalty_front_x = goal_line_x - world_model->penalty_area_size.y() * 0.5;
    const double penalty_side_y = world_model->penalty_area_size.y() * 0.5;
    const double side_center_y = std::midpoint(field_half_width, penalty_side_y);

    forward_lines.emplace_back(Point(0, 0), Point(penalty_front_x, 0.));
    forward_lines.emplace_back(Point(0, side_center_y), Point(goal_line_x, side_center_y));
    forward_lines.emplace_back(Point(0, -side_center_y), Point(goal_line_x, -side_center_y));

    const double mid_line_y = penalty_side_y * 0.5;
    forward_lines.emplace_back(Point(0, mid_line_y), Point(penalty_front_x, mid_line_y));
    forward_lines.emplace_back(Point(0, -mid_line_y), Point(penalty_front_x, -mid_line_y));

    // 攻めの方向に応じて符号を調整
    for (auto & line : forward_lines) {
      line.first.x() = -world_model->getOurSideSign() * line.first.x();
      line.second.x() = -world_model->getOurSideSign() * line.second.x();
    }

    return forward_lines;
  }

  auto calculateRobotCommand(const std::vector<RobotIdentifier> & robots, PlannerContext &)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto forward_skill : forward_skills) {
      forward_skill->run();
      robot_commands.emplace_back(forward_skill->getRobotCommand());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override
  {
    auto forward_lines = createForwardLines();
    const uint8_t selectable_num =
      std::min(selectable_robots_num, static_cast<uint8_t>(forward_lines.size()));
    auto selected = this->getSelectedRobotsByScore(
      selectable_num, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(robot->id);
      },
      prev_roles, context);
    if (forward_lines.size() > selected.size()) {
      forward_lines.resize(selected.size());
    }

    std::vector<Point> selected_robot_pos =
      selected | ranges::views::transform([this](const auto & robot_id) -> Point {
        return world_model->getOurRobot(robot_id)->pose.pos;
      }) |
      ranges::to<std::vector>;

    auto solution = getOptimalAssignments(selected_robot_pos, forward_lines);

    for (const auto & [index, selected_robot_id] : selected | ranges::views::enumerate) {
      auto skill = std::make_shared<skills::Forward>(selected_robot_id, world_model);
      auto line = forward_lines[solution[index]];
      skill->setParameter("front_point", line.second);
      skill->setParameter("back_point", line.first);
      forward_skills.emplace_back(skill);
    }

    return selected;
  }

  std::vector<std::shared_ptr<skills::Forward>> forward_skills;
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__FORWARD_PLANNER_HPP_
