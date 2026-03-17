// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_physics/position_assignments.hpp>
#include <crane_sessions/forward_session.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
auto ForwardSession::createForwardLines() const -> std::vector<Segment>
{
  std::vector<Segment> forward_lines;

  const double goal_line_x = world_model->fieldSize().x() * 0.5;
  const double field_half_width = world_model->fieldSize().y() * 0.5;
  const double penalty_front_x = goal_line_x - world_model->penaltyAreaSize().y() * 0.5;
  const double penalty_side_y = world_model->penaltyAreaSize().y() * 0.5;
  const double side_center_y = std::midpoint(field_half_width, penalty_side_y);
  const double ball_x_abs = world_model->ball().pos.x() * (-world_model->getOurSideSign());
  const double back_x = std::clamp(ball_x_abs - 1.0, -goal_line_x + 1.0, goal_line_x - 3.0);

  auto push_line = [&](Point p1, Point p2) {
    Segment line{p1, p2};
    if (bg::distance(line, world_model->ball().pos) > 0.5) {
      forward_lines.emplace_back(p1, p2);
      return true;
    } else {
      return false;
    }
  };

  push_line(Point(back_x, side_center_y), Point(goal_line_x - 1.0, side_center_y));
  push_line(Point(back_x, -side_center_y), Point(goal_line_x - 1.0, -side_center_y));
  push_line(Point(back_x, 0), Point(penalty_front_x, 0.));

  const double mid_line_y = side_center_y * 0.5;
  push_line(Point(back_x, mid_line_y), Point(penalty_front_x, mid_line_y));
  push_line(Point(back_x, -mid_line_y), Point(penalty_front_x, -mid_line_y));

  // 攻めの方向に応じて符号を調整
  for (auto & line : forward_lines) {
    line.first.x() = -world_model->getOurSideSign() * line.first.x();
    line.second.x() = -world_model->getOurSideSign() * line.second.x();
  }

  return forward_lines;
}

auto ForwardSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
  -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>>
{
  if (robots.empty()) {
    forward_skills.clear();
    return {SessionBase::Status::RUNNING, {}};
  }

  // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
  if (forward_skills.size() != robots.size()) {
    forward_skills.clear();

    auto forward_lines = createForwardLines();

    if (forward_lines.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("ForwardSession"),
        "Forward lines are empty. Falling back to stop-like commands.");

      for (const auto & robot_id : robots) {
        auto skill = std::make_shared<skills::Forward>(robot_id.id, world_model);
        const auto robot_pos = world_model->getRobot(robot_id)->pose.pos;
        skill->setParameter("front_point", robot_pos);
        skill->setParameter("back_point", robot_pos);
        skill->setParameter("max_vel", 1.5);
        skill->planner_visualizer = visualizer;
        forward_skills.emplace_back(skill);
      }
    } else {
      if (forward_lines.size() > robots.size()) {
        forward_lines.resize(robots.size());
      }

      std::vector<Point> robot_positions =
        robots | ranges::views::transform([this](const auto & robot_id) -> Point {
          return world_model->getRobot(robot_id)->pose.pos;
        }) |
        ranges::to<std::vector>;

      auto solution = getOptimalAssignments(robot_positions, forward_lines);

      forward_line_assignments.clear();
      for (const auto & [index, robot_id] : robots | ranges::views::enumerate) {
        auto skill = std::make_shared<skills::Forward>(robot_id.id, world_model);

        int line_index = solution[index];
        if (line_index < 0 || static_cast<size_t>(line_index) >= forward_lines.size()) {
          line_index = static_cast<int>(index % forward_lines.size());
          RCLCPP_WARN(
            rclcpp::get_logger("ForwardSession"),
            "Invalid assignment index (%d). Using fallback line index (%d).", solution[index],
            line_index);
        }

        const auto & line = forward_lines[line_index];
        skill->setParameter("front_point", line.second);
        skill->setParameter("back_point", line.first);
        skill->setParameter("max_vel", 1.5);
        skill->planner_visualizer = visualizer;
        forward_skills.emplace_back(skill);
        forward_line_assignments.emplace_back(line_index);
      }
    }
  }

  // 毎フレームback_xを更新してラインパラメータを再設定
  if (!forward_skills.empty() && forward_line_assignments.size() == forward_skills.size()) {
    auto current_lines = createForwardLines();
    if (!current_lines.empty()) {
      for (size_t i = 0; i < forward_skills.size(); ++i) {
        const int line_index = forward_line_assignments[i];
        if (static_cast<size_t>(line_index) < current_lines.size()) {
          const auto & line = current_lines[line_index];
          forward_skills[i]->setParameter("front_point", line.second);
          forward_skills[i]->setParameter("back_point", line.first);
        }
      }
    }
  }

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto forward_skill : forward_skills) {
    forward_skill->run();
    robot_commands.emplace_back(forward_skill->getRobotCommand());
  }
  return {SessionBase::Status::RUNNING, robot_commands};
}
}  // namespace crane
