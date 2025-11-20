// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_planner_plugins/total_defense_planner.hpp>
#include <limits>
#include <numeric>
#include <ranges>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
TotalDefensePlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext &)
{
  if (robots.empty()) {
    return {PlannerBase::Status::RUNNING, {}};
  }

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  auto defender_robots = robots | ranges::views::filter([&](const auto & robot) {
                           return robot.id != world_model->getOurGoalieId();
                         }) |
                         ranges::to<std::vector>();

  auto ball = world_model->ball().pos;

  //
  // calc ball line
  //
  Segment ball_line(ball, ball + world_model->ball().vel.normalized() * 20.f);
  {
    // シュート判定
    auto goal_posts = world_model->getOurGoalPosts();
    Segment goal_line(goal_posts.first, goal_posts.second);
    auto intersections = getIntersections(ball_line, goal_line);
    if (intersections.empty()) {
      // シュートがなければ通常の動き
      ball_line.first = ball;
      ball_line.second = world_model->getOurGoalCenter();
    }
  }

  auto defense_parameter = getDefenseLinePointParameter(ball_line, world_model);
  Segment defense_parameter_goal_line = ball_line;
  if (not defense_parameter) {
    defense_parameter_goal_line = Segment{
      world_model->goal(),
      world_model->ball().pos + (world_model->ball().pos - world_model->goal()).normalized() * 2.0};
    defense_parameter = getDefenseLinePointParameter(defense_parameter_goal_line, world_model);
  }

  std::vector<Point> defense_points;
  if (defense_parameter) {
    defense_points = getDefenseLinePoints(
      defender_robots.size(), ball_line, world_model, m_is_goalie_total_defense_mode,
      *defense_parameter);
  }

  if (goalie) {
    goalie->run();
    robot_commands.emplace_back(goalie->getRobotCommand());
  }

  if (not defense_points.empty()) {
    auto defender_commands = assignRobotsToPoints(
      defender_robots, defense_points, "total_defense_planner", world_model->ball().pos,
      [&](std::shared_ptr<RobotCommandWrapper> & command) {
        command->disableBasicAvoidances();
      });
    for (const auto & cmd : defender_commands) {
      robot_commands.emplace_back(cmd);
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  } else {
    for (auto robot_id = defender_robots.begin(); robot_id != defender_robots.end(); ++robot_id) {
      int index = std::distance(defender_robots.begin(), robot_id);
      [[maybe_unused]] Point target_point = [&]() {
        if (not defense_points.empty()) {
          return defense_points.at(index);
        } else {
          return Point(0, 0);
        }
      }();

      auto command = std::make_shared<RobotCommandWrapper>(
        "total_defense_planner/stop", robot_id->id, world_model);

      auto robot = world_model->getRobot(*robot_id);

      // Stop at same position
      command->stopHere();

      robot_commands.emplace_back(command->getMsg());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }
}

auto TotalDefensePlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  std::vector<uint8_t> selected;
  std::vector<uint8_t> remaining_robots = selectable_robots;
  // キーパーを確保
  auto goalie_id = world_model->getOurGoalieId();
  if (ranges::count(selectable_robots, goalie_id) > 0) {
    selected.push_back(goalie_id);
    remaining_robots |=
      ranges::actions::remove_if([goalie_id](auto elem) { return elem == goalie_id; });
    goalie = std::make_shared<skills::Goalie>(world_model->getOurGoalieId(), world_model);
  }

  // TODO(HansRobo): Attackerを供出するかどうかの実装
  remaining_robots |= ranges::actions::remove_if(
    [&](auto elem) { return elem == world_model->getOurFrontier()->robot->id; });

  // 直接脅威へのディフェンダー
  Segment ball_line{world_model->goal(), world_model->ball().pos};
  auto parameter = getDefenseLinePointParameter(ball_line, world_model);
  if (not parameter) {
    // ペナルティエリア内にボールが侵入したときにディフェンダがいなくならないように対応
    Segment alternative_ball_line{
      world_model->goal(),
      world_model->ball().pos + (world_model->ball().pos - world_model->goal()).normalized() * 2.0};
    parameter = getDefenseLinePointParameter(alternative_ball_line, world_model);
  }

  if (not parameter) {
    return selected;
  } else {
    const auto defense_point = getDefenseLinePoint(parameter.value(), world_model);
    auto selected_first_defenders = this->getSelectedRobotsByScore(
      selectable_robots_num - selected.size(), remaining_robots,
      [this, defense_point](const std::shared_ptr<RobotInfo> & robot) {
        // defense pointに近いほどスコアが高い
        return 100. - world_model->getSquareDistanceFromRobot(robot->id, defense_point);
      },
      prev_roles, context);

    ranges::copy(selected_first_defenders, ranges::back_inserter(selected));
    ranges::remove_if(remaining_robots, [selected_first_defenders](const uint8_t id) {
      return ranges::any_of(
        selected_first_defenders, [id](const uint8_t selected_id) { return selected_id == id; });
    });

    // TODO(HansRobo): 間接脅威へのディフェンダー

    return selected;
  }
}
}  // namespace crane
