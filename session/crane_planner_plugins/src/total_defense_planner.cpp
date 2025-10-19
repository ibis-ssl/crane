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
  [[maybe_unused]] const double OFFSET_X = 0.2;
  [[maybe_unused]] const double OFFSET_Y = 0.2;

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
      defender_robots.size(), ball_line, m_is_goalie_total_defense_mode, *defense_parameter);
  }

  if (goalie) {
    goalie->run();
    robot_commands.emplace_back(goalie->getRobotCommand());
  }

  if (not defense_points.empty()) {
    std::vector<Point> robot_points;
    for (auto robot_id : defender_robots) {
      robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
    }

    auto solution = getOptimalAssignments(robot_points, defense_points);

    for (auto robot_id = defender_robots.begin(); robot_id != defender_robots.end(); ++robot_id) {
      int index = std::distance(defender_robots.begin(), robot_id);
      Point target_point = defense_points[solution[index]];

      auto command =
        std::make_shared<RobotCommandWrapper>("total_defense_planner", robot_id->id, world_model);
      auto robot = world_model->getRobot(*robot_id);

      command->setTargetPosition(target_point);
      command->setTargetTheta(getAngle(world_model->ball().pos - target_point));
      command->disableCollisionAvoidance();
      command->disableBallAvoidance();

      robot_commands.emplace_back(command->getMsg());
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

std::vector<Point> TotalDefensePlanner::getDefenseArcPoints(
  const int robot_num, const Segment & ball_line) const
{
  const double DEFENSE_INTERVAL = 0.5;
  const double RADIUS_OFFSET = 0.5;
  std::vector<Point> defense_points;
  // ペナルティエリアの一番遠い点を通る円の半径
  const double RADIUS =
    std::hypot(world_model->penaltyAreaSize().x(), world_model->penaltyAreaSize().y() * 0.5) +
    RADIUS_OFFSET;
  // r * theta = interval
  // theta = interval / e
  const double ANGLE_INTERVAL = DEFENSE_INTERVAL / RADIUS;

  auto defense_point = [&]() -> Point {
    Circle circle;
    circle.center = world_model->getOurGoalCenter();
    circle.radius = RADIUS;
    auto intersections = getIntersections(circle, ball_line);
    switch (static_cast<int>(intersections.size())) {
      case 0: {
        // ボールの進行方向がこちらを向いていないときは、中間地点に潜り込む
        return world_model->getOurGoalCenter() +
               (world_model->ball().pos - world_model->getOurGoalCenter()).normalized() * RADIUS;
      }
      case 1: {
        return intersections[0];
      }
      default: {
        // ボールに一番近い交点を返す
        double min_distance = std::numeric_limits<double>::max();
        Point best_intersection =
          world_model->getOurGoalCenter() +
          (world_model->ball().pos - world_model->getOurGoalCenter()).normalized() * RADIUS;
        for (auto & intersection : intersections) {
          double distance = (world_model->ball().pos - intersection).norm();
          if (distance < min_distance) {
            min_distance = distance;
            best_intersection = intersection;
          }
        }
        return best_intersection;
      }
    }
  }();

  double defense_angle = getAngle(defense_point - world_model->getOurGoalCenter());
  for (int i = 0; i < robot_num; i++) {
    double normalized_angle_offset = (robot_num - i - 1) / 2.;
    defense_points.emplace_back(
      world_model->getOurGoalCenter() +
      getNormVec(defense_angle + ANGLE_INTERVAL * normalized_angle_offset) * RADIUS);
  }
  return defense_points;
}

/// @brief ディフェンダーの壁の位置を返す
/// @param defense_robot_num キーパーを除く壁の枚数
/// @param is_open_center ゴーリーを中央に置くかどうか
/// @param defense_parameter ディフェンスラインのパラメータ
/// @return センターから順に並べたディフェンスラインのポイント
///         is_open_centerがtrueの場合はゴーリーを中央に置く
std::vector<Point> TotalDefensePlanner::getDefenseLinePoints(
  const int defense_robot_num, const Segment & ball_line, const bool is_open_center,
  const double defense_parameter) const
{
  const double DEFENSE_INTERVAL = 0.2;
  std::vector<Point> defense_points;

  if (defense_parameter) {
    double upper_parameter = defense_parameter;
    double lower_parameter = upper_parameter;

    auto add_parameter = [&](double parameter) -> bool {
      const double OFFSET_X = 0.2;
      const double OFFSET_Y = 0.2;
      auto [threshold1, threshold2, threshold3] =
        getDefenseLinePointParameterThresholds(OFFSET_X, OFFSET_Y, world_model);
      if (parameter < 0. || parameter > threshold3) {
        return false;
      } else {
        if (upper_parameter < parameter) {
          upper_parameter = parameter;
        }
        if (lower_parameter > parameter) {
          lower_parameter = parameter;
        }
        defense_points.push_back(getDefenseLinePoint(parameter, world_model));
        return true;
      }
    };
    // 1台目
    if (not is_open_center) {
      upper_parameter = defense_parameter;
      lower_parameter = defense_parameter;
      add_parameter(defense_parameter);
    }
    // is_open_centerがtrueのときは両脇から配置開始する
    const int remaining_robot_num = is_open_center ? defense_robot_num : defense_robot_num - 1;
    // 中央の開け具合を計算する。前進守備するとき(ゴールにボールが近いとき)は開けない
    auto open_center_ratio_opt = world_model->getForwardDefenseRatio(ball_line);
    double open_center_interval = 0.0;
    if (not open_center_ratio_opt) {
      open_center_interval = DEFENSE_INTERVAL;
    } else {
      open_center_interval = (1.0 - (*open_center_ratio_opt)) * DEFENSE_INTERVAL;
    }
    // 2台目以降
    for (int i = 0; i < remaining_robot_num; i++) {
      if (is_open_center && i < 2) {
        // 中央を開けるとき
        if (i == 0) {
          if (not add_parameter(upper_parameter + open_center_interval)) {
            add_parameter(lower_parameter - open_center_interval);
          }
        } else if (i == 1) {
          if (not add_parameter(lower_parameter - open_center_interval)) {
            add_parameter(upper_parameter + open_center_interval);
          }
        }
      } else if (i % 2 == 0) {
        // upper側に追加
        if (not add_parameter(upper_parameter + DEFENSE_INTERVAL)) {
          // だめならlower側
          add_parameter(lower_parameter - DEFENSE_INTERVAL);
        }
      } else {
        // lower側に追加
        if (not add_parameter(lower_parameter - DEFENSE_INTERVAL)) {
          // だめならupper側
          add_parameter(upper_parameter + DEFENSE_INTERVAL);
        }
      }
    }
  }

  return defense_points;
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
