// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <crane_tactics/total_defense_tactic.hpp>
#include <limits>
#include <numeric>
#include <range/v3/action/remove_if.hpp>
#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/algorithm/count.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/functional/comparisons.hpp>
#include <range/v3/iterator/insert_iterators.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <ranges>

namespace crane
{
std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
TotalDefenseTactic::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {TacticBase::Status::RUNNING, {}};
  }

  std::vector<crane_msgs::msg::PositionCommand> robot_commands;

  auto defender_robots = robots | ranges::views::filter([&](const auto & robot) {
                           return robot.id != world_model->getOurGoalieId();
                         }) |
                         ranges::to<std::vector>();

  const auto & ball = world_model->ball();

  //
  // calc ball line
  //
  Segment ball_line(ball.pos, ball.pos + ball.vel.normalized() * 20.f);
  {
    // シュート判定
    auto goal_posts = world_model->getOurGoalPosts();
    Segment goal_line(goal_posts.first, goal_posts.second);
    auto intersections = getIntersections(ball_line, goal_line);
    if (intersections.empty()) {
      // シュートがなければ通常の動き
      ball_line.first = ball.pos;
      ball_line.second = world_model->getOurGoalCenter();
    }
  }

  auto defense_parameter = getDefenseLinePointParameter(ball_line, world_model);
  Segment defense_parameter_goal_line = ball_line;
  if (not defense_parameter) {
    defense_parameter_goal_line =
      Segment{world_model->goal(), ball.pos + (ball.pos - world_model->goal()).normalized() * 2.0};
    defense_parameter = getDefenseLinePointParameter(defense_parameter_goal_line, world_model);
  }

  // ディフェンダー数を決定（パラメータで制御可能、残りはMarkerへ）
  const size_t max_defense_line_robots =
    static_cast<size_t>(getTacticParameter<int>("max_defense_line_robots", 3));
  size_t num_defense_line_robots = std::min(defender_robots.size(), max_defense_line_robots);

  std::vector<Point> defense_points;
  if (defense_parameter) {
    defense_points = getDefenseLinePoints(
      num_defense_line_robots, ball_line, world_model, m_is_goalie_total_defense_mode,
      *defense_parameter);
  }

  if (goalie) {
    goalie->run();
    robot_commands.emplace_back(goalie->getRobotCommand());
  }

  // ディフェンダーとマーカーのロボットを分割
  std::vector<RobotIdentifier> defense_line_robots;
  std::vector<uint8_t> marker_robot_ids;

  if (not defense_points.empty()) {
    // defense_pointsの数だけディフェンダーを選択、残りはMarkerへ
    for (size_t i = 0; i < defender_robots.size(); ++i) {
      if (i < defense_points.size()) {
        defense_line_robots.push_back(defender_robots[i]);
      } else {
        marker_robot_ids.push_back(defender_robots[i].id);
      }
    }
  } else {
    // defense_pointsがない場合は全員Markerへ
    for (const auto & robot : defender_robots) {
      marker_robot_ids.push_back(robot.id);
    }
  }

  // ディフェンダーをdefense_pointsに割り当て
  if (not defense_line_robots.empty() && not defense_points.empty()) {
    auto defender_commands = assignRobotsToPoints(
      defense_line_robots, defense_points, "total_defense_planner", ball.pos,
      [&](std::shared_ptr<PositionCommandWrapper> & command) {
        command->disableBasicAvoidances();
      });
    for (const auto & cmd : defender_commands) {
      robot_commands.emplace_back(cmd);
    }
  }

  // SecondThreatDefender用に1台確保
  const bool enable_second_threat_defender =
    getTacticParameter<bool>("enable_second_threat_defender", true);
  constexpr double SECOND_THREAT_DEFENDER_OFFSET = 0.3;

  if (enable_second_threat_defender && not marker_robot_ids.empty()) {
    auto target =
      skills::SecondThreatDefender::getDefaultPoint(world_model, SECOND_THREAT_DEFENDER_OFFSET);

    // targetに最も近いロボットを選出
    auto remaining_robots =
      marker_robot_ids |
      ranges::views::transform([&](const auto & id) { return world_model->getOurRobot(id); }) |
      ranges::to<std::vector>();

    auto best_robot = ranges::min(remaining_robots, ranges::less{}, [&](const auto & robot) {
      return (robot->pose.pos - target).norm();
    });

    // SecondThreatDefenderスキル作成
    second_threat_defender =
      std::make_shared<skills::SecondThreatDefender>(best_robot->id, world_model);
    second_threat_defender->setParameter("offset", SECOND_THREAT_DEFENDER_OFFSET);

    // marker_robot_idsから除外
    marker_robot_ids.erase(
      std::remove(marker_robot_ids.begin(), marker_robot_ids.end(), best_robot->id),
      marker_robot_ids.end());
  } else {
    second_threat_defender.reset();
  }

  // SecondThreatDefenderの実行
  if (second_threat_defender) {
    second_threat_defender->run();
    robot_commands.emplace_back(second_threat_defender->getRobotCommand());
  }

  // 残りのロボットをMarkerに割り当て
  if (not marker_robot_ids.empty()) {
    assignMarkingTargets(marker_robot_ids);
  } else {
    std::lock_guard lock(markers_mutex);
    markers.clear();
  }

  // Markerの実行
  {
    auto lock = std::lock_guard(markers_mutex);
    for (const auto & marker : markers) {
      marker->run();
      robot_commands.emplace_back(marker->getRobotCommand());
    }
  }

  if (not defense_line_robots.empty() || not markers.empty()) {
    return {TacticBase::Status::RUNNING, robot_commands};
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

      auto command = std::make_shared<PositionCommandWrapper>(
        "total_defense_planner/stop", robot_id->id, world_model);

      auto robot = world_model->getRobot(*robot_id);

      // Stop at same position
      command->stopHere();

      robot_commands.emplace_back(command->getMsg());
    }
    return {TacticBase::Status::RUNNING, robot_commands};
  }
}

auto TotalDefenseTactic::assignMarkingTargets(const std::vector<uint8_t> & available_robots)
  -> std::vector<uint8_t>
{
  auto lock = std::lock_guard(markers_mutex);
  auto danger_enemies = getDangerEnemies(world_model);

  for (const auto & [robot, score] : danger_enemies) {
    visualizer->drawDebugLabel(
      robot->pose.pos + Point(0., 0.2), "MarkerScore: " + std::to_string(score));
  }

  if (danger_enemies.size() > available_robots.size()) {
    danger_enemies.resize(available_robots.size());
  }

  RobotList remaining_selectable_robots =
    available_robots |
    ranges::views::transform([&](const auto & id) { return world_model->getOurRobot(id); }) |
    ranges::to<std::vector>();

  std::vector<uint8_t> selected_robots;

  markers.clear();

  for (const auto & [enemy_robot, score] : danger_enemies) {
    // マークする敵ロボットに一番近い味方ロボットを選択
    if (not remaining_selectable_robots.empty()) {
      auto best_marking_robot = ranges::min(
        remaining_selectable_robots, ranges::less{},
        [&](const auto & robot) { return (robot->pose.pos - enemy_robot->pose.pos).norm(); });

      selected_robots.push_back(best_marking_robot->id);
      remaining_selectable_robots.erase(
        ranges::find_if(remaining_selectable_robots, [best_marking_robot](const auto & robot) {
          return robot->id == best_marking_robot->id;
        }));

      // skillを作って設定
      markers.emplace_back(
        std::make_shared<skills::Marker>(
          "total_defense_planner/marker", static_cast<uint8_t>(best_marking_robot->id),
          world_model));

      markers.back()->setParameter("marking_robot_id", enemy_robot->id);
      markers.back()->setParameter("mark_mode", std::string("intercept_pass"));
      markers.back()->setParameter("mark_distance", 0.5);

      visualizer->drawCircle(enemy_robot->pose.pos, 0.3, "black", 10);
      visualizer->drawLine(
        best_marking_robot->pose.pos,
        enemy_robot->pose.pos +
          (enemy_robot->pose.pos - best_marking_robot->pose.pos).normalized() * 0.3,
        "black", 20);
    }
  }
  return selected_robots;
}
}  // namespace crane
