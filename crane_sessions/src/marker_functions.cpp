// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/math/constants/constants.hpp>
#include <crane_sessions/marker_functions.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/functional/comparisons.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <ranges>

namespace crane
{
auto getDangerEnemies(const WorldModelWrapper::SharedPtr & world_model)
  -> std::vector<std::pair<std::shared_ptr<RobotInfo>, double>>
{
  RobotList defense_robots;
  defense_robots.emplace_back(world_model->getOurRobot(world_model->getOurGoalieId()));

  const auto their_robots = world_model->theirs().robotsWhere().available().get();
  auto robots_and_scores =
    their_robots | ranges::views::filter([&](const auto & robot) {
      if (not world_model->point_checker.isInOurHalf(robot->pose.pos)) {
        // 相手コートにいる敵ロボットはマークしない
        return false;
      } else if (robot->getDistance(world_model->ball().pos) < 1.0) {
        // ボールに近い敵ロボットはマークしない
        return false;
      } else {
        return true;
      }
    }) |
    ranges::views::transform([&](const auto & robot) {
      auto [_, angle_width] = world_model->getLargestGoalAngleRangeFromPoint(
        robot->pose.pos, world_model->getOurGoalPosts(), defense_robots);
      double x_diff = std::abs(world_model->getOurGoalCenter().x() - robot->pose.pos.x());
      double score = [&]() {
        double angle_deg_width = angle_width * boost::math::constants::radian<double>();
        if (angle_deg_width > 15.0) {
          return angle_deg_width;
        } else {
          return angle_deg_width + 10.0 - std::clamp(x_diff * 2.0, 1.0, 10.0);
        }
      }();
      return std::make_pair(robot, score);
    }) |
    ranges::to<std::vector>();

  // 高スコアが前
  std::ranges::sort(robots_and_scores, [](auto & a, auto & b) { return a.second > b.second; });
  return robots_and_scores;
}

auto assignMarkersToEnemies(
  const std::vector<uint8_t> & available_robot_ids,
  const WorldModelWrapper::SharedPtr & world_model,
  const VisualizerMessageBuilder::SharedPtr & visualizer, const std::string & command_name,
  bool assign_remaining) -> MarkingResult
{
  MarkingResult result;

  auto danger_enemies = getDangerEnemies(world_model);

  for (const auto & [robot, score] : danger_enemies) {
    visualizer->drawDebugLabel(
      robot->pose.pos + Point(0., 0.2), "MarkerScore: " + std::to_string(score));
  }

  if (danger_enemies.size() > available_robot_ids.size()) {
    danger_enemies.resize(available_robot_ids.size());
  }

  RobotList remaining_robots =
    available_robot_ids |
    ranges::views::transform([&](const auto & id) { return world_model->getOurRobot(id); }) |
    ranges::to<std::vector>();

  for (const auto & [enemy_robot, score] : danger_enemies) {
    if (remaining_robots.empty()) {
      break;
    }
    auto best_robot = ranges::min(remaining_robots, ranges::less{}, [&](const auto & robot) {
      return (robot->pose.pos - enemy_robot->pose.pos).norm();
    });

    result.selected_robot_ids.push_back(best_robot->id);
    remaining_robots.erase(ranges::find_if(remaining_robots, [&](const auto & robot) {
      return robot->id == best_robot->id;
    }));

    auto marker = std::make_shared<skills::Marker>(
      command_name, static_cast<uint8_t>(best_robot->id), world_model);
    marker->setParameter("marking_robot_id", enemy_robot->id);
    marker->setParameter("mark_mode", std::string("intercept_pass"));
    marker->setParameter("mark_distance", 0.5);
    result.markers.push_back(marker);

    visualizer->drawCircle(enemy_robot->pose.pos, 0.3, "black", 10);
    visualizer->drawLine(
      best_robot->pose.pos,
      enemy_robot->pose.pos + (enemy_robot->pose.pos - best_robot->pose.pos).normalized() * 0.3,
      "black", 20);
  }

  // assign_remaining=trueのとき、マーク対象のいない残余ロボットにもMarkerを生成
  if (assign_remaining) {
    for (const auto & robot : remaining_robots) {
      result.markers.push_back(
        std::make_shared<skills::Marker>(
          command_name, static_cast<uint8_t>(robot->id), world_model));
      result.selected_robot_ids.push_back(robot->id);
    }
  }

  return result;
}

}  // namespace crane
