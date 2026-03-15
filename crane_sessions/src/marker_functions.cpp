// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/math/constants/constants.hpp>
#include <crane_geometry/geometry_operations.hpp>
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
  bool assign_remaining, const std::string & mark_mode,
  const std::unordered_map<uint8_t, uint8_t> & prev_assignments) -> MarkingResult
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

  const bool is_save_goal = (mark_mode == "save_goal");
  const Point reference = is_save_goal ? world_model->getOurGoalCenter() : world_model->ball().pos;

  for (const auto & [enemy_robot, score] : danger_enemies) {
    if (remaining_robots.empty()) {
      break;
    }

    // マーキングセグメントを事前計算（割当コストと可視化で共用）
    constexpr double mark_dist = 0.5;
    constexpr double max_mark_dist = 1.5;
    auto dir = (reference - enemy_robot->pose.pos).normalized();
    Point near_end = enemy_robot->pose.pos + dir * mark_dist;
    Point far_end = enemy_robot->pose.pos + dir * max_mark_dist;
    if (is_save_goal) {
      constexpr double penalty_offset = 0.15;
      Segment enemy_to_goal{enemy_robot->pose.pos, world_model->getOurGoalCenter()};
      auto intersection =
        world_model->getIntersectionOurPenaltyArea(enemy_to_goal, penalty_offset, penalty_offset);
      if (intersection) {
        double dist_to_boundary = (intersection.value() - enemy_robot->pose.pos).norm();
        double clamped_max = std::max(mark_dist, dist_to_boundary - 0.05);
        far_end = enemy_robot->pose.pos + dir * std::min(max_mark_dist, clamped_max);
        near_end = enemy_robot->pose.pos + dir * std::min(mark_dist, clamped_max);
      }
    }
    Segment marking_segment{near_end, far_end};

    auto best_robot = ranges::min(remaining_robots, ranges::less{}, [&](const auto & robot) {
      // 割当コスト: 敵に対するマーキングセグメントへの距離（固定点ではなく線分ベース）
      double dist = getClosestPointAndDistance(robot->pose.pos, marking_segment).distance;
      // ヒステリシス: 前フレームで同じ敵をマークしていたロボットにボーナス
      // (Sumatra DesiredDefendersCalcUtil 参考)
      constexpr double MARKER_HYSTERESIS_BONUS = 0.5;  // 前フレーム割り当て継続ボーナス [m]
      auto it = prev_assignments.find(enemy_robot->id);
      if (it != prev_assignments.end() && it->second == robot->id) {
        dist -= MARKER_HYSTERESIS_BONUS;
      }
      return dist;
    });

    result.enemy_to_marker[enemy_robot->id] = best_robot->id;
    result.selected_robot_ids.push_back(best_robot->id);
    remaining_robots.erase(ranges::find_if(remaining_robots, [&](const auto & robot) {
      return robot->id == best_robot->id;
    }));

    auto marker = std::make_shared<skills::Marker>(
      command_name, static_cast<uint8_t>(best_robot->id), world_model);
    marker->setParameter("marking_robot_id", enemy_robot->id);
    marker->setParameter("mark_mode", mark_mode);
    marker->setParameter("mark_distance", 0.5);
    result.markers.push_back(marker);

    visualizer->drawCircle(enemy_robot->pose.pos, 0.3, "black", 10);
    // マーキングセグメントを可視化（コスト計算と同一セグメント）
    visualizer->drawLine(near_end, far_end, "green", 10);
    visualizer->drawLine(best_robot->pose.pos, enemy_robot->pose.pos, "black", 20);
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
