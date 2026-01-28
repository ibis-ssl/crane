// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/math/constants/constants.hpp>
#include <crane_sessions/marker_functions.hpp>
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

}  // namespace crane
