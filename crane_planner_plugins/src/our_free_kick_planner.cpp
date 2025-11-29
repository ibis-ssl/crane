// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/our_free_kick_planner.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
OurDirectFreeKickPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  for (auto & command : other_robots) {
    command->stopHere();
    robot_commands.push_back(command->getMsg());
  }
  if (kicker) {
    if (!fake_over) {
      kicker->lookAtBall();
      double x = world_model->ball().pos.x();
      if (x > 0) {
        x -= 0.2;
      } else {
        x += 0.2;
      }
      Point target;
      target << x, world_model->ball().pos.y();
      kicker->setTargetPosition(target);
      if (++fake_count > 30 && kicker->getRobot()->getDistance(target) < 0.1) {
        fake_over = true;
      }
    } else {
      auto [best_angle, goal_angle_width] =
        world_model->getLargestGoalAngleRangeFromPoint(world_model->ball().pos);
      Point best_pass_target = world_model->ball().pos + getNormVec(best_angle) * 0.3;

      // シュートの隙がないときは仲間へパス
      if (goal_angle_width < 0.07) {
        auto available_robots = world_model->ours().getAvailableRobots(kicker->getRobot()->id);
        auto our_robots = available_robots | ranges::views::filter([&](const auto & robot) {
                            auto role = cached_prev_roles.find(robot->id);
                            if (role == cached_prev_roles.end()) {
                              return true;  // roleが見つからない場合は含める
                            }
                            // defenderとキーパー以外を選択
                            return role->second.planner_name != "defender" &&
                                   role->second.planner_name.find("goalie") == std::string::npos;
                          }) |
                          ranges::to<std::vector>();

        if (auto nearest_robot = world_model->getNearestRobotWithDistanceFromPoint(
              world_model->ball().pos, our_robots);
            nearest_robot.has_value()) {
          best_pass_target = nearest_robot->robot->pose.pos;
        }
        //      if((world_model->ball().pos - world_model->getOurGoalCenter()).norm()
        //      < (world_model->ball().pos - world_model->getTheirGoalCenter().norm()) {
        //
        //      }
        // ディフェンダーにしかパスをせず、非常に危なっかしいのでとりあえず真ん中にけるモードを作成
        best_pass_target << 0, 0;
      }

      // 経由ポイント

      Point intermediate_point =
        world_model->ball().pos + (world_model->ball().pos - best_pass_target).normalized() * 0.2;

      double dot = (kicker->getRobot()->pose.pos - world_model->ball().pos)
                     .normalized()
                     .dot((world_model->ball().pos - best_pass_target).normalized());
      double target_theta = getAngle(best_pass_target - world_model->ball().pos);
      kicker->setTargetTheta(target_theta);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (
        dot < 0.75 || std::abs(getAngleDiff(target_theta, kicker->getRobot()->pose.theta)) > 0.1) {
        kicker->setTargetPosition(intermediate_point);
      } else {
        kicker->setTargetPosition(world_model->ball().pos);
        kicker->disableBallAvoidance();

        double pass_line_to_enemy = [&]() {
          Segment line{world_model->ball().pos, best_pass_target};
          auto enemies = world_model->theirs().getAvailableRobots();
          auto distances_view = enemies | ranges::views::transform([&](const auto & robot) {
                                  return bg::distance(robot->pose.pos, line);
                                });
          return ranges::empty(distances_view) ? std::numeric_limits<double>::max()
                                               : ranges::min(distances_view);
        }();

        // 敵が遮っている場合はチップキック
        if (pass_line_to_enemy < 0.4) {
          kicker->kickWithChip(1.0);
        } else {
          kicker->kickStraight(0.7);
        }
      }
    }

    double max_vel = std::min(4.0, kicker->getRobot()->getDistance(world_model->ball().pos) + 0.5);
    kicker->setMaxVelocity("OurDirectFreeKickPlanner", max_vel);

    robot_commands.push_back(kicker->getMsg());
  }
  return {Status::RUNNING, robot_commands};
}
auto OurDirectFreeKickPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  cached_prev_roles = prev_roles;

  auto robots_sorted = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [&](const std::shared_ptr<RobotInfo> & robot) {
      // ボールに近いほうが先頭
      return 100. / robot->getDistance(world_model->ball().pos);
    },
    prev_roles);
  // ゴールキーパーはキッカーに含めない(ロボットがキーパーのみの場合は除く)
  if (robots_sorted.size() > 1 && robots_sorted.front() == world_model->getOurGoalieId()) {
    robots_sorted.erase(robots_sorted.begin());
  }

  if (not robots_sorted.empty()) {
    // 一番ボールに近いロボットがキッカー
    kicker = std::make_shared<RobotCommandWrapper>(
      "our_free_kick_planner/kicker", robots_sorted.front(), world_model);
  } else {
    return {};
  }
  if (robots_sorted.size() > 1) {
    for (auto it = robots_sorted.begin() + 1; it != robots_sorted.end(); it++) {
      other_robots.emplace_back(
        std::make_shared<RobotCommandWrapper>("our_free_kick_planner/other", *it, world_model));
    }
  }
  return robots_sorted;
}
}  // namespace crane
