// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/our_free_kick_session.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/range/operations.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
OurDirectFreeKickSession::calculatePositionCommand(
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

      // シュートの隙がないときはとりあえず真ん中に蹴る
      if (goal_angle_width < 0.07) {
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
          auto enemies = world_model->theirs().robotsWhere().available().get();
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
    kicker->setMaxVelocity("OurDirectFreeKickSession", max_vel);

    robot_commands.push_back(kicker->getMsg());
  }
  return {Status::RUNNING, robot_commands};
}
}  // namespace crane
