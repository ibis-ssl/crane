// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SIMPLE_ATTACKER_HPP_
#define CRANE_ROBOT_SKILLS__SIMPLE_ATTACKER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
class SimpleAttacker : public SkillBase<>
{
public:
  explicit SimpleAttacker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("SimpleAttacker", id, world_model, DefaultStates::DEFAULT)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        auto [best_target, goal_angle_width] = getBestShootTargetWithWidth();

        // シュートの隙がないときは仲間へパス
        if (goal_angle_width < 0.07) {
          auto our_robots = world_model->ours.getAvailableRobots();
          our_robots.erase(
            std::remove_if(
              our_robots.begin(), our_robots.end(),
              [&](const auto & our_robot) { return our_robot->id == robot->id; }),
            our_robots.end());
          auto nearest_robot =
            world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, our_robots);
          best_target = nearest_robot.first->pose.pos;
        }

        // 経由ポイント

        Point intermediate_point =
          world_model->ball.pos + (world_model->ball.pos - best_target).normalized() * 0.2;

        double dot = (robot->pose.pos - world_model->ball.pos)
                       .normalized()
                       .dot((world_model->ball.pos - best_target).normalized());
        double target_theta = getAngle(best_target - world_model->ball.pos);
        // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
        if (dot < 0.95 || std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.05) {
          command.setTargetPosition(intermediate_point);
          command.enableCollisionAvoidance();
        } else {
          command.setTargetPosition(world_model->ball.pos);
          command.kickStraight(0.7).disableCollisionAvoidance();
          command.enableCollisionAvoidance();
          command.disableBallAvoidance();
        }

        command.setTargetTheta(getAngle(best_target - world_model->ball.pos));

        bool is_in_defense = world_model->isDefenseArea(world_model->ball.pos);
        bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

        if ((not is_in_field) or is_in_defense) {
          command.stopHere();
        }
        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override
  {
    os << "[Idle] stop_by_position: " << getParameter<bool>("stop_by_position") ? "true" : "false";
  }

  auto getBestShootTargetWithWidth() -> std::pair<Point, double>
  {
    const auto & ball = world_model->ball.pos;

    Interval goal_range;

    auto goal_posts = world_model->getTheirGoalPosts();
    goal_range.append(getAngle(goal_posts.first - ball), getAngle(goal_posts.second - ball));

    for (auto & enemy : world_model->theirs.robots) {
      double distance = (ball - enemy->pose.pos).norm();
      constexpr double MACHINE_RADIUS = 0.1;

      double center_angle = getAngle(enemy->pose.pos - ball);
      double diff_angle =
        atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

      goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
    }

    auto largest_interval = goal_range.getLargestInterval();

    double target_angle = (largest_interval.first + largest_interval.second) / 2.0;

    return {
      ball + getNormVec(target_angle) * 0.5, largest_interval.second - largest_interval.first};
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SIMPLE_ATTACKER_HPP_
