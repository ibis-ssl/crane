// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__PENALTY_KICK_HPP_
#define CRANE_ROBOT_SKILLS__PENALTY_KICK_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>

namespace crane::skills
{
enum class PenaltyKickState {
  PREPARE,
  KICK,
  DONE,
};

class PenaltyKick : public SkillBase<PenaltyKickState>
{
private:
  std::optional<Point> start_ball_point = std::nullopt;

public:
  explicit PenaltyKick(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<PenaltyKickState>("PenaltyKick", id, wm, PenaltyKickState::PREPARE)
  {
    setParameter("start_from_kick", false);
    setParameter("prepare_margin", 0.6);
    addStateFunction(
      PenaltyKickState::PREPARE, [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        Point target = world_model->ball.pos;
        auto margin = getParameter<double>("prepare_margin");
        target.x() += world_model->getOurGoalCenter().x() > 0 ? margin : -margin;
        command->setTargetPosition(target);
        command->lookAtBall();
        return Status::RUNNING;
      });

    addTransition(PenaltyKickState::PREPARE, PenaltyKickState::KICK, [this]() {
      if (getParameter<bool>("start_from_kick")) {
        return true;
      } else {
        return world_model->play_situation.getSituationCommandID() ==
               crane_msgs::msg::PlaySituation::OUR_PENALTY_START;
      }
    });
    addStateFunction(
      PenaltyKickState::KICK, [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not start_ball_point) {
          start_ball_point = world_model->ball.pos;
        }

        auto [best_target, goal_angle_width] = getBestShootTargetWithWidth();
        visualizer->addPoint(best_target.x(), best_target.y(), 1, "red", 1.0, "best_target");

        // 経由ポイント
        Point intermediate_point =
          world_model->ball.pos + (world_model->ball.pos - best_target).normalized() * 0.2;
        visualizer->addPoint(
          intermediate_point.x(), intermediate_point.y(), 1, "red", 1.0, "intermediate_point");

        double dot = (robot->pose.pos - world_model->ball.pos)
                       .normalized()
                       .dot((world_model->ball.pos - best_target).normalized());
        double target_theta = getAngle(best_target - world_model->ball.pos);
        // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
        if (dot < 0.95 || std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.05) {
          command->setTargetPosition(intermediate_point);
          command->enableCollisionAvoidance();
        } else {
          command->setTargetPosition(world_model->ball.pos);
          command->kickStraight(0.7).disableCollisionAvoidance();
          command->enableCollisionAvoidance();
          command->disableBallAvoidance();
        }

        command->setTargetTheta(getAngle(best_target - world_model->ball.pos));
        return Status::RUNNING;
      });

    addTransition(PenaltyKickState::KICK, PenaltyKickState::DONE, [this]() {
      return world_model->isDefenseArea(world_model->ball.pos);
    });

    addStateFunction(
      PenaltyKickState::DONE, [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        command->stopHere();
        return Status::RUNNING;
      });
  }

  // SimpleAttackerからコピー
  auto getBestShootTargetWithWidth() -> std::pair<Point, double>
  {
    const auto & ball = world_model->ball.pos;

    Interval goal_range;

    auto goal_posts = world_model->getTheirGoalPosts();
    goal_range.append(getAngle(goal_posts.first - ball), getAngle(goal_posts.second - ball));

    for (auto & enemy : world_model->theirs.getAvailableRobots()) {
      double distance = enemy->getDistance(ball);
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

  void print(std::ostream & os) const override
  {
    os << "[Idle] stop_by_position: " << getParameter<bool>("stop_by_position") ? "true" : "false";
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__PENALTY_KICK_HPP_
