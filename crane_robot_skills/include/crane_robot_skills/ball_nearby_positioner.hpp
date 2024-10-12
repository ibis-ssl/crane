// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_
#define CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class BallNearByPositioner : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit BallNearByPositioner(RobotCommandWrapperBase::SharedPtr & base)
  : SkillBase("BallNearByPositioner", base)
  {
    // このロボットのインデックス
    setParameter("current_robot_index", 0);
    setParameter("total_robot_number", 1);
    // 整列ポリシー（arc/straight）
    setParameter("line_policy", std::string("arc"));
    // ボールの位置決めポリシー（goal/pass）
    setParameter("positioning_policy", std::string("goal"));
    // 整列距離
    setParameter("robot_interval", 0.3);
    setParameter("margin_distance", 0.6);
  }

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    auto situation = world_model()->play_situation.getSituationCommandID();
    double distance_from_ball = [&]() {
      switch (situation) {
        case crane_msgs::msg::PlaySituation::THEIR_DIRECT_FREE:
          return 0.5;
        case crane_msgs::msg::PlaySituation::THEIR_INDIRECT_FREE:
          return 0.5;
        case crane_msgs::msg::PlaySituation::STOP:
          return 0.5;
        case crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT:
          return 0.5;
        default:
          return 0.0;
      }
    }();

    distance_from_ball += getParameter<double>("margin_distance");
    double normalized_offset =
      (getParameter<int>("total_robot_number") - getParameter<int>("current_robot_index") - 1) / 2.;
    double offset = normalized_offset * getParameter<double>("robot_interval");
    Point base_position =
      world_model()->ball.pos +
      [&](const std::string & policy) {
        if (policy == "goal") {
          return (world_model()->getOurGoalCenter() - world_model()->ball.pos).normalized();
        } else if (policy == "pass") {
          // 2番目に近いロボット
          auto theirs = world_model()->theirs.getAvailableRobots();
          if (theirs.size() > 2) {
            auto nearest_robot =
              world_model()->getNearestRobotWithDistanceFromPoint(world_model()->ball.pos, theirs);
            theirs.erase(
              std::remove_if(
                theirs.begin(), theirs.end(),
                [&](const auto & r) { return r->id == nearest_robot.first->id; }),
              theirs.end());
            auto second_nearest_robot =
              world_model()->getNearestRobotWithDistanceFromPoint(world_model()->ball.pos, theirs);
            return (second_nearest_robot.first->pose.pos - world_model()->ball.pos).normalized();
          } else {
            throw std::runtime_error(
              "[BallNearByPositioner] 「positioning policy: "
              "pass」の計算には少なくとも2台の敵ロボットが必要です");
          }
        } else {
          throw std::runtime_error(
            "[BallNearByPositioner] "
            "予期しないパラメータ「positioning_policy」が入力されています: " +
            policy);
        }
      }(getParameter<std::string>("positioning_policy")) *
        distance_from_ball;

    Point target_position = [&](const std::string & policy) -> Point {
      Vector2 ball_to_base = (base_position - world_model()->ball.pos);
      if (policy == "arc") {
        double base_angle = getAngle(ball_to_base);
        // r x theta = interval
        // theta = interval / r
        double angle_interval = getParameter<double>("robot_interval") / distance_from_ball;
        return world_model()->ball.pos +
               getNormVec(base_angle + normalized_offset * angle_interval) * distance_from_ball;
      } else if (policy == "straight") {
        return ball_to_base + getVerticalVec(ball_to_base.normalized()) * offset;
      } else {
        throw std::runtime_error(
          "[BallNearByPositioner] 予期しないパラメータ「line_policy」が入力されています: " +
          policy);
      }
    }(getParameter<std::string>("line_policy"));

    command.setTargetPosition(target_position).lookAtBall();
    return Status::RUNNING;
  }

  void print(std::ostream & os) const override { os << "[BallNearByPositioner]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_
