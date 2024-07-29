// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__REDIRECT_HPP_
#define CRANE_ROBOT_SKILLS__REDIRECT_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class Redirect : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit Redirect(RobotCommandWrapperBase::SharedPtr & base) : SkillBase("Redirect", base)
  {
    setParameter("kick_power", 0.3);
    setParameter("kick_with_chip", false);
    setParameter("redirect_target", Point(0, 0));
    // min_slack, max_slack, closest
    setParameter("policy", "closest");
  }

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    auto interception_point = getInterceptionPoint();
    auto target_angle = getRobotAngle(
      world_model()->ball.pos, world_model()->ball.vel, interception_point,
      getParameter<Point>("redirect_target"));
    command.setDribblerTargetPosition(interception_point, target_angle);
    if (getParameter<bool>("kick_with_chip")) {
      command.kickWithChip(getParameter<double>("kick_power"));
    } else {
      command.kickStraight(getParameter<double>("kick_power"));
    }
    return Status::RUNNING;
  }

  void print(std::ostream & os) const override { os << "[Receive]"; }

  Point getInterceptionPoint() const
  {
    std::string policy = getParameter<std::string>("policy");
    if (policy.ends_with("slack")) {
      auto [max_slack_point, max_slack] = world_model()->getMinMaxSlackInterceptPoint();
      if (policy == "max_slack" && max_slack_point) {
        return max_slack_point.value();
      } else if (policy == "min_slack" && max_slack_point) {
        return max_slack_point.value();
      }
      return world_model()->ball.pos;
    } else if (policy == "closest") {
      Segment ball_line(
        world_model()->ball.pos,
        (world_model()->ball.pos + world_model()->ball.vel.normalized() * 10.0));
      auto result = getClosestPointAndDistance(robot()->pose.pos, ball_line);
      return result.closest_point;
    } else {
      throw std::runtime_error("Invalid policy for Receive::getInterceptionPoint: " + policy);
    }
  }

  double getRobotAngle(
    Point ball_pos, Velocity ball_vel, Point interception_point, Point redirect_target) const
  {
    Vector2 to_ball = ball_pos - interception_point;
    Vector2 to_target = redirect_target - interception_point;
    // ボールとターゲットの角度の中間角を求める（暫定実装）
    return getIntermediateAngle(getAngle(to_ball), getAngle(to_target));
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__REDIRECT_HPP_
