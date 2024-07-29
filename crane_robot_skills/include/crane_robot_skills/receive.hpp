// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__RECEIVE_HPP_
#define CRANE_ROBOT_SKILLS__RECEIVE_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class Receive : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit Receive(RobotCommandWrapperBase::SharedPtr & base) : SkillBase("Receive", base)
  {
    setParameter("dribble_power", 0.3);
    setParameter("enable_software_bumper", true);
    setParameter("software_bumper_start_time", 0.5);
    // min_slack, max_slack, closest
    setParameter("policy", std::string("closest"));
  }

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    auto offset = [&]() -> Point {
      if (getParameter<bool>("enable_software_bumper")) {
        // ボール到着まで残り<software_bumper_start_time>秒になったら、ボール速度方向に少し加速して衝撃を和らげる
        double ball_speed = world_model()->ball.vel.norm();
        if (
          robot()->getDistance(world_model()->ball.pos) <
          ball_speed * getParameter<double>("software_bumper_start_time")) {
          // ボールから逃げ切らないようにするため、速度の0.5倍に制限
          command.setMaxVelocity(ball_speed * 0.5);
          // ボール速度方向に速度の0.5倍だけオフセット（1m/sで近づいていたら0.5m）
          return world_model()->ball.vel.normalized() * (world_model()->ball.vel.norm() * 0.5);
        } else {
          return Point(0, 0);
        }
      } else {
        return Point(0, 0);
      }
    }();
    auto interception_point = getInterceptionPoint() + offset;
    command.lookAtBallFrom(interception_point)
      .setDribblerTargetPosition(interception_point)
      .dribble(getParameter<double>("dribble_power"))
      .disableBallAvoidance();

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
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__RECEIVE_HPP_
