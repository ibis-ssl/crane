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
    //    setParameter("target", Point(0, 0));
    //    setParameter("kick_power", 0.5f);
    //    setParameter("chip_kick", false);
    //    setParameter("with_dribble", false);
    //    setParameter("dribble_power", 0.3f);
    //    setParameter("dot_threshold", 0.95f);
    //    setParameter("angle_threshold", 0.1f);
    //    setParameter("around_interval", 0.3f);
    // min_slack, max_slack, none, closest
    setParameter("policy", "max_slack");
  }

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    auto interception_point = getInterceptionPoint();

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
