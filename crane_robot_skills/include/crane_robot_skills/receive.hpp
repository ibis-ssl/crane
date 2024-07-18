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
class Receive : public SkillBase<>
{
public:
  explicit Receive(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<>("Receive", id, wm, DefaultStates::DEFAULT)
  {
    setParameter("receive_policy", "closest");
    setParameter("next_action", "trap");
    setParameter("dribble_power", 0.5);
    setParameter("redirect_robot_theta", 0.0);
    setParameter("redirect_chip", false);
    setParameter("redirect_kick_power", 0.5);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        // ボールラインに乗る（ポリシー：最近傍・MaxSlack・MinSlack）
        // 受け取った瞬間にやること（ドリブルしてトラップ・リダイレクト（角度・キック力指定））
        auto next_action = getParameter<std::string>("next_action");
        auto dribble_power = getParameter<double>("dribble_power");
        auto receive_point = getReceivePoint();
        if (next_action == "trap") {
          command->setTargetPosition(receive_point)
            .lookAtBallFrom(receive_point)
            .disableBallAvoidance()
            .dribble(dribble_power);
        } else if (next_action == "redirect") {
          command->setTargetTheta(getParameter<double>("redirect_robot_theta"))
            .setDribblerTargetPosition(receive_point)
            .dribble(dribble_power)
            .disableBallAvoidance();

          auto redirect_kick_power = getParameter<double>("redirect_kick_power");
          if (getParameter<bool>("redirect_chip")) {
            command->kickWithChip(redirect_kick_power);
          } else {
            command->kickStraight(redirect_kick_power);
          }
        } else {
          throw std::runtime_error("unexpected string for next_action is detected");
        }

        return Status::RUNNING;
      });
  }

  auto getReceivePoint() -> Point
  {
    auto policy = getParameter<std::string>("receive_policy");
    if (policy == "closest") {
      Segment ball_line(
        world_model->ball.pos,
        (world_model->ball.pos +
         world_model->ball.vel.normalized() * (world_model->ball.pos - robot->pose.pos).norm()));
      auto result = getClosestPointAndDistance(robot->pose.pos, ball_line);
      return result.closest_point;
    } else if (policy == "max_slack") {
      throw std::runtime_error("max_slack for receive_policy is not implemented");
    } else if (policy == "min_slack") {
      throw std::runtime_error("min_slack for receive_policy is not implemented");
    } else {
      throw std::runtime_error("unexpected string for receive_policy is detected");
    }
  }

  void print(std::ostream & os) const override { os << "[Receive]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__RECEIVE_HPP_
