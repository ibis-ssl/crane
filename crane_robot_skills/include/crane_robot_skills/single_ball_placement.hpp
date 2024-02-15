// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
#define CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

#include "get_ball_contact.hpp"
#include "go_over_ball.hpp"
#include "move_with_ball.hpp"
#include "sleep.hpp"

namespace crane::skills
{
enum class SingleBallPlacementStates { GO_OVER_BALL, MOVE_TO_TARGET, PLACE_BALL, SLEEP };

class SingleBallPlacement : public SkillBase<SingleBallPlacementStates>
{
private:
  std::shared_ptr<GoOverBall> go_over_ball;

  std::shared_ptr<GetBallContact> get_ball_contact;

  std::shared_ptr<MoveWithBall> move_with_ball;

  std::shared_ptr<Sleep> sleep = nullptr;

public:
  explicit SingleBallPlacement(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<SingleBallPlacementStates>(
      "SingleBallPlacement", id, world_model, SingleBallPlacementStates::GO_OVER_BALL)
  {
    setParameter("placement_x", 0.);
    setParameter("placement_y", 0.);
    addStateFunction(
      SingleBallPlacementStates::GO_OVER_BALL,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not go_over_ball) {
          go_over_ball = std::make_shared<GoOverBall>(robot->id, world_model);
        }

        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[SingleBallPlacement]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
