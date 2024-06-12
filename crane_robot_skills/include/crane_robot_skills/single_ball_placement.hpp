// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
#define CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

#include "get_ball_contact.hpp"
#include "go_over_ball.hpp"
#include "move_with_ball.hpp"
#include "robot_command_as_skill.hpp"
#include "sleep.hpp"

namespace crane::skills
{
enum class SingleBallPlacementStates {
  PULL_BACK_FROM_EDGE_PREPARE,
  PULL_BACK_FROM_EDGE_TOUCH,
  PULL_BACK_FROM_EDGE_PULL,
  GO_OVER_BALL,
  CONTACT_BALL,
  MOVE_TO_TARGET,
  PLACE_BALL,
  SLEEP,
  LEAVE_BALL,
};

class SingleBallPlacement : public SkillBase<SingleBallPlacementStates>
{
private:
  std::shared_ptr<GoOverBall> go_over_ball;

  std::shared_ptr<GetBallContact> get_ball_contact;

  std::shared_ptr<MoveWithBall> move_with_ball;

  std::shared_ptr<Sleep> sleep = nullptr;

  std::shared_ptr<CmdSetTargetPosition> set_target_position;

  Status skill_status = Status::RUNNING;

  std::optional<Point> pull_back_target;

public:
  explicit SingleBallPlacement(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  void print(std::ostream & os) const override;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
