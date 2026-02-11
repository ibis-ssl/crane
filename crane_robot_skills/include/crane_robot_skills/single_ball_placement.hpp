// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
#define CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_

#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

#include "receive.hpp"
#include "sleep.hpp"

namespace crane::skills
{
enum class SingleBallPlacementStates {
  ENTRY_POINT,
  RECEIVE_BALL,
  PULL_BACK_FROM_EDGE_PREPARE,
  PULL_BACK_FROM_EDGE_TOUCH,
  PULL_BACK_FROM_EDGE_PULL,
  PULL_BACK_FROM_EDGE_OVER_SLEEP,
  PULL_BACK_FROM_EDGE_OVER_LEAVE,
  GO_OVER_BALL,
  PASS_TO_TARGET,
  CONTACT_BALL,
  MOVE_TO_TARGET,
  PLACE_BALL,
  SLEEP,
  LEAVE_BALL,
};

class SingleBallPlacement : public SkillBaseWithState
{
private:
  static std::string getStateName(int s);

  std::shared_ptr<Receive> receive;

  std::shared_ptr<Sleep> sleep = nullptr;

  Status skill_status = Status::RUNNING;

  std::optional<Point> pull_back_target;

  double pull_back_angle;

public:
  template <typename... Args>
  explicit SingleBallPlacement(Args &&... args)
  : SkillBaseWithState(
      static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), &SingleBallPlacement::getStateName,
      "SingleBallPlacement", std::forward<Args>(args)...)
  {
    initialize();
  }

  void initialize();

  SingleBallPlacementStates getCurrentState() const
  {
    return static_cast<SingleBallPlacementStates>(SkillBaseWithState::getCurrentState());
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
