// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__ATTACKER_HPP_
#define CRANE_ROBOT_SKILLS__ATTACKER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/interval.hpp>
#include <crane_robot_skills/goal_kick.hpp>
#include <crane_robot_skills/kick.hpp>
#include <crane_robot_skills/receive.hpp>
#include <crane_robot_skills/redirect.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <crane_robot_skills/steal_ball_vel.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane::skills
{
enum class AttackerState {
  ENTRY_POINT,
  FORCED_PASS,
  CUT_THEIR_PASS,
  STEAL_BALL,
  REDIRECT_GOAL_KICK,
  GOAL_KICK,
  CLEARING_KICK,
  STANDARD_PASS,
  LOW_CHANCE_GOAL_KICK,
  MOVE_BALL_TO_OPPONENT_HALF,
  RECEIVE_BALL,
  GO_TO_BALL,
  THROUGH,
  STOP,
};
class Attacker : public SkillBaseWithState<AttackerState, RobotCommandWrapperPosition>
{
public:
  explicit Attacker(RobotCommandWrapperBase::SharedPtr & base);

  void print(std::ostream & os) const override { os << "[Attacker] "; }

  std::shared_ptr<RobotInfo> selectPassReceiver();

  Point & kick_target;

  int & forced_pass_receiver_id;

  int & forced_pass_phase;

  Kick kick_skill;

  GoalKick goal_kick_skill;

  Receive receive_skill;

  Redirect redirect_skill;

  StealBallVel steal_ball_skill;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__ATTACKER_HPP_
