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
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane::skills
{
enum class AttackerState {
  ENTRY_POINT,
  FORCED_PASS,
  REDIRECT_GOAL_KICK,
  GOAL_KICK,
  STANDARD_PASS,
  LOW_CHANCE_GOAL_KICK,
  MOVE_BALL_TO_OPPONENT_HALF,
  RECEIVE_BALL,
  THROUGH,
  FINAL_GUARD,
};
class Attacker : public SkillBaseWithState<AttackerState, RobotCommandWrapperPosition>
{
public:
  explicit Attacker(RobotCommandWrapperBase::SharedPtr & base);

  void print(std::ostream & os) const override { os << "[Attacker] "; }

  std::shared_ptr<RobotInfo> selectPassReceiver();

  std::optional<uint8_t> pass_receiver_id = std::nullopt;

  Point & kick_target;

  int & forced_pass_receiver_id;

  Kick kick_skill;

  GoalKick goal_kick_skill;

  Receive receive_skill;

  std::optional<Point> goal_front_dance_target = std::nullopt;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__ATTACKER_HPP_
