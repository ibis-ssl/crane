// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__ATTACKER_HPP_
#define CRANE_ROBOT_SKILLS__ATTACKER_HPP_

#include <chrono>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_robot_skills/goal_kick.hpp>
#include <crane_robot_skills/kick_old.hpp>
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
  RECEIVE,
  KICK,
};

class Attacker : public SkillBaseWithState
{
private:
  static std::string getStateName(int s);

public:
  template <typename... Args>
  explicit Attacker(Args &&... args)
  : SkillBaseWithState(
      static_cast<int>(AttackerState::ENTRY_POINT), &Attacker::getStateName, "attacker",
      std::forward<Args>(args)...),
    kick_skill(command),
    goal_kick_skill(command),
    receive_skill(command)
  {
    over_dribble_check_enabled_ = true;
    initialize();
  }

  void initialize();

  AttackerState getCurrentState() const
  {
    return static_cast<AttackerState>(SkillBaseWithState::getCurrentState());
  }

  void printTextOnRobot(std::string s)
  {
    visualizer->drawCenteredLabel(robot()->pose.pos + Vector2(0., 0.5), s, "white", 50);
  }

  std::optional<uint8_t> pass_receiver_id = std::nullopt;

  Point kick_target{Point::Zero()};

  int forced_pass_receiver_id = -1;

  KickOld kick_skill;

  GoalKick goal_kick_skill;

  Receive receive_skill;

protected:
  void onPostUpdate() override;

private:
  void configurePassKick(const Point & target, KickOld & kick_skill);

  // KICK状態の進捗タイムアウト用
  std::chrono::steady_clock::time_point kick_state_entry_time{};
  bool in_kick_state = false;

  bool shouldUseChipKick(const Point & target);

  double evaluateGoalAngle(const Point & position);

  double calculatePassScore(const Point & target);
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__ATTACKER_HPP_
