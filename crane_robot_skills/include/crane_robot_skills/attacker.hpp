// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__ATTACKER_HPP_
#define CRANE_ROBOT_SKILLS__ATTACKER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
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
  RECEIVE,
  KICK,
  FINAL_GUARD,
};

class Attacker : public SkillBaseWithState<AttackerState>
{
public:
  template <typename... Args>
  explicit Attacker(Args &&... args)
  : SkillBaseWithState<AttackerState>("Attacker", std::forward<Args>(args)...),
    kick_target(getContextReference<Point>("kick_target")),
    forced_pass_receiver_id(getContextReference<int>("forced_pass_receiver")),
    kick_skill(command),
    goal_kick_skill(command),
    receive_skill(command)
  {
    initialize();
  }

  void initialize();

  void print(std::ostream & os) const override { os << "[Attacker] "; }

  void printTextOnRobot(std::string s)
  {
    visualizer->text()
      .position(robot()->pose.pos + Vector2(0., 0.5))
      .text(s)
      .fontSize(50)
      .fill("white")
      .textAnchor("middle")
      .build();
  }

  std::shared_ptr<RobotInfo> selectPassReceiver();

  std::optional<uint8_t> pass_receiver_id = std::nullopt;

  Point & kick_target;

  int & forced_pass_receiver_id;

  Kick kick_skill;

  GoalKick goal_kick_skill;

  Receive receive_skill;

  std::optional<Point> goal_front_dance_target = std::nullopt;

  struct OverDribbleInfo
  {
    bool detected = false;
    Point previous_position;
    double distance = 0.0;

    auto update(const Point & current_position, const Point & ball_position) -> void;
  } over_dribble;

private:
  void configurePassKick(const Point & target, Kick & kick_skill);

  bool shouldUseChipKick(const Point & target);

  double evaluateGoalAngle(const Point & position);

  bool isPassBlocked(const Point & target);

  double calculatePassScore(const Point & target);
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__ATTACKER_HPP_
