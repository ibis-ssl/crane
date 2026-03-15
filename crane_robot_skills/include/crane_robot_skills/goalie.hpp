// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GOALIE_HPP_
#define CRANE_ROBOT_SKILLS__GOALIE_HPP_

#include <chrono>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/kick.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <vector>

namespace crane::skills
{
class Goalie : public SkillBase
{
public:
  template <typename... Args>
  explicit Goalie(Args &&... args)
  : SkillBase("Goalie", std::forward<Args>(args)...), kick_skill(command)
  {
    initialize();
  }

  Status update() override;

  void emitBallFromPenaltyArea();

  void inplay(bool enable_emit);

  std::string phase;

  Kick kick_skill;

  std::optional<Point> prev_wait_point;

  std::optional<std::chrono::steady_clock::time_point> ball_in_penalty_since_;

private:
  void initialize();

  auto clampXToGoalLine(const Point & position, double margin) const -> Point;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GOALIE_HPP_
