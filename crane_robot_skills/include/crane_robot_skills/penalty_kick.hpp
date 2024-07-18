// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__PENALTY_KICK_HPP_
#define CRANE_ROBOT_SKILLS__PENALTY_KICK_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_basics/interval.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>

namespace crane::skills
{
enum class PenaltyKickState {
  PREPARE,
  KICK,
  DONE,
};

class PenaltyKick : public SkillBaseWithState<PenaltyKickState>
{
private:
  std::optional<Point> start_ball_point = std::nullopt;

public:
  explicit PenaltyKick(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  void print(std::ostream & os) const override
  {
    os << "[Idle] stop_by_position: " << getParameter<bool>("stop_by_position") ? "true" : "false";
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__PENALTY_KICK_HPP_
