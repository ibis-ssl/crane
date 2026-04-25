// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICK_OLD_HPP_
#define CRANE_ROBOT_SKILLS__KICK_OLD_HPP_

#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
// 983f026a（2025-09-14）以前の回り込みアルゴリズム（move_direction ベース 2 フェーズ）
enum class KickOldState {
  ENTRY_POINT,
  AROUND_BALL_AND_KICK,
};

class KickOld : public SkillBaseWithState
{
private:
  static std::string getStateName(int s);

public:
  template <typename... Args>
  explicit KickOld(Args &&... args)
  : SkillBaseWithState(
      static_cast<int>(KickOldState::ENTRY_POINT), &KickOld::getStateName, "kick_old",
      std::forward<Args>(args)...)
  {
    initialize();
  }

  auto getBallExitPointFromField(const double offset = 0.3) -> Point;

private:
  void initialize();

  void kickWithChip();

  void kickStraight();
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICK_OLD_HPP_
