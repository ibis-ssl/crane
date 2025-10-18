// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICK_HPP_
#define CRANE_ROBOT_SKILLS__KICK_HPP_

#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/receive.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
enum class KickState {
  ENTRY_POINT,
  AROUND_BALL_AND_KICK,
  REDIRECT_KICK,
  POSITIVE_REDIRECT_KICK,
};

class Kick : public SkillBaseWithState<KickState>
{
private:
  Receive receive_skill;

public:
  template <typename... Args>
  explicit Kick(Args &&... args)
  : SkillBaseWithState<KickState>("Kick", std::forward<Args>(args)...),
    receive_skill(command),
    phase(getContextReference<std::string>("phase"))
  {
    initialize();
  }

  /**
   * @brief ボールがフィールドから出る位置を取得
   * @param offset 出る位置を内側にずらすオフセット
   * @return ボールが出る位置
   */
  auto getBallExitPointFromField(const double offset = 0.3) -> Point;

  void print(std::ostream & os) const override { os << "[Kick]"; }

  std::string & phase;

private:
  void initialize();

  void kickWithChip();

  void kickStraight();
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICK_HPP_
