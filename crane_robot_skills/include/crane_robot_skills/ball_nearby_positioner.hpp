// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_
#define CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_

#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class BallNearByPositioner : public SkillBase
{
public:
  template <typename... Args>
  explicit BallNearByPositioner(Args &&... args)
  : SkillBase("ball_positioner", std::forward<Args>(args)...)
  {
    // このロボットのインデックス
    setParameter("current_robot_index", 0);
    setParameter("total_robot_number", 1);
    // 整列ポリシー（arc/straight）
    setParameter("line_policy", std::string("arc"));
    // ボールの位置決めポリシー（goal/pass/auto）
    setParameter("positioning_policy", std::string("auto"));
    // 整列距離
    setParameter("robot_interval", 0.3);
    setParameter("margin_distance", 0.8);
    // ボール以外のターゲット
    setParameter("alternative_target_mode", false);
    setParameter("alternative_target", Point(0, 0));
  }

  auto update() -> Status override;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__BALL_NEARBY_POSITIONER_HPP_
