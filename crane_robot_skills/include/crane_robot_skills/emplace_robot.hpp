// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__EMPLACE_ROBOT_HPP_
#define CRANE_ROBOT_SKILLS__EMPLACE_ROBOT_HPP_

#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class EmplaceRobot : public SkillBase
{
public:
  template <typename... Args>
  explicit EmplaceRobot(Args &&... args) : SkillBase("emplace_robot", std::forward<Args>(args)...)
  {
    // このロボットのインデックス
    setParameter("current_robot_index", 0);
    setParameter("total_robot_number", 1);

    // yが+の位置に整列
    setParameter("emplace_line_positive", true);

    // 整列距離
    setParameter("robot_interval", 0.3);
    // ボールとの距離
    setParameter("margin_distance", 0.5);
    setParameter("max_speed", 1.5);
  }

  Status update() override;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__EMPLACE_ROBOT_HPP_
