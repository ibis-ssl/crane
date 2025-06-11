// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__FORWARD_HPP_
#define CRANE_ROBOT_SKILLS__FORWARD_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

#include "skill_base.hpp"

namespace crane::skills
{
class Forward : public SkillBase
{
public:
  template <typename... Args>
  explicit Forward(Args &&... args) : SkillBase("Forward", std::forward<Args>(args)...)
  {
    // 敵陣側
    setParameter("front_point", Point(0, 0));
    // 自陣側
    setParameter("back_point", Point(0, 0));

    setParameter("max_ball_distance", 5.0);
    setParameter("max_vel", 1.0);
  }

  auto update() -> Status override;

  void print(std::ostream & os) const override { os << "[Forward]"; }

  VisualizerMessageBuilder::SharedPtr planner_visualizer;
};

}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__FORWARD_HPP_
