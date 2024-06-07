// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__TURN_AROUND_POINT_HPP_
#define CRANE_ROBOT_SKILLS__TURN_AROUND_POINT_HPP_

#include <algorithm>
#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
/**
 * 点を中心に回転する
 * 目標角度は目標点から見たロボットの角度
 */
class TurnAroundPoint : public SkillBase<>
{
public:
  explicit TurnAroundPoint(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  void setTargetPoint(const Point & target_point)
  {
    setParameter("target_x", target_point.x());
    setParameter("target_y", target_point.y());
  }

  void setTargetAngle(double target_angle) { setParameter("target_angle", target_angle); }

  double current_target_angle;

  // 周回する円弧の半径。マイナスで初期化してあとから設定する。
  double target_distance = -1.0;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__TURN_AROUND_POINT_HPP_
