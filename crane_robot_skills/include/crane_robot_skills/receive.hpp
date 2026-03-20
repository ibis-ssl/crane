// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__RECEIVE_HPP_
#define CRANE_ROBOT_SKILLS__RECEIVE_HPP_

#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <optional>

namespace crane::skills
{
class Receive : public SkillBase
{
public:
  template <typename... Args>
  explicit Receive(Args &&... args) : SkillBase("receive", std::forward<Args>(args)...)
  {
    setParameter("dribble_power", 0.3);
    setParameter("enable_software_bumper", true);
    setParameter("software_bumper_start_time", 0.5);
    // min_slack, max_slack, closest
    setParameter("policy", std::string("closest"));
    setParameter("enable_active_receive", true);
    setParameter("enable_redirect", false);
    setParameter("redirect_target", Point(0, 0));
    setParameter("redirect_kick_power", 0.3);

    // Visualization toggles
    setParameter("viz_ball_traj", true);
    setParameter("viz_candidates", true);
    setParameter("viz_offset_arrow", true);
    setParameter("viz_redirect_preview", true);
    setParameter("viz_enemy_block", true);
  }

  Status update() override;

  Point getInterceptionPoint() const;

  // 敵割り込み検出・回避関数
  bool isEnemyBlockingPassLine(const Point & interception_point) const;
  Point getInterceptionPointWithEnemyAvoidance() const;

private:
  // 前フレームのインターセプト位置（ジッター抑制用、Sumatra BallInterceptor参考）
  mutable std::optional<Point> prev_interception_point_;
};

}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__RECEIVE_HPP_
