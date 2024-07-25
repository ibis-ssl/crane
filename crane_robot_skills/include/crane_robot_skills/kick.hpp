// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICK_HPP_
#define CRANE_ROBOT_SKILLS__KICK_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class Kick : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit Kick(RobotCommandWrapperBase::SharedPtr & base) : SkillBase("Kick", base)
  {
    setParameter("target_x", 0.0f);
    setParameter("target_y", 0.0f);
    setParameter("kick_power", 0.5f);
    setParameter("dot_threshold", 0.95f);
    setParameter("angle_threshold", 0.1f);
    setParameter("around_interval", 0.3f);
  }

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    {  // この部分はいずれ回り込み＆キックのスキルとして一般化したい
       // (その時は動くボールへの回り込みを含めて)
      // パラメータ候補：キックパワー・dotしきい値・角度しきい値・経由ポイント距離・突撃速度
      Point target;
      target << getParameter<double>("target_x"), getParameter<double>("target_y");
      auto ball_pos = world_model()->ball.pos;
      double dot =
        (robot()->pose.pos - ball_pos).normalized().dot((ball_pos - target).normalized());
      if (
        (dot > getParameter<double>("dot_threshold")) ||
        (robot()->pose.pos - ball_pos).norm() > 0.1 &&
          std::abs(getAngleDiff(getAngle(target - world_model()->ball.pos), robot()->pose.theta)) >
            getParameter<double>("angle_threshold")) {
        // キック
        command.setTargetPosition(ball_pos + (target - ball_pos).normalized() * 0.3)
          .kickStraight(getParameter<double>("kick_power")
          .disableCollisionAvoidance()
          .enableCollisionAvoidance()
          .disableBallAvoidance();
      } else {
        // 経由ポイントへGO
        Point intermediate_point =
          ball_pos + (ball_pos - target).normalized() * getParameter<double>("around_interval");
        command.setTargetPosition(intermediate_point)
          .enableCollisionAvoidance()
          .enableBallAvoidance();
      }
      // 共通コマンド
      command.liftUpDribbler().setTargetTheta(getAngle(target - world_model()->ball.pos));
    }

    return Status::RUNNING;
  }

  void print(std::ostream & os) const override { os << "[Kick]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICK_HPP_
