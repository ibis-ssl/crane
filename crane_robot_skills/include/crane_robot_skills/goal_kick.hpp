// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GOAL_KICK_HPP_
#define CRANE_ROBOT_SKILLS__GOAL_KICK_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class GoalKick : public SkillBase<>
{
public:
  explicit GoalKick(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<>("GoalKick", id, wm, DefaultStates::DEFAULT)
  {
    setParameter("キック角度の最低要求精度[deg]", 1.0);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        auto [best_angle, goal_angle_width] =
          world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
        // 隙間のなかで更に良い角度を計算する。
        // キック角度の最低要求精度をオフセットとしてできるだけ端っこを狙う

        Point target = world_model->ball.pos + getNormVec(best_angle) * 0.5;

        {  // この部分はいずれ回り込み＆キックのスキルとして一般化したい
           // (その時は動くボールへの回り込みを含めて)
          // パラメータ候補：キックパワー・dotしきい値・角度しきい値・経由ポイント距離・突撃速度
          double dot =
            (robot->pose.pos - ball_pos).normalized().dot((ball_pos - target).normalized());
          if (
            (dot > 0.95 || (robot->pose.pos - ball_pos).norm() > 0.1) &&
            std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.1) {
            // キック
            command->setTargetPosition(ball_pos + (kick_target - ball_pos).normalized() * 0.5)
              .kickStraight(0.8)
              .disableCollisionAvoidance()
              .enableCollisionAvoidance()
              .disableBallAvoidance();
          } else {
            // 経由ポイントへGO
            Point intermediate_point = ball_pos + (ball_pos - target).normalized() * 0.3;
            command->setTargetPosition(intermediate_point)
              .enableCollisionAvoidance()
              .enableBallAvoidance();
          }
          // 共通コマンド
          command->liftUpDribbler().setTargetTheta(getAngle(target - world_model->ball.pos));
        }

        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[GoalKick] "; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GOAL_KICK_HPP_
