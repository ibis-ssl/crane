// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__TURN_AROUND_POINT_HPP_
#define CRANE_ROBOT_SKILLS__TURN_AROUND_POINT_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane
{
/**
 * 点を中心に回転する
 * 目標角度は目標点から見たロボットの角度
 */
class TurnAroundPoint : public SkillBase<>
{
public:
  explicit TurnAroundPoint(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("TurnAroundPoint", id, world_model, DefaultStates::DEFAULT)
  {
    setParameter("target_x", 0.0);
    setParameter("target_y", 0.0);
    setParameter("target_angle", 0.0);
    // 半径方向の制御ゲイン
    setParameter("dr_p_gain", 30.0);
    setParameter("max_velocity", 0.5);
    setParameter("max_turn_omega", M_PI_4);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        Point target_point(getParameter<double>("target_x"), getParameter<double>("target_y"));
        double target_angle = getParameter<double>("target_angle");
        if (target_distance < 0.0) {
          // 初回のみ実行時のロボットとの距離を計算して、円弧の半径とする
          target_distance = (robot->pose.pos - target_point).norm();
          current_target_angle = getAngle(robot->pose.pos - target_point);
          auto max_velocity = getParameter<double>("max_velocity");
          auto max_turn_omega = getParameter<double>("max_turn_omega");
          if (target_distance * max_turn_omega > max_velocity) {
            max_velocity = target_distance * max_turn_omega;
          } else {
            max_turn_omega = max_velocity / target_distance;
          }
        }

        if (std::abs(getAngleDiff(getAngle(robot->pose.pos - target_point), target_angle)) < 0.1) {
          command.stopHere();
          return SkillBase::Status::SUCCESS;
        } else {
          // 円弧を描いて移動する

          double current_angle = getAngle(robot->pose.pos - target_point);
          double angle_diff = getAngleDiff(target_angle, current_angle);

          // 半径方向の距離のズレ
          double dr = (robot->pose.pos - target_point).norm() - target_distance;
          std::cout << "dr: " << dr << std::endl;
          std::cout << "target_distance: " << target_distance << std::endl;
          auto max_velocity = getParameter<double>("max_velocity");
          // 円弧を描くような速度
          Velocity velocity = ((target_point - robot->pose.pos).normalized() *
                               (dr * getParameter<double>("dr_p_gain"))) +
                              getNormVec(current_angle + std::copysign(M_PI_2, angle_diff)) *
                                std::min(max_velocity, std::abs(angle_diff * 0.6));
          command.setVelocity(velocity);

          //              current_target_angle += std::copysign(max_turn_omega / 30.0f, angle_diff);
          //              command.setTargetPosition(target_point + getNormVec(current_target_angle) * target_distance);

          // 中心点の方を向く
          command.setTargetTheta(normalizeAngle(current_angle + M_PI));
          return SkillBase::Status::RUNNING;
        }
      });
  }

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
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__TURN_AROUND_POINT_HPP_
