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
  explicit TurnAroundPoint(
    Point point, double angle, uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("turn_around_point", id, world_model, DefaultStates::DEFAULT),
    target_point(point),
    target_angle(angle)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        if (target_distance < 0.0) {
          target_distance = (robot->pose.pos - target_point).norm();
          current_target_angle = getAngle(robot->pose.pos - target_point);
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
          double current_angle = getAngle(robot->pose.pos - target_point);

          double angle_diff = getAngleDiff(target_angle, current_angle);

          double dr = (robot->pose.pos - target_point).norm() - target_distance;
          std::cout << "dr: " << dr << std::endl;
          std::cout << "target_distance: " << target_distance << std::endl;
          Velocity velocity = ((target_point - robot->pose.pos).normalized() * (dr * 30.)) +
                              getNormVec(current_angle + std::copysign(M_PI_2, angle_diff)) *
                                std::min(max_velocity, std::abs(angle_diff * 0.6));
          command.setVelocity(velocity);

          //              current_target_angle += std::copysign(max_turn_omega / 30.0f, angle_diff);
          //              command.setTargetPosition(target_point + getNormVec(current_target_angle) * target_distance);

          command.setTargetTheta(normalizeAngle(current_angle + M_PI));
          return SkillBase::Status::RUNNING;
        }
      });
  }

  Point target_point;

  double target_angle;

  double current_target_angle;

  double target_distance = -1.0;

  double max_turn_omega = M_PI_4;

  double max_velocity = 0.5;
};
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__TURN_AROUND_POINT_HPP_
