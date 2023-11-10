// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
#define CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane
{
/**
 * ボールを持って移動する
 * 前提：GetBallContactが成功していること
 */
class MoveWithBall : public SkillBase<>
{
public:
  explicit MoveWithBall(Pose2D pose, uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("move_with_ball", id, world_model, DefaultStates::DEFAULT), target_pose(pose)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        if (not robot->ball_contact.findPastContact(2.0)) {
          // ボールが離れたら失敗
          return SkillBase::Status::FAILURE;
        } else if (
          (robot->pose.pos - target_pose.pos).norm() < 0.1 &&
          std::abs(getAngleDiff(robot->pose.theta, target_pose.theta)) < 0.1) {
          command.setTargetPosition(target_pose.pos, target_pose.theta);
          command.dribble(0.5);
          // ターゲットに到着したら成功
          return SkillBase::Status::SUCCESS;
        } else {
          command.setTargetPosition(getTargetPoint());
          command.setTargetTheta(getTargetAngle());
          command.dribble(0.1);
          return SkillBase::Status::RUNNING;
        }
      });
  }

  Point getTargetPoint()
  {
    // 正しい方向でドリブルできている場合だけ前進
    if (getAngleDiff(robot->pose.theta, getTargetAngle()) < 0.3) {
      if (robot->ball_contact.findPastContact(0.5)) {
        return robot->pose.pos + (target_pose.pos - robot->pose.pos).normalized() * 0.2;
      }
    }
    return robot->pose.pos;
  }

  double getTargetAngle()
  {
    auto distance = world_model->getDistanceFromRobot(robot->getID(), target_pose.pos);
    auto to_target = getAngle(target_pose.pos - robot->pose.pos);

    if (distance > 0.2) {
      return to_target;
    } else if (distance > 0.1) {
      double ratio = (distance - 0.1) / (0.2 - 0.1);
      return ratio * to_target + (1.0 - ratio) * target_pose.theta;
    } else {
      return target_pose.theta;
    }
  }

  Pose2D target_pose;
};
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
