// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
#define CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
/**
 * ボールを持って移動する
 * 前提：GetBallContactが成功していること
 */
class MoveWithBall : public SkillBase<>
{
public:
  explicit MoveWithBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("MoveWithBall", id, world_model, DefaultStates::DEFAULT)
  {
    setParameter("target_x", 0.0);
    setParameter("target_y", 0.0);
    setParameter("target_theta", 0.0);
    setParameter("reach_threshold", 0.1);
    setParameter("reach_angle_threshold", 0.1);
    setParameter("ball_lost_timeout", 2.0);
    setParameter("dribble_power", 0.1);
    // これ以上の角度をずれて進むと停止する
    setParameter("moving_direction_tolerance", 0.3);
    // この時間以上ボールが離れたら停止する
    setParameter("max_contact_lost_time", 0.3);
    // ドリブル時の目標位置の設置距離
    setParameter("dribble_target_horizon", 0.2);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        Pose2D target_pose;
        target_pose.pos.x() = getParameter<double>("target_x");
        target_pose.pos.y() = getParameter<double>("target_y");
        target_pose.theta = getParameter<double>("target_theta");
        if (not robot->ball_contact.findPastContact(getParameter<double>("ball_lost_timeout"))) {
          // ボールが離れたら失敗
          return Status::FAILURE;
        } else if (
          (robot->pose.pos - target_pose.pos).norm() < getParameter<double>("reach_threshold")) {
          if (
            std::abs(getAngleDiff(robot->pose.theta, target_pose.theta)) <
            getParameter<double>("reach_angle_threshold")) {
            command.setTargetPosition(target_pose.pos, target_pose.theta);
            command.dribble(0.0);
            // ターゲットに到着したら成功
            return Status::SUCCESS;
          } else {
            // ターゲットに到着しているが、角度がずれている場合
            phase = "最終角度に調整しています";
            double angle_diff = getAngleDiff(robot->pose.theta, target_pose.theta);
            if (std::abs(angle_diff) < getParameter<double>("reach_angle_threshold") * 2) {
              command.setTargetPosition(target_pose.pos, target_pose.theta);
            } else {
              // 0.4 = MP/8
              double angle_target = robot->pose.theta - std::copysign(0.4, angle_diff);
              command.setTargetPosition(target_pose.pos, angle_target);
            }
            return Status::RUNNING;
          }
        } else {
          phase = "目標位置に向かっています";
          command.setTargetPosition(getTargetPoint(target_pose));
          // 目標姿勢の角度ではなく、目標位置の方向を向いて進む
          command.setTargetTheta(getTargetAngle(target_pose));
          command.dribble(getParameter<double>("dribble_power"));
          return Status::RUNNING;
        }
      });
  }

  Point getTargetPoint(const Pose2D & target_pose)
  {
    // 正しい方向でドリブルできている場合だけ前進
    if (
      getAngleDiff(robot->pose.theta, getTargetAngle(target_pose)) <
      getParameter<double>("moving_direction_tolerance")) {
      if (robot->ball_contact.findPastContact(getParameter<double>("max_contact_lost_time"))) {
        return robot->pose.pos + (target_pose.pos - robot->pose.pos).normalized() *
                                   getParameter<double>("dribble_target_horizon");
      }
    }
    return robot->pose.pos;
  }

  double getTargetAngle(const Pose2D & target_pose)
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

  void setTargetPose(const Pose2D & target_pose)
  {
    setParameter("target_x", target_pose.pos.x());
    setParameter("target_y", target_pose.pos.y());
    setParameter("target_theta", target_pose.theta);
  }

  void print(std::ostream & out) const override
  {
    out << "[MoveWithBall] " << phase;
    out << ", target: " << getParameter<double>("target_x") << ", "
        << getParameter<double>("target_y");
  }

  std::string phase;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
