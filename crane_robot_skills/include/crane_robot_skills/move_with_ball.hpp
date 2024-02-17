// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
#define CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_

#include <chrono>
#include <crane_geometry/eigen_adapter.hpp>
#include <crane_geometry/time.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <optional>

namespace crane::skills
{
/**
 * ボールを持って移動する
 * 前提：GetBallContactが成功していること
 */

enum class MoveWithBallStates { SUCCESS, RUNNING, FAILURE };
class MoveWithBall : public SkillBase<>
{
public:
  explicit MoveWithBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model);

  Point getTargetPoint(const Point & target_pos)
  {
    // 正しい方向でドリブルできている場合だけ前進
    if (
      getAngleDiff(robot->pose, target_theta) <
      getParameter<double>("moving_direction_tolerance")) {
      if (robot->ball_contact.findPastContact(getParameter<double>("max_contact_lost_time"))) {
        return robot->pose.pos + (target_pos - robot->pose.pos).normalized() *
                                   getParameter<double>("dribble_target_horizon");
      }
    }
    return robot->pose.pos;
  }

  //  double getTargetAngle(const Pose2D & target_pose)
  //  {
  //    auto distance = world_model->getDistanceFromRobot(robot->getID(), target_pose.pos);
  //    auto to_target = getAngle(target_pose.pos - robot->pose.pos);
  //
  //    if (distance > 0.2) {
  //      return to_target;
  //    } else if (distance > 0.1) {
  //      double ratio = (distance - 0.1) / (0.2 - 0.1);
  //      return ratio * to_target + (1.0 - ratio) * target_pose.theta;
  //    } else {
  //      return target_pose.theta;
  //    }
  //  }

  void setTargetPoint(const Point & target_point)
  {
    setParameter("target_x", target_point.x());
    setParameter("target_y", target_point.y());
    //    setParameter("target_theta", target_pose.theta);
  }

  auto parseTargetPoint() const -> Point
  {
    return Point{getParameter<double>("target_x"), getParameter<double>("target_y")};
  }

  void print(std::ostream & out) const override
  {
    out << "[MoveWithBall] " << phase;
    out << ", target: " << getParameter<double>("target_x") << ", "
        << getParameter<double>("target_y");
  }

  std::string phase;

  std::optional<std::chrono::steady_clock::time_point> ball_stabilizing_start_time = std::nullopt;

  double target_theta;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
