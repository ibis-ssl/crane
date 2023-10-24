// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GET_BALL_CONTACT_HPP_
#define CRANE_ROBOT_SKILLS__GET_BALL_CONTACT_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane
{
class GetBallContact : public SkillBase<>
{
public:
  explicit GetBallContact(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("get_ball_contact", id, world_model, DefaultStates::DEFAULT)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        // 規定時間以上接していたらOK
        if (
          robot->ball_contact.getContactDuration() >
          std::chrono::duration<double>(MINIMUM_CONTACT_DURATION)) {
          return SkillBase::Status::SUCCESS;
        } else {
          double distance = (robot->pose.pos - world_model->ball.pos).norm();

          double target_distance = std::max(distance - 0.1, 0.0);

          auto approach_vec = getApproachNormVec();
          command.setDribblerTargetPosition(world_model->ball.pos - approach_vec * target_distance);
          command.setTargetTheta(getAngle(approach_vec));
          return SkillBase::Status::RUNNING;
        }
      });
  }

private:
  Vector2 getApproachNormVec()
  {
    // if robot is far away from ball, the approach angle is the angle to the ball from robot
    // if robot is close to ball, the approach angle is robot angle
    // and, the approach angle is interpolated between these two cases
    constexpr double FAR_THRESHOLD = 3.5;
    constexpr double NEAR_THRESHOLD = 0.5;

    Vector2 far_vec{(robot->pose.pos - world_model->ball.pos).normalized()};
    Vector2 near_vec{cos(robot->pose.theta), sin(robot->pose.theta)};

    double distance = (robot->pose.pos - world_model->ball.pos).norm();

    return [&]() {
      if (distance > FAR_THRESHOLD) {
        return far_vec;
      } else if (distance < NEAR_THRESHOLD) {
        return near_vec;
      } else {
        double ratio = (distance - NEAR_THRESHOLD) / (FAR_THRESHOLD - NEAR_THRESHOLD);
        return (far_vec * ratio + near_vec * (1.0 - ratio)).normalized();
      }
    }();
  }

  std::optional<builtin_interfaces::msg::Time> last_contact_start_time;
  builtin_interfaces::msg::Time last_contact_time;
  Point last_contact_point;

  //  double target_distance = 0.0;

  constexpr static double MINIMUM_CONTACT_DURATION = 0.5;
};
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__GET_BALL_CONTACT_HPP_
