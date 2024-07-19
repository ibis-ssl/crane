// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/get_ball_contact.hpp>

namespace crane::skills
{
GetBallContact::GetBallContact(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase("GetBallContact", id, wm),
  last_contact_point(getContextReference<Point>("last_contact_point"))
{
  setParameter("min_contact_duration", 0.5);
  setParameter("dribble_power", 0.5);
}

Status GetBallContact::update(const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  if (
    robot->ball_contact.getContactDuration() >
    std::chrono::duration<double>(getParameter<double>("min_contact_duration"))) {
    return Status::SUCCESS;
  } else {
    double distance = (robot->pose.pos - world_model->ball.pos).norm();

    double target_distance = std::max(distance - 0.1, 0.0);

    auto approach_vec = getApproachNormVec();
    command->setDribblerTargetPosition(world_model->ball.pos + approach_vec * 0.05);
    command->setTargetTheta(getAngle(world_model->ball.pos - robot->pose.pos));
    command->dribble(getParameter<double>("dribble_power"));
    command->disableBallAvoidance();
    return Status::RUNNING;
  }
}

void GetBallContact::print(std::ostream & out) const
{
  out << "[GetBallContact] ";
  auto contact_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(robot->ball_contact.getContactDuration())
      .count();
  if (contact_duration > 0) {
    out << "contacted: " << contact_duration << "ms";
  } else {
    out << "ball distance: " << (robot->pose.pos - world_model->ball.pos).norm();
  }
}

Vector2 GetBallContact::getApproachNormVec()
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
}  // namespace crane::skills
