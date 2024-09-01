// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/rotate.hpp>

namespace crane::skills
{
Rotate::Rotate(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("Rotate", base),is_runnning(false)
{
  setParameter("target_theta", 0.0);
}

Status Rotate::update(
  [[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  double target_theta = getParameter<double>("target_theta");

  if (is_runnning && (0.0 == target_theta)) {
    command.stopHere();
    return Status::SUCCESS;
  } else {
    if (false == is_runnning && (0.0 != target_theta)){
      is_runnning = true;
    }
    
    // Eigen::Vector2d velocity;
    // velocity.theta = target_theta;
    // command.setVelocity(velocity);
    return Status::RUNNING;
  }
}

void Rotate::print(std::ostream& os) const
{
  os << "target_theta: " << getParameter<double>("target_theta");
}

}  // namespace crane::skills
