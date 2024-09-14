// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/rotate.hpp>
#include <chrono>

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
      last_time = std::chrono::system_clock::now();
      return Status::RUNNING;
    }
    // auto now_time = std::chrono::system_clock::now();
    // double vel_omega = robot()->vel.omega;
    // double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_time-last_time).count();
    // command.setVelocity();
    return Status::RUNNING;
  }
}

void Rotate::print(std::ostream& os) const
{
  os << "target_theta: " << getParameter<double>("target_theta");
}

}  // namespace crane::skills
