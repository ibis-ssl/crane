// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__ROTATE_HPP_
#define CRANE_ROBOT_SKILLS__ROTATE_HPP_

#include <algorithm>
#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
/**
 * その場で回転する
 */
class Rotate : public SkillBase<RobotCommandWrapperSimpleVelocity>
{
public:
  explicit Rotate(RobotCommandWrapperBase::SharedPtr & base);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;
  void print(std::ostream & os) const override;

  void setRotateSpeed(double target_vel_theta)
  {
    setParameter("target_theta", target_vel_theta);
  }

private:
  double last_theta;
  bool is_runnning;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__ROTATE_HPP_