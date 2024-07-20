// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GET_BALL_CONTACT_HPP_
#define CRANE_ROBOT_SKILLS__GET_BALL_CONTACT_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class GetBallContact : public SkillBase
{
public:
  explicit GetBallContact(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & out) const override;

private:
  Vector2 getApproachNormVec();

  std::optional<builtin_interfaces::msg::Time> last_contact_start_time;

  builtin_interfaces::msg::Time last_contact_time;

  Point & last_contact_point;

  //  double target_distance = 0.0;
};

}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GET_BALL_CONTACT_HPP_
