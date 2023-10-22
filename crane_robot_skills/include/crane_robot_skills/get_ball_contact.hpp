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
  explicit GetBallContact(
    uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("get_ball_contact", id, world_model, DefaultStates::DEFAULT)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        using std::chrono::seconds;
        if(robot->ball_contact.getContactDuration() > seconds(MINIMUM_CONTACT_DURATION)){
          return SkillBase::Status::SUCCESS;
        }
        robot->ball_contact.getContactDuration()
        return SkillBase::Status::RUNNING;
      });
  }
private:
  std::optional<builtin_interfaces::msg::Time> last_contact_start_time;
  builtin_interfaces::msg::Time last_contact_time;
  Point last_contact_point;

  constexpr static double MINIMUM_CONTACT_DURATION = 0.5;
};
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__GET_BALL_CONTACT_HPP_
