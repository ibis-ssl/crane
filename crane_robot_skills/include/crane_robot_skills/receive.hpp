// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__RECEIVE_HPP_
#define CRANE_ROBOT_SKILLS__RECEIVE_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class Receive : public SkillBase
{
public:
  template <typename... Args>
  explicit Receive(Args &&... args) : SkillBase("Receive", std::forward<Args>(args)...)
  {
    setParameter("dribble_power", 0.3);
    setParameter("enable_software_bumper", true);
    setParameter("software_bumper_start_time", 0.5);
    // min_slack, max_slack, closest
    setParameter("policy", std::string("closest"));
    setParameter("enable_active_receive", true);
    setParameter("enable_redirect", false);
    setParameter("redirect_target", Point(0, 0));
    setParameter("redirect_kick_power", 0.3);
    setParameter("robot_acc_for_prediction", 2.5);
    setParameter("robot_max_vel_for_prediction", 5.0);
  }

  Status update() override;

  void print(std::ostream & os) const override { os << "[Receive]"; }

  Point getInterceptionPoint() const;
};

}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__RECEIVE_HPP_
