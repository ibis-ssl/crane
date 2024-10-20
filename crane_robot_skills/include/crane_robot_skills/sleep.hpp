// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SLEEP_HPP_
#define CRANE_ROBOT_SKILLS__SLEEP_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class Sleep : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit Sleep(RobotCommandWrapperBase::SharedPtr & base);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & os) const override;

  double getRestTime() const;

  void reset() { is_started = false; }

  bool & is_started;

  std::chrono::time_point<std::chrono::steady_clock> start_time;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SLEEP_HPP_
