// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SLEEP_HPP_
#define CRANE_ROBOT_SKILLS__SLEEP_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
class Sleep : public SkillBase<>
{
public:
  explicit Sleep(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("Sleep", id, world_model, DefaultStates::DEFAULT)
  {
    setParameter("duration", 0.0);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not is_started) {
          start_time = std::chrono::steady_clock::now();
          is_started = true;
        }

        auto elapsed_time =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time);
        if (elapsed_time.count() > getParameter<double>("duration")) {
          return Status::SUCCESS;
        } else {
          return Status::RUNNING;
        }
      });
  }

  void print(std::ostream & os) const override
  {
    os << "[Sleep] 残り時間: " << getRestTime() << "秒";
  }

  double getRestTime() const
  {
    auto elapsed_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time);
    return getParameter<double>("duration") - elapsed_time.count();
  }
  bool is_started = false;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SLEEP_HPP_
