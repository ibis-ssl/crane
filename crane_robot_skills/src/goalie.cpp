// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/goalie.hpp>

namespace crane::skills
{
Goalie::Goalie(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("Goalie", id, wm, DefaultStates::DEFAULT)
{
  setParameter("run_inplay", true);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
      ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      auto situation = world_model->play_situation.getSituationCommandID();
      if (getParameter<bool>("run_inplay")) {
        situation = crane_msgs::msg::PlaySituation::INPLAY;
      }
      switch (situation) {
        case crane_msgs::msg::PlaySituation::HALT:
          phase = "HALT, stop here";
          command.stopHere();
          break;
        case crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION:
          [[fallthrough]];
        case crane_msgs::msg::PlaySituation::THEIR_PENALTY_START:
          phase = "ペナルティキック";
          inplay(command, false);
          break;
        default:
          inplay(command, true);
          break;
      }
      return Status::RUNNING;
    });
}
}  // namespace crane::skills
