// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/get_ball_contact.hpp>

namespace crane::skills
{
GetBallContact::GetBallContact(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model)
: SkillBase<>("GetBallContact", id, world_model, DefaultStates::DEFAULT)
{
  setParameter("min_contact_duration", 0.5);
  setParameter("dribble_power", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
      ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      // 規定時間以上接していたらOK
      std::cout << "ContactDuration: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                     robot->ball_contact.getContactDuration())
                     .count()
                << std::endl;
      // TODO: ロボットからのフィードバック情報を使う
      if (
        robot->ball_contact.getContactDuration() >
        std::chrono::duration<double>(getParameter<double>("min_contact_duration"))) {
        return Status::SUCCESS;
      } else {
        double distance = (robot->pose.pos - world_model->ball.pos).norm();

        double target_distance = std::max(distance - 0.1, 0.0);

        auto approach_vec = getApproachNormVec();
        command.setDribblerTargetPosition(world_model->ball.pos);
        command.setTargetTheta(getAngle(world_model->ball.pos - robot->pose.pos));
        command.dribble(getParameter<double>("dribble_power"));
        return Status::RUNNING;
      }
    });
}
}  // namespace crane::skills
