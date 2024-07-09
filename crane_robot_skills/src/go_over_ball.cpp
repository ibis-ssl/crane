// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/go_over_ball.hpp>

namespace crane::skills
{
GoOverBall::GoOverBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("GoOverBall", id, wm, DefaultStates::DEFAULT)
{
  setParameter("next_target_x", 0.0);
  setParameter("next_target_y", 0.0);
  setParameter("margin", 0.5);
  setParameter("reach_threshold", 0.05);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not has_started) {
        Point next_target{
          getParameter<double>("next_target_x"), getParameter<double>("next_target_y")};
        Vector2 r =
          (world_model->ball.pos - next_target).normalized() * getParameter<double>("margin");
        final_target_pos = world_model->ball.pos + r;
        intermediate_target_pos = std::make_pair(
          world_model->ball.pos + getVerticalVec(r), world_model->ball.pos - getVerticalVec(r));
        has_started = true;
      }

      auto cmd = std::dynamic_pointer_cast<RobotCommandWrapperPosition>(command);

      cmd->lookAtBallFrom(final_target_pos);

      auto final_distance = (robot->pose.pos - final_target_pos).norm();
      auto [intermediate_distance, intermediate_point] = [&]() {
        auto d1 = (robot->pose.pos - intermediate_target_pos.first).norm();
        auto d2 = (robot->pose.pos - intermediate_target_pos.second).norm();
        if (d1 < d2) {
          return std::make_pair(d1, intermediate_target_pos.first);
        } else {
          return std::make_pair(d2, intermediate_target_pos.second);
        }
      }();

      if (intermediate_distance < final_distance && not has_passed_intermediate_target) {
        cmd->setTargetPosition(intermediate_point);
        if (intermediate_distance < getParameter<double>("reach_threshold")) {
          std::cout << "Reached intermediate target" << std::endl;
          has_passed_intermediate_target = true;
        }
      } else {
        cmd->setTargetPosition(final_target_pos);
      }

      if (final_distance < getParameter<double>("reach_threshold")) {
        return Status::SUCCESS;
      } else {
        return Status::RUNNING;
      }
    });
}

void GoOverBall::print(std::ostream & out) const
{
  out << "[GoOverBall] ";
  if (
    has_passed_intermediate_target &&
    (robot->pose.pos - final_target_pos).norm() > getParameter<double>("reach_threshold")) {
    out << "中間地点へ向かっています";
  } else {
    out << "最終地点へ向かっています, 距離　" << (robot->pose.pos - final_target_pos).norm();
  }
}
}  // namespace crane::skills
