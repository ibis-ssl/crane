// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MARKER_HPP_
#define CRANE_ROBOT_SKILLS__MARKER_HPP_

#include <algorithm>
#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane::skills
{
class Marker : public SkillBase<>
{
public:
  explicit Marker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<>("Marker", id, wm, DefaultStates::DEFAULT)
  {
    setParameter("marking_robot_id", 0);
    setParameter("mark_distance", 0.5);
    setParameter("mark_mode", "save_goal");
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        auto marked_robot = world_model->getTheirRobot(getParameter<int>("marking_robot_id"));
        auto enemy_pos = marked_robot->pose.pos;

        std::string mode = getParameter<std::string>("mark_mode");
        Point marking_point;
        double target_theta;

        if (mode == "save_goal") {
          marking_point = enemy_pos + (world_model->getOurGoalCenter() - enemy_pos).normalized() *
                                        getParameter<double>("mark_distance");
          target_theta = getAngle(enemy_pos - world_model->getOurGoalCenter());
        } else if (mode == "intercept_pass") {
        } else {
          throw std::runtime_error("unknown mark mode");
        }
        command->setTargetPosition(marking_point, target_theta);
        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[Marker]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__MARKER_HPP_
