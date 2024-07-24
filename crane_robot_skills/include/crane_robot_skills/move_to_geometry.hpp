// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MOVE_TO_GEOMETRY_HPP_
#define CRANE_ROBOT_SKILLS__MOVE_TO_GEOMETRY_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane::skills
{
template <typename Geometry>
class MoveToGeometry : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit MoveToGeometry(
    uint8_t id, Geometry geometry, const std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase("MoveToGeometry", base), geometry(geometry)
  {
    setParameter("reach_threshold", 0.1);
  }

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override
  {
    if ((robot->pose.pos - getTargetPoint()).norm() < getParameter<double>("reach_threshold")) {
      return Status::SUCCESS;
    } else {
      command.setTargetPosition(getTargetPoint());
      return Status::RUNNING;
    }
  }

  Point getTargetPoint()
  {
    return getClosestPointAndDistance(geometry, robot->pose.pos).closest_point;
  }

  void updateGeometry(Geometry geometry) { this->geometry = geometry; }

protected:
  Geometry geometry;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__MOVE_TO_GEOMETRY_HPP_
