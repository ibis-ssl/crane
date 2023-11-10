// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MOVE_TO_GEOMETRY_HPP_
#define CRANE_ROBOT_SKILLS__MOVE_TO_GEOMETRY_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane
{
template <typename Geometry>
class MoveToGeometry : public SkillBase<>
{
public:
  explicit MoveToGeometry(
    uint8_t id, Geometry geometry, std::shared_ptr<WorldModelWrapper> & world_model,
    const double threshold = 0.1)
  : SkillBase<>("move_to_geometry", id, world_model, DefaultStates::DEFAULT), geometry(geometry)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this, threshold](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot,
        crane::RobotCommandWrapper & command) -> SkillBase::Status {
        if ((robot->pose.pos - getTargetPoint()).norm() < threshold) {
          return SkillBase::Status::SUCCESS;
        } else {
          command.setTargetPosition(getTargetPoint());
          return SkillBase::Status::RUNNING;
        }
      });
  }

  Point getTargetPoint()
  {
    ClosestPoint result;
    bg::closest_point(geometry, robot->pose.pos, result);
    return result.closest_point;
  }

  void updateGeometry(Geometry geometry) { this->geometry = geometry; }

protected:
  Geometry geometry;
};
}  // namespace crane
#endif  // CRANE_ROBOT_SKILLS__MOVE_TO_GEOMETRY_HPP_