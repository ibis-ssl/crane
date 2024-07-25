// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MARKER_HPP_
#define CRANE_ROBOT_SKILLS__MARKER_HPP_

#include <algorithm>
#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane::skills
{
class Marker : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit Marker(RobotCommandWrapperBase::SharedPtr & base);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & os) const override { os << "[Marker]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__MARKER_HPP_
