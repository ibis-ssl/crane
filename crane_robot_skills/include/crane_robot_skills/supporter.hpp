// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SUPPORTER_HPP_
#define CRANE_ROBOT_SKILLS__SUPPORTER_HPP_

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
class Supporter : public SkillBase<>
{
public:
  explicit Marker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<>("Supporter", id, wm, DefaultStates::DEFAULT)
  {
    setParameter("sample_param", 0.);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        auto sample_param = getParameter<double>("sample_param");

        // TODO(HansRobo): 実装
        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[Supporter]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SUPPORTER_HPP_
