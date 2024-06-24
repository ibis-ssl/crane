// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GOALIE_HPP_
#define CRANE_ROBOT_SKILLS__GOALIE_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <vector>

namespace crane::skills
{
class Goalie : public SkillBase<>
{
public:
  explicit Goalie(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  void emitBallFromPenaltyArea(
    crane::RobotCommandWrapper::SharedPtr & command,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer);

  void inplay(
    crane::RobotCommandWrapper::SharedPtr & command, bool enable_emit,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer);

  void print(std::ostream & os) const override { os << "[Goalie] " << phase; }

  std::string phase;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GOALIE_HPP_
