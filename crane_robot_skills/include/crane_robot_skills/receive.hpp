// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__RECEIVE_HPP_
#define CRANE_ROBOT_SKILLS__RECEIVE_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class Receive : public SkillBase<>
{
public:
  explicit Receive(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<>("Receive", id, wm, DefaultStates::DEFAULT)
  {
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        // ボールラインに乗る（ポリシー：最近傍・MaxSlack・MinSlack）
        // 受け取った瞬間にやること（ドリブルしてトラップ・リダイレクト（角度・キック力指定））
        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[Receive]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__RECEIVE_HPP_
