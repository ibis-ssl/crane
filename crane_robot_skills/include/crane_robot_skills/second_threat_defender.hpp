// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SECOND_THREAT_DEFENDER_HPP_
#define CRANE_ROBOT_SKILLS__SECOND_THREAT_DEFENDER_HPP_

#include <crane_robot_skills/skill_base.hpp>
#include <string>
#include <utility>

namespace crane::skills
{
class SecondThreatDefender : public SkillBase
{
public:
  template <typename... Args>
  explicit SecondThreatDefender(Args &&... args)
  : SkillBase("second_threat_defender", std::forward<Args>(args)...)
  {
    initialize();
  }

  void initialize();

  static auto getDefaultPoint(const WorldModelWrapper::SharedPtr & world_model, const double offset)
    -> Point
  {
    // ボールと反対側にあるゴールの角
    // 左右の判定には getBallSideSign() を使う。生の ball().pos.y() の符号で決めると、
    // ボールがセンターにあるときvisionノイズで毎フレーム反転し、この目標点が左右の角へ
    // 数m飛ぶ。suitability関数（SecondThreatDefenderSession）もこの関数を呼ぶため、
    // 目標位置とロボット選択の両方が同時に振動する。
    return {
      (world_model->fieldSize().x() * 0.5 - world_model->getDefenseHeight() - offset) *
        world_model->getOurSideSign(),
      (world_model->getDefenseWidth() * 0.5 + offset) * -world_model->getBallSideSign()};
  }

  Status update() override;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SECOND_THREAT_DEFENDER_HPP_
