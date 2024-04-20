// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_
#define CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>

namespace crane::skills
{
enum class StealBallState {
  MOVE_TO_FRONT,
  STEAL,
};
class StealBall : public SkillBase<StealBallState>
{
public:
  explicit StealBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<StealBallState>("StealBall", id, wm, StealBallState::MOVE_TO_FRONT)
  {
    // ボールを奪う方法
    // front: 正面からドリブラーでボールを奪う
    // side: 横から押し出すようにボールを奪う
    setParameter("steal_method", "front");
    addStateFunction(
      StealBallState::MOVE_TO_FRONT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        // ボールの正面に移動
        // 到着判定すると遅くなるので、敵ロボットにボールが隠されていなかったら次に行ってもいいかも
        return Status::RUNNING;
      });
    addStateFunction(
      StealBallState::STEAL,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        command->disableBallAvoidance();
        command->disableCollisionAvoidance();
        const auto method = getParameter<std::string>("steal_method");
        if (method == "front") {
          command->setKickerTagetPosition(world_model->ball.pos);
          command->dribble(0.5);
        } else if (method == "side") {
          command->setKickerTagetPosition(world_model->ball.pos);
          if (robot->getDistance(world_model->ball.pos) < (0.085 + 0.005)) {
            // ロボット半径より近くに来れば急回転して刈り取れる
            command->setTargetTheta(getAngle(world_model->ball.pos - robot->pose.pos) + M_PI / 2);
          } else {
            command->setTargetTheta(getAngle(world_model->ball.pos - robot->pose.pos));
          }
        }
        return Status::RUNNING;
      });
  }

  void print(std::ostream & os) const override { os << "[StealBall]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_
