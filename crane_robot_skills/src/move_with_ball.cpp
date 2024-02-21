// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/move_with_ball.hpp>

namespace crane::skills
{
MoveWithBall::MoveWithBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("MoveWithBall", id, wm, DefaultStates::DEFAULT)
{
  setParameter("target_x", 0.0);
  setParameter("target_y", 0.0);
  //    setParameter("target_theta", 0.0);
  setParameter("reach_threshold", 0.1);
  //    setParameter("reach_angle_threshold", 0.1);
  setParameter("ball_lost_timeout", 2.0);
  setParameter("dribble_power", 0.1);
  // これ以上の角度をずれて進むと停止する
  setParameter("moving_direction_tolerance", 0.3);
  // この時間以上ボールが離れたら停止する
  setParameter("max_contact_lost_time", 0.3);
  // ドリブル時の目標位置の設置距離
  setParameter("dribble_target_horizon", 0.2);
  setParameter("ball_stabilizing_time", 0.5);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](
      const std::shared_ptr<WorldModelWrapper> & world_model,
      const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
      ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      command.setMaxVelocity(0.5);
      auto target_pos = parseTargetPoint();
      command.setDribblerTargetPosition(target_pos);
      target_theta = getAngle(target_pos - robot->pose.pos);
      command.setTargetTheta(target_theta);
      // 開始時にボールに接していることが前提にある
      if (not robot->ball_contact.findPastContact(getParameter<double>("ball_lost_timeout"))) {
        // ボールが離れたら失敗
        return Status::FAILURE;
      } else if (robot->getDistance(target_pos) < getParameter<double>("reach_threshold")) {
        // ターゲットに到着してしばらく待ったら成功
        if (not ball_stabilizing_start_time) {
          ball_stabilizing_start_time = std::chrono::steady_clock::now();
        }
        if (
          getElapsedSec(*ball_stabilizing_start_time) >
          getParameter<double>("ball_stabilizing_time")) {
          command.dribble(0.0);
          return Status::SUCCESS;
        } else {
          return Status::RUNNING;
        }
      } else {
        phase = "目標位置に向かっています";
        command.setTargetPosition(getTargetPoint(target_pos));
        // 目標姿勢の角度ではなく、目標位置の方向を向いて進む
        command.setTargetTheta(target_theta);
        command.dribble(getParameter<double>("dribble_power"));
        return Status::RUNNING;
      }
    });
}
}  // namespace crane::skills
