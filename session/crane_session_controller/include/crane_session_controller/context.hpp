// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__CONTEXT_HPP_
#define CRANE_SESSION_CONTROLLER__CONTEXT_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <memory>
#include <variant>
#include <vector>

namespace crane
{

enum class ActionStage {
  UNASSIGNED,
  PRE_STAGE,
  ON_STAGE,
  POST_STAGE,
};

struct ActionBase
{
  uint8_t robot_id;

  std::shared_ptr<crane::RobotCommandWrapper> command = nullptr;

  virtual auto update(const WorldModelWrapper::SharedPtr & world_model, const ActionStage stage)
    -> crane_msgs::msg::RobotCommand = 0;
};

/**
PassActionとは具体的にどのアクションを指すのか？
=> ここでPassActionとは、次のロボットへパスをするためにキックをするActionのこととする。
連続PassActionがあれば、以下のように遷移する

1. ロボット1の近くにボールがある
  a. PassAction1: ON_STAGE(ボールにアプローチしてキックする)
  b. PassAction2: PRE_STAGE(ボールを受け取るポイントに移動する)
2. ロボット1がボールをキックした
  a. PassAction1: POST_STAGE(キックが終わったので、ON_STAGEを終える)
  b. PassAction2: ON_STAGE(ボールを受け取るポイントに移動する)

 */
struct PassAction : public ActionBase
{
  Point pass_target_point;

  Point pass_estimated_receive_point;

  auto update(const WorldModelWrapper::SharedPtr & world_model, const ActionStage stage)
    -> crane_msgs::msg::RobotCommand override
  {
    if (not command) {
      command = std::make_shared<crane::RobotCommandWrapper>(robot_id, world_model);
    }

    switch (stage) {
      case ActionStage::PRE_STAGE: {
        command->setTargetPosition(pass_estimated_receive_point)
          .lookAtBallFrom(pass_estimated_receive_point);
        break;
      }
      case ActionStage::ON_STAGE: {
        // 前のActionでボールがキックされた瞬間にON_STAGEになるのでボールが受け取れる位置に移動する
        command->kickStraight(0.5)
          .setTargetTheta([&]() {
            // TODO(HansRobo): 力積などを考慮して適切な角度にする
            double robot_to_target = getAngle(pass_target_point - command->robot->pose.pos);
            double robot_to_ball = getAngle(world_model->ball.pos - command->robot->pose.pos);
            return getIntermediateAngle(robot_to_target, robot_to_ball);
          }())
          .setDribblerTargetPosition([&]() {
            Segment ball_line{
              world_model->ball.pos, world_model->ball.pos + world_model->ball.vel * 4.0};
            return getClosestPointAndDistance(ball_line, command->robot->kicker_center())
              .closest_point;
          }())
          .disableBallAvoidance();
        break;
      }
      case ActionStage::POST_STAGE: {
      }
    }
    return command->getMsg();
  }
};

// 今の所はpass_target_pointをゴールに設定するとShootActionになる
using ShootAction = PassAction;

struct DribbleAction : public ActionBase
{
  Point target_point;

  auto update(const WorldModelWrapper::SharedPtr & world_model, const ActionStage stage)
    -> crane_msgs::msg::RobotCommand override
  {
    if (not command) {
      command = std::make_shared<crane::RobotCommandWrapper>(robot_id, world_model);
    }
    return command->getMsg();
  }
};

struct Context
{
  std::vector<std::variant<PassAction, ShootAction, DribbleAction>> actions;
};
}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__CONTEXT_HPP_
