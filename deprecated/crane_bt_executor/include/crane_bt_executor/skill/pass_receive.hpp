// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__SKILL__PASS_RECEIVE_HPP_
#define CRANE_BT_EXECUTOR__SKILL__PASS_RECEIVE_HPP_

#include <memory>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/tool.hpp"
#include "crane_bt_executor/utils/target.hpp"

class PassReceive : public Composite
{
public:
  explicit PassReceive(TargetModule receive_point) : receive_point_(receive_point)
  {
  }
  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    auto ball = world_model->ball;
    auto pos = robot.info->pose.pos;
    auto receive_pos = receive_point_.getPoint(world_model);

    float ball_vel = ball.vel.dot((pos - ball.pos).normalized());
    if (ball_vel < -0.5f)
    {
      return Status::FAILURE;
    }
    else if (ball_vel > 0.5f)
    {
      ClosestPoint result;
      Segment ball_line(ball.pos, (ball.pos + ball.vel.normalized() * (ball.pos - pos).norm()));
      bg::closest_point(pos, ball_line, result);
      robot.builder->addDribble(0.5f);
      robot.builder->setTargetPos(result.closest_point, false);
      robot.builder->setTargetTheta(tool::getAngle(ball.pos - pos));
    }
    else
    {
      robot.builder->setTargetPos(receive_pos);
      robot.builder->setTargetTheta(tool::getAngle(ball.pos - pos));
    }

    return Status::RUNNING;
  }
  TargetModule receive_point_;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__PASS_RECEIVE_HPP_
