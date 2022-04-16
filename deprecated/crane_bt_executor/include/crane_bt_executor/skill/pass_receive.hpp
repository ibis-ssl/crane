// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
  Status run(std::shared_ptr<WorldModelWrapper> world_model, RobotIO robot) override
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
