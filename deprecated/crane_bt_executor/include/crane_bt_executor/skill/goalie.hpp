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

#ifndef CRANE_BT_EXECUTOR__SKILL__GOALIE_HPP_
#define CRANE_BT_EXECUTOR__SKILL__GOALIE_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/target.hpp"

class Goalie : public Composite
{
public:
  Goalie()
  {
  }

  Status run(std::shared_ptr<WorldModelWrapper> world_model, RobotIO robot) override
  {
    auto ball = world_model->ball.pos;
    Point goal_front;
    goal_front << -world_model->field_size.x() * 0.5f + 0.3f, 0.0f;
    Point goal_l, goal_r;
    goal_l << -world_model->field_size.x() * 0.5f, 0.5f;
    goal_r << -world_model->field_size.x() * 0.5f, -0.5f;
    Segment seg_l(goal_front, goal_l);
    Segment seg_r(goal_front, goal_r);

    Point goal_center;
    goal_center << -world_model->field_size.x() * 0.5f + 0.1f, 0.0f;
    Segment first_threat_line(goal_center, ball);
    Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
    std::vector<Point> intersections;

    if (ball.y() > 0)
    {
      bg::intersection(ball_line, seg_l, intersections);
    }
    else
    {
      bg::intersection(ball_line, seg_r, intersections);
    }
    if (intersections.empty())
    {
      if (ball.y() > 0)
      {
        bg::intersection(first_threat_line, seg_l, intersections);
      }
      else
      {
        bg::intersection(first_threat_line, seg_r, intersections);
      }
    }
    if (intersections.empty())
    {
      return Status::RUNNING;
    }

    robot.builder->setTargetPos(intersections.front(), false);

    return Status::RUNNING;
  }
};
#endif  // CRANE_BT_EXECUTOR__SKILL__GOALIE_HPP_
