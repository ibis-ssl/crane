// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
  Goalie() {}

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    auto ball = world_model->ball.pos;

    Point goal_l, goal_r;
    goal_l << -world_model->field_size.x() * 0.5f, 0.5f;
    goal_r << -world_model->field_size.x() * 0.5f, -0.5f;
    Segment goal_line(goal_r, goal_l);

    Point goal_center;
    goal_center << -world_model->field_size.x() * 0.5f + 0.1f, 0.0f;

    Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
    std::vector<Point> intersections;

    // check shoot
    bg::intersection(ball_line, goal_line, intersections);
    if (not intersections.empty()) {
    }

    if (ball.y() > 0) {
      bg::intersection(ball_line, seg_l, intersections);
    } else {
      bg::intersection(ball_line, seg_r, intersections);
    }
    if (intersections.empty()) {
      if (ball.y() > 0) {
        bg::intersection(first_threat_line, seg_l, intersections);
      } else {
        bg::intersection(first_threat_line, seg_r, intersections);
      }
    }
    if (intersections.empty()) {
      return Status::RUNNING;
    }

    robot.builder->setTargetPos(intersections.front(), false);

    return Status::RUNNING;
  }
};
#endif  // CRANE_BT_EXECUTOR__SKILL__GOALIE_HPP_
