// Copyright (c) 2020 ibis-ssl
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

#ifndef CRANE_BT_EXECUTOR__SKILL__MOVE_HPP_
#define CRANE_BT_EXECUTOR__SKILL__MOVE_HPP_

#include <iostream>
#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"

class Move : public Composite
{
public:
  Move(float x, float y)
  : x_(x), y_(y), theta_(0), use_theta_(false) {}
  Move(float x, float y, float theta)
  : x_(x), y_(y), theta_(theta), use_theta_(true) {}
  Move(Point p)
  {
    Move(p.x(), p.y());
  }
  Status run(std::shared_ptr<WorldModel> world_model, RobotIO robot) override
  {
    Point target;
    target << x_, y_;

    // check
    if (bg::distance(target, robot.info->pose.pos) < 0.05f) {
      if (use_theta_) {
        if (tool::getAngleDiff(theta_, robot.info->pose.theta) < 0.05f) {
          std::cout << "Reached! : " << x_ << " , " << y_ <<  std::endl;
          return Status::SUCCESS;
        }
      } else {
        std::cout << "Reached! : " << x_ << " , " << y_ <<  std::endl;
        return Status::SUCCESS;
      }
    }

    std::cout << "Approarching : " << robot.info->pose.pos.x() << " , " << robot.info->pose.pos.y() <<  std::endl;
    robot.builder->setTargetPos(target);
    if (use_theta_) {
      robot.builder->setTargetTheta(theta_);
    }
    return Status::RUNNING;
  }
  float x_, y_, theta_;
  bool use_theta_;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__MOVE_HPP_
