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

#ifndef CRANE_BT_EXECUTOR__SKILL__FACE_HPP_
#define CRANE_BT_EXECUTOR__SKILL__FACE_HPP_

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/tool.hpp"
class Face : public Composite
{
public:
  Face(float x, float y)
  : x_(x), y_(y) {}
  Status run(WorldModel & world_model, RobotIO robot) override
  {
    Point target;
    target << x_, y_;
    float target_angle = tool::getAngle(tool::getDirectonVec(robot.info->pose.pos), target);
    if (tool::getAngleDiff(target_angle, robot.info->pose.theta) < 0.05f) {
      return Status::SUCCESS;
    }

    robot.builder->setTargetTheta(target_angle);
    return Status::RUNNING;
  }
  float x_, y_;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__FACE_HPP_