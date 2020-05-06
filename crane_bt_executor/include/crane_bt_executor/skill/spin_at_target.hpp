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

#ifndef CRANE_BT_EXECUTOR__SKILL__SPIN_AT_TARGET_HPP_
#define CRANE_BT_EXECUTOR__SKILL__SPIN_AT_TARGET_HPP_

#include <memory>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/tool.hpp"

class SpinAtTarget : public Composite
{
public:
  SpinAtTarget(TargetModule target, TargetModule over_target, float theta_threshold = 0.1f)
  : target_(target), over_target_(over_target), THETA_THRESHOLD(theta_threshold) {}
  Status run(std::shared_ptr<WorldModel> world_model, RobotIO robot) override
  {
    Point robot_pos = robot.info->pose.pos;
    Point target_pos = target_.getPoint(world_model);
    Point over_target_pos = over_target_.getPoint(world_model);

    if (INITIAL_DISTANCE == -1) {
      INITIAL_DISTANCE = (robot.info->pose.pos - target_pos).norm();
    }

    float current_angle = tool::getAngle(robot_pos - target_pos);
    float target_angle = tool::getAngle(over_target_pos - target_pos);

    if (std::abs(tool::getAngleDiff(current_angle, target_angle)) < THETA_THRESHOLD) {
      std::cout << "SpinAtTarget finished!" << int(robot.info->id) << std::endl;
      return Status::SUCCESS;
    }

    // the sign of cross product is as same as sin(theta)
    Vector2 a = robot_pos - target_pos;
    Vector2 b = robot_pos - over_target_pos;
    float sin = a.x() * b.y() - b.x() * a.y();

    Eigen::Rotation2D<float> rot;
    if (sin < 0) {
      rot.angle() = M_PI_2;
    } else {rot.angle() = -M_PI_2;}

    float diff = (robot_pos - target_pos).norm() - INITIAL_DISTANCE;
    Vector2 dir = rot * (robot_pos - target_pos) - (robot_pos - target_pos) * diff;

    float dist = dir.norm() * tool::getAngleDiff(current_angle, target_angle);

    robot.builder->setVelocity(dir, dist);
    robot.builder->setTargetTheta(0);
    return Status::RUNNING;
  }

  TargetModule target_, over_target_;
  const float THETA_THRESHOLD;
  float INITIAL_DISTANCE = -1;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__SPIN_AT_TARGET_HPP_