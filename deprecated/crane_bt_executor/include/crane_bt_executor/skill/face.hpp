// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__SKILL__FACE_HPP_
#define CRANE_BT_EXECUTOR__SKILL__FACE_HPP_

#include <memory>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/target.hpp"
#include "crane_bt_executor/utils/tool.hpp"

class Face : public Composite
{
public:
  explicit Face(TargetModule target) : target_(target) {}
  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    float target_angle =
      tool::getAngle(tool::getDirectonVec(robot.info->pose.pos, target_.getPoint(world_model)));
    if (tool::getAngleDiff(target_angle, robot.info->pose.theta) < 0.05f) {
      return Status::SUCCESS;
    }

    robot.builder->setTargetTheta(target_angle);
    return Status::RUNNING;
  }
  TargetModule target_;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__FACE_HPP_
