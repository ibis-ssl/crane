// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__SKILL__MOVE_HPP_
#define CRANE_BT_EXECUTOR__SKILL__MOVE_HPP_

#include <iostream>
#include <memory>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/target.hpp"

class Move : public Composite
{
public:
  explicit Move(TargetModule target, float threshold = 0.05f)
  : target_(target), THRESHOLD_(threshold)
  {
  }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    Point target = target_.getPoint(world_model);
    // check
    if (bg::distance(target, robot.info->pose.pos) < THRESHOLD_) {
      std::cout << "Reached! : " << target.x() << " , " << target.y() << std::endl;
      return Status::SUCCESS;
    }

    robot.builder->setTargetPos(target, false);
    //    robot.builder->setTargetTheta(1.57f);

    return Status::RUNNING;
  }
  TargetModule target_;
  const float THRESHOLD_;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__MOVE_HPP_
