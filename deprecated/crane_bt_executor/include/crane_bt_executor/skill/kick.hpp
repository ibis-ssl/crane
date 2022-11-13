// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__SKILL__KICK_HPP_
#define CRANE_BT_EXECUTOR__SKILL__KICK_HPP_

#include <iostream>
#include <memory>

#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/target.hpp"

class Kick : public Composite
{
public:
  explicit Kick(float power = 1.0f) : POWER_(power)
  {
  }

  Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) override
  {
    robot.builder->addStraightKick(POWER_);

    return Status::SUCCESS;
  }
  const float POWER_;
};
#endif  // CRANE_BT_EXECUTOR__SKILL__KICK_HPP_
