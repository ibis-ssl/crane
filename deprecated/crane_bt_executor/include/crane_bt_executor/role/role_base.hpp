// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__ROLE__ROLE_BASE_HPP_
#define CRANE_BT_EXECUTOR__ROLE__ROLE_BASE_HPP_

#include "crane_bt_executor/behavior_tree/multi_robot_sequence.hpp"
#include "crane_bt_executor/role/role_command.hpp"

class RoleBase : public MultiRobotBehavior
{
public:
  RoleBase() {}
  virtual void configure(RoleCommand cmd) = 0;
  virtual void onAssignUpdate() = 0;
  virtual void onParamUpdate() = 0;
  //  virtual void update(const WorldModelWrapper & world_model) = 0;
};
#endif  // CRANE_BT_EXECUTOR__ROLE__ROLE_BASE_HPP_
