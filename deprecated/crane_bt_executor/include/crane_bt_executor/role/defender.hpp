// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__ROLE__DEFENDER_HPP_
#define CRANE_BT_EXECUTOR__ROLE__DEFENDER_HPP_

#include "crane_bt_executor/role/role_base.hpp"

class DefenderRole : public RoleBase
{
public:
  DefenderRole()
  {
  }
  void configure(RoleCommand cmd) override
  {
  }
  void onAssignUpdate() override
  {
  }
  void onParamUpdate() override
  {
  }
};
#endif  // CRANE_BT_EXECUTOR__ROLE__DEFENDER_HPP_
