// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__SKILL__KICK_TO_TARGET_HPP_
#define CRANE_BT_EXECUTOR__SKILL__KICK_TO_TARGET_HPP_

#include <memory>

#include "crane_bt_executor/behavior_tree/parallel_all.hpp"
#include "crane_bt_executor/behavior_tree/sequence.hpp"
#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/target.hpp"
#include "crane_bt_executor/utils/tool.hpp"

class KickToTarget : public Sequence
{
public:
  explicit KickToTarget(TargetModule target, float power = 1.0f);

  TargetModule target_;
  float kick_power_;
};

#endif  // CRANE_BT_EXECUTOR__SKILL__KICK_TO_TARGET_HPP_
