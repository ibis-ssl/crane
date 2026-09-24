// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__PASS_PLAN_HPP_
#define CRANE_MSG_WRAPPERS__PASS_PLAN_HPP_

#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>

namespace crane
{
/// 出し手・受け手が同じ条件で計画を採用する。セットプレイは専用スキルに任せる。
inline auto isUsablePassPlan(const crane_msgs::msg::PassPlan & plan, const WorldModelWrapper & wm)
  -> bool
{
  using Plan = crane_msgs::msg::PassPlan;
  if (
    wm.getMsg().play_situation.command.value != crane_msgs::msg::PlaySituation::INPLAY ||
    (plan.state != Plan::STATE_PLANNING && plan.state != Plan::STATE_BALL_IN_FLIGHT) ||
    plan.kicker_id < 0 || plan.receiver_id < 0 || plan.kicker_id == plan.receiver_id ||
    plan.is_chip || !std::isfinite(plan.kick_speed) || plan.kick_speed <= 0.0 ||
    !std::isfinite(plan.score) || plan.score <= 0.0) {
    return false;
  }
  bool kicker_available = false;
  bool receiver_available = false;
  for (const auto & robot : wm.ours().robotsWhere().available().excludeGoalie().get()) {
    kicker_available |= static_cast<int>(robot->id) == plan.kicker_id;
    receiver_available |= static_cast<int>(robot->id) == plan.receiver_id;
  }
  const Point target(plan.receive_point.x, plan.receive_point.y);
  return kicker_available && receiver_available && target.allFinite() &&
         target.x() * wm.getOurSideSign() < 0.0 && wm.point_checker.isFieldInside(target) &&
         !wm.point_checker.isPenaltyArea(target);
}
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__PASS_PLAN_HPP_
