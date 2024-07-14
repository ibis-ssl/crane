// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PLANNERS_HPP_
#define CRANE_PLANNER_PLUGINS__PLANNERS_HPP_

#include <crane_planner_base/planner_base.hpp>
#include <memory>
#include <string>

#include "attacker_planner.hpp"
#include "catch_ball_planner.hpp"
#include "defender_planner.hpp"
#include "formation_planner.hpp"
#include "marker_planner.hpp"
#include "our_free_kick_planner.hpp"
#include "our_kickoff_planner.hpp"
#include "our_penalty_kick_planner.hpp"
#include "skill_planner.hpp"
// #include "temporary/ball_placement_planner.hpp"
#include "their_penalty_kick_planner.hpp"
#include "tigers_goalie_planner.hpp"
#include "waiter_planner.hpp"

namespace crane
{
template <typename... Ts>
auto generatePlanner(const std::string & planner_name, Ts... ts) -> PlannerBase::SharedPtr
{
  if (planner_name == "attacker") {
    return std::make_shared<AttackerPlanner>(ts...);
    //  } else if (planner_name == "ball_placement") {
    //  return std::make_shared<BallPlacementPlanner>(ts...);
  } else if (planner_name == "ball_placement_skill") {
    return std::make_shared<BallPlacementSkillPlanner>(ts...);
  } else if (planner_name == "defender") {
    return std::make_shared<DefenderPlanner>(ts...);
  } else if (planner_name == "formation") {
    return std::make_shared<FormationPlanner>(ts...);
  } else if (planner_name == "goalie_skill") {
    return std::make_shared<GoalieSkillPlanner>(ts...);
  } else if (planner_name == "marker") {
    return std::make_shared<MarkerPlanner>(ts...);
  } else if (planner_name == "receiver_skill") {
    return std::make_shared<ReceiverSkillPlanner>(ts...);
  } else if (planner_name == "catch_ball") {
    return std::make_shared<CatchBallPlanner>(ts...);
  } else if (planner_name == "tigers_goalie") {
    return std::make_shared<TigersGoaliePlanner>(ts...);
  } else if (planner_name == "waiter") {
    return std::make_shared<WaiterPlanner>(ts...);
  } else if (planner_name == "our_kickoff") {
    return std::make_shared<OurKickOffPlanner>(ts...);
  } else if (planner_name == "our_penalty_kick") {
    return std::make_shared<OurPenaltyKickPlanner>(ts...);
  } else if (planner_name == "their_penalty_kick") {
    return std::make_shared<TheirPenaltyKickPlanner>(ts...);
  } else if (planner_name == "our_direct_free") {
    return std::make_shared<OurDirectFreeKickPlanner>(ts...);
  } else if (planner_name == "steal_ball") {
    return std::make_shared<StealBallSkillPlanner>(ts...);
  } else if (planner_name == "free_kick_saver") {
    return std::make_shared<FreeKickSaverSkillPlanner>(ts...);
  } else if (planner_name == "simple_kickoff") {
    return std::make_shared<SimpleKickOffSkillPlanner>(ts...);
  } else {
    throw std::runtime_error("Unknown planner name: " + planner_name);
  }
}
}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__PLANNERS_HPP_
