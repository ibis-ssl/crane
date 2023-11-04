// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PLANNERS_HPP_
#define CRANE_PLANNER_PLUGINS__PLANNERS_HPP_

#include <crane_planner_base/planner_base.hpp>

#include "attacker_planner.hpp"
#include "ball_placement_planner.hpp"
#include "defender_planner.hpp"
#include "formation_planner.hpp"
#include "goalie_planner.hpp"
#include "kickoff_planner.hpp"
#include "marker_planner.hpp"
#include "receive_planner.hpp"
#include "tigers_goalie_planner.hpp"
#include "waiter_planner.hpp"

namespace crane
{
template <typename... Ts>
auto generatePlanner(const std::string & planner_name, Ts... ts) -> std::shared_ptr<PlannerBase>
{
  if (planner_name == "attacker") {
    return std::make_shared<AttackerPlanner>(ts...);
  } else if (planner_name == "ball_placement") {
    return std::make_shared<BallPlacementPlanner>(ts...);
  } else if (planner_name == "defender") {
    return std::make_shared<DefenderPlanner>(ts...);
  } else if (planner_name == "formation") {
    return std::make_shared<FormationPlanner>(ts...);
  } else if (planner_name == "goalie") {
    return std::make_shared<GoaliePlanner>(ts...);
  } else if (planner_name == "kickoff") {
    return std::make_shared<KickOffPlanner>(ts...);
  } else if (planner_name == "marker") {
    return std::make_shared<MarkerPlanner>(ts...);
  } else if (planner_name == "receive") {
    return std::make_shared<ReceivePlanner>(ts...);
  } else if (planner_name == "tigers_goalie") {
    return std::make_shared<TigersGoaliePlanner>(ts...);
  } else if (planner_name == "waiter") {
    return std::make_shared<WaiterPlanner>(ts...);
  }
}
}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__PLANNERS_HPP_
