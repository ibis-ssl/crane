// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PLANNERS_HPP_
#define CRANE_PLANNER_PLUGINS__PLANNERS_HPP_

#include <crane_planner_plugins/planner_base.hpp>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "attacker_skill_planner.hpp"
#include "catch_ball_planner.hpp"
#include "defender_planner.hpp"
#include "formation_planner.hpp"
#include "marker_planner.hpp"
#include "offensive_planner.hpp"
#include "our_free_kick_planner.hpp"
#include "our_kickoff_planner.hpp"
#include "our_penalty_kick_planner.hpp"
#include "pass_receiver_planner.hpp"
#include "placement_avoidance_planner.hpp"
#include "simple_placer_planner.hpp"
#include "skill_planner.hpp"
#include "test_planner.hpp"
// #include "temporary/ball_placement_planner.hpp"
#include "emplace_robot_planner.hpp"
#include "sandwich_ball_placement_planner.hpp"
#include "simple_ai_planner.hpp"
#include "their_penalty_kick_planner.hpp"
#include "tigers_goalie_planner.hpp"
#include "total_defense_planner.hpp"
#include "waiter_planner.hpp"

namespace crane
{
template <typename... Ts>
inline auto generatePlanner(const std::string & planner_name, Ts &&... ts) -> PlannerBase::SharedPtr
{
  static auto planner_factory =
    std::unordered_map<std::string, std::function<PlannerBase::SharedPtr(Ts...)>>{
      // clang-format off
      // NOLINTBEGIN(whitespace/line_length)
      {"attacker_skill",               [](Ts... ts) { return std::make_shared<AttackerSkillPlanner>(ts...); }},
      {"ball_nearby_positioner_skill", [](Ts... ts) { return std::make_shared<BallNearByPositionerSkillPlanner>(ts...); }},
      {"ball_placement_avoidance",     [](Ts... ts) { return std::make_shared<BallPlacementAvoidancePlanner>(ts...); }},
      {"ball_placement_skill",         [](Ts... ts) { return std::make_shared<BallPlacementSkillPlanner>(ts...); }},
      {"defender",                     [](Ts... ts) { return std::make_shared<DefenderPlanner>(ts...); }},
      {"wing_formation",               [](Ts... ts) { return std::make_shared<WingFormationPlanner>(ts...); }},
      {"ibis_formation",               [](Ts... ts) { return std::make_shared<IbisFormationPlanner>(ts...); }},
      {"goalie_skill",                 [](Ts... ts) { return std::make_shared<GoalieSkillPlanner>(ts...); }},
      {"marker",                       [](Ts... ts) { return std::make_shared<MarkerPlanner>(ts...); }},
      {"sub_attacker_skill",           [](Ts... ts) { return std::make_shared<SubAttackerSkillPlanner>(ts...); }},
      {"catch_ball",                   [](Ts... ts) { return std::make_shared<CatchBallPlanner>(ts...); }},
      {"tigers_goalie",                [](Ts... ts) { return std::make_shared<TigersGoaliePlanner>(ts...); }},
      {"waiter",                       [](Ts... ts) { return std::make_shared<WaiterPlanner>(ts...); }},
      {"our_kickoff",                  [](Ts... ts) { return std::make_shared<OurKickOffPlanner>(ts...); }},
      {"our_penalty_kick",             [](Ts... ts) { return std::make_shared<OurPenaltyKickPlanner>(ts...); }},
      {"pass_receive",                 [](Ts... ts) { return std::make_shared<PassReceiverPlanner>(ts...); }},
      {"their_penalty_kick",           [](Ts... ts) { return std::make_shared<TheirPenaltyKickPlanner>(ts...); }},
      {"offensive",                    [](Ts... ts) { return std::make_shared<OffensivePlanner>(ts...); }},
      {"our_direct_free",              [](Ts... ts) { return std::make_shared<OurDirectFreeKickPlanner>(ts...); }},
      {"steal_ball",                   [](Ts... ts) { return std::make_shared<StealBallSkillPlanner>(ts...); }},
      {"free_kick_saver",              [](Ts... ts) { return std::make_shared<FreeKickSaverSkillPlanner>(ts...); }},
      {"sandwich_ball_placement",      [](Ts... ts) { return std::make_shared<SandwichBallPlacementPlanner>(ts...); }},
      {"simple_ai",                    [](Ts... ts) { return std::make_shared<SimpleAIPlanner>(ts...); }},
      {"simple_kickoff",               [](Ts... ts) { return std::make_shared<SimpleKickOffSkillPlanner>(ts...); }},
      {"simple_placer",                [](Ts... ts) { return std::make_shared<SimplePlacerPlanner>(ts...); }},
      {"test",                         [](Ts... ts) { return std::make_shared<TestPlanner>(ts...); }},
      {"total_defense",                [](Ts... ts) { return std::make_shared<TotalDefensePlanner>(ts...); }},
      {"emplace_robot",                [](Ts... ts) { return std::make_shared<EmplaceRobotPlanner>(ts...); }}
      // NOLINTEND
      // clang-format on
    };

  auto it = planner_factory.find(planner_name);
  if (it != planner_factory.end()) {
    return it->second(std::forward<Ts>(ts)...);
  }
  throw std::runtime_error("Unknown planner name: " + planner_name);
}
}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__PLANNERS_HPP_
