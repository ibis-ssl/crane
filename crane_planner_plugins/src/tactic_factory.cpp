// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/tactic_factory.hpp>
#include <functional>
#include <stdexcept>
#include <unordered_map>

// 全プランナーのインクルード（.cppファイルでのみ必要）
#include <crane_planner_plugins/attacker_skill_tactic.hpp>
#include <crane_planner_plugins/ball_calibration_data_collector_tactic.hpp>
#include <crane_planner_plugins/center_stop_kick_tactic.hpp>
#include <crane_planner_plugins/defender_tactic.hpp>
#include <crane_planner_plugins/emplace_robot_tactic.hpp>
#include <crane_planner_plugins/formation_tactic.hpp>
#include <crane_planner_plugins/forward_tactic.hpp>
#include <crane_planner_plugins/marker_tactic.hpp>
#include <crane_planner_plugins/our_free_kick_tactic.hpp>
#include <crane_planner_plugins/our_penalty_kick_tactic.hpp>
#include <crane_planner_plugins/pass_receiver_tactic.hpp>
#include <crane_planner_plugins/passable_ball_placement_tactic.hpp>
#include <crane_planner_plugins/placement_avoidance_tactic.hpp>
#include <crane_planner_plugins/second_threat_defender_tactic.hpp>
#include <crane_planner_plugins/simple_ai_tactic.hpp>
#include <crane_planner_plugins/simple_placer_tactic.hpp>
#include <crane_planner_plugins/skill_tactic.hpp>
#include <crane_planner_plugins/test_tactic.hpp>
#include <crane_planner_plugins/their_penalty_kick_tactic.hpp>
#include <crane_planner_plugins/total_defense_tactic.hpp>
#include <crane_planner_plugins/waiter_tactic.hpp>

namespace crane
{
using TacticFactory =
  std::function<TacticBase::SharedPtr(WorldModelWrapper::SharedPtr &, rclcpp::Node &)>;

namespace
{
// プランナーファクトリマップの初期化
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define PLANNER_ENTRY(name, PlannerClass) \
  {name, [](auto & wm, auto & node) { return std::make_shared<PlannerClass>(wm, node); }}

auto getTacticFactoryMap() -> const std::unordered_map<std::string, TacticFactory> &
{
  static const std::unordered_map<std::string, TacticFactory> factory_map{
    PLANNER_ENTRY("attacker_skill", AttackerSkillTactic),
    PLANNER_ENTRY("ball_nearby_positioner_skill", BallNearByPositionerSkillTactic),
    PLANNER_ENTRY(
      "placement_target_nearby_positioner_skill", PlacementTargetNearByPositionerSkillTactic),
    PLANNER_ENTRY("ball_placement_avoidance", BallPlacementAvoidanceTactic),
    PLANNER_ENTRY("ball_placement_skill", BallPlacementSkillTactic),
    PLANNER_ENTRY("passable_ball_placement", PassableBallPlacementTactic),
    PLANNER_ENTRY("placement_target_placer", PlacementTargetPlacerTactic),
    PLANNER_ENTRY("defender", DefenderTactic),
    PLANNER_ENTRY("wing_formation", WingFormationTactic),
    PLANNER_ENTRY("ibis_formation", IbisFormationTactic),
    PLANNER_ENTRY("goalie_skill", GoalieSkillTactic),
    PLANNER_ENTRY("marker", MarkerTactic),
    PLANNER_ENTRY("sub_attacker_skill", SubAttackerSkillTactic),
    PLANNER_ENTRY("waiter", WaiterTactic),
    PLANNER_ENTRY("our_penalty_kick", OurPenaltyKickTactic),
    PLANNER_ENTRY("pass_receive", PassReceiverTactic),
    PLANNER_ENTRY("their_penalty_kick", TheirPenaltyKickTactic),
    PLANNER_ENTRY("our_direct_free", OurDirectFreeKickTactic),
    PLANNER_ENTRY("simple_ai", SimpleAITactic),
    PLANNER_ENTRY("simple_kickoff", SimpleKickOffSkillTactic),
    PLANNER_ENTRY("simple_placer", SimplePlacerTactic),
    PLANNER_ENTRY("test", TestTactic),
    PLANNER_ENTRY("total_defense", TotalDefenseTactic),
    PLANNER_ENTRY("emplace_robot", EmplaceRobotTactic),
    PLANNER_ENTRY("forward", ForwardTactic),
    PLANNER_ENTRY("second_threat_defender", SecondThreatDefenderTactic),
    PLANNER_ENTRY("ball_calibration_data_collector", BallCalibrationDataCollectorTactic),
    PLANNER_ENTRY("center_stop_kick", CenterStopKickTactic),
  };
  return factory_map;
}
#undef PLANNER_ENTRY
}  // namespace

auto generatePlanner(
  const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> TacticBase::SharedPtr
{
  const auto & factory_map = getTacticFactoryMap();
  auto it = factory_map.find(tactic_name);
  if (it != factory_map.end()) {
    return it->second(world_model, node);
  }
  throw std::runtime_error("Unknown tactic name: " + tactic_name);
}

auto getAvailablePlannerNames() -> std::vector<std::string>
{
  const auto & factory_map = getTacticFactoryMap();
  std::vector<std::string> names;
  names.reserve(factory_map.size());
  for (const auto & [name, _] : factory_map) {
    names.push_back(name);
  }
  return names;
}

}  // namespace crane
