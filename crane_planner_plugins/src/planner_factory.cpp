// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/planner_factory.hpp>
#include <functional>
#include <stdexcept>
#include <unordered_map>

// 全プランナーのインクルード（.cppファイルでのみ必要）
#include <crane_planner_plugins/attacker_skill_planner.hpp>
#include <crane_planner_plugins/ball_calibration_data_collector_planner.hpp>
#include <crane_planner_plugins/center_stop_kick_planner.hpp>
#include <crane_planner_plugins/defender_planner.hpp>
#include <crane_planner_plugins/emplace_robot_planner.hpp>
#include <crane_planner_plugins/formation_planner.hpp>
#include <crane_planner_plugins/forward_planner.hpp>
#include <crane_planner_plugins/marker_planner.hpp>
#include <crane_planner_plugins/offensive_planner.hpp>
#include <crane_planner_plugins/our_free_kick_planner.hpp>
#include <crane_planner_plugins/our_penalty_kick_planner.hpp>
#include <crane_planner_plugins/pass_receiver_planner.hpp>
#include <crane_planner_plugins/passable_ball_placement_planner.hpp>
#include <crane_planner_plugins/placement_avoidance_planner.hpp>
#include <crane_planner_plugins/second_threat_defender_planner.hpp>
#include <crane_planner_plugins/simple_ai_planner.hpp>
#include <crane_planner_plugins/simple_placer_planner.hpp>
#include <crane_planner_plugins/skill_planner.hpp>
#include <crane_planner_plugins/test_planner.hpp>
#include <crane_planner_plugins/their_penalty_kick_planner.hpp>
#include <crane_planner_plugins/total_defense_planner.hpp>
#include <crane_planner_plugins/waiter_planner.hpp>

namespace crane
{
using PlannerFactory =
  std::function<PlannerBase::SharedPtr(WorldModelWrapper::SharedPtr &, rclcpp::Node &)>;

namespace
{
// プランナーファクトリマップの初期化
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define PLANNER_ENTRY(name, PlannerClass) \
  {name, [](auto & wm, auto & node) { return std::make_shared<PlannerClass>(wm, node); }}

auto getPlannerFactoryMap() -> const std::unordered_map<std::string, PlannerFactory> &
{
  static const std::unordered_map<std::string, PlannerFactory> factory_map{
    PLANNER_ENTRY("attacker_skill", AttackerSkillPlanner),
    PLANNER_ENTRY("ball_nearby_positioner_skill", BallNearByPositionerSkillPlanner),
    PLANNER_ENTRY(
      "placement_target_nearby_positioner_skill", PlacementTargetNearByPositionerSkillPlanner),
    PLANNER_ENTRY("ball_placement_avoidance", BallPlacementAvoidancePlanner),
    PLANNER_ENTRY("ball_placement_skill", BallPlacementSkillPlanner),
    PLANNER_ENTRY("passable_ball_placement", PassableBallPlacementPlanner),
    PLANNER_ENTRY("placement_target_placer", PlacementTargetPlacerPlanner),
    PLANNER_ENTRY("defender", DefenderPlanner),
    PLANNER_ENTRY("wing_formation", WingFormationPlanner),
    PLANNER_ENTRY("ibis_formation", IbisFormationPlanner),
    PLANNER_ENTRY("goalie_skill", GoalieSkillPlanner),
    PLANNER_ENTRY("marker", MarkerPlanner),
    PLANNER_ENTRY("sub_attacker_skill", SubAttackerSkillPlanner),
    PLANNER_ENTRY("waiter", WaiterPlanner),
    PLANNER_ENTRY("our_penalty_kick", OurPenaltyKickPlanner),
    PLANNER_ENTRY("pass_receive", PassReceiverPlanner),
    PLANNER_ENTRY("their_penalty_kick", TheirPenaltyKickPlanner),
    PLANNER_ENTRY("offensive", OffensivePlanner),
    PLANNER_ENTRY("our_direct_free", OurDirectFreeKickPlanner),
    PLANNER_ENTRY("simple_ai", SimpleAIPlanner),
    PLANNER_ENTRY("simple_kickoff", SimpleKickOffSkillPlanner),
    PLANNER_ENTRY("simple_placer", SimplePlacerPlanner),
    PLANNER_ENTRY("test", TestPlanner),
    PLANNER_ENTRY("total_defense", TotalDefensePlanner),
    PLANNER_ENTRY("emplace_robot", EmplaceRobotPlanner),
    PLANNER_ENTRY("forward", ForwardPlanner),
    PLANNER_ENTRY("second_threat_defender", SecondThreatDefenderPlanner),
    PLANNER_ENTRY("ball_calibration_data_collector", BallCalibrationDataCollectorPlanner),
    PLANNER_ENTRY("center_stop_kick", CenterStopKickPlanner),
  };
  return factory_map;
}
#undef PLANNER_ENTRY
}  // namespace

auto generatePlanner(
  const std::string & planner_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> PlannerBase::SharedPtr
{
  const auto & factory_map = getPlannerFactoryMap();
  auto it = factory_map.find(planner_name);
  if (it != factory_map.end()) {
    return it->second(world_model, node);
  }
  throw std::runtime_error("Unknown planner name: " + planner_name);
}

auto getAvailablePlannerNames() -> std::vector<std::string>
{
  const auto & factory_map = getPlannerFactoryMap();
  std::vector<std::string> names;
  names.reserve(factory_map.size());
  for (const auto & [name, _] : factory_map) {
    names.push_back(name);
  }
  return names;
}

}  // namespace crane
