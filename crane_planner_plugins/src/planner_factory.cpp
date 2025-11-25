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
auto getPlannerFactoryMap() -> const std::unordered_map<std::string, PlannerFactory> &
{
  // clang-format off
  static const std::unordered_map<std::string, PlannerFactory> factory_map{
    {"attacker_skill",                            [](auto & wm, auto & node) { return std::make_shared<AttackerSkillPlanner>(wm, node); }},
    {"ball_nearby_positioner_skill",              [](auto & wm, auto & node) { return std::make_shared<BallNearByPositionerSkillPlanner>(wm, node); }},
    {"placement_target_nearby_positioner_skill",  [](auto & wm, auto & node) { return std::make_shared<PlacementTargetNearByPositionerSkillPlanner>(wm, node); }},
    {"ball_placement_avoidance",                  [](auto & wm, auto & node) { return std::make_shared<BallPlacementAvoidancePlanner>(wm, node); }},
    {"ball_placement_skill",                      [](auto & wm, auto & node) { return std::make_shared<BallPlacementSkillPlanner>(wm, node); }},
    {"passable_ball_placement",                   [](auto & wm, auto & node) { return std::make_shared<PassableBallPlacementPlanner>(wm, node); }},
    {"placement_target_placer",                   [](auto & wm, auto & node) { return std::make_shared<PlacementTargetPlacerPlanner>(wm, node); }},
    {"defender",                                  [](auto & wm, auto & node) { return std::make_shared<DefenderPlanner>(wm, node); }},
    {"wing_formation",                            [](auto & wm, auto & node) { return std::make_shared<WingFormationPlanner>(wm, node); }},
    {"ibis_formation",                            [](auto & wm, auto & node) { return std::make_shared<IbisFormationPlanner>(wm, node); }},
    {"goalie_skill",                              [](auto & wm, auto & node) { return std::make_shared<GoalieSkillPlanner>(wm, node); }},
    {"marker",                                    [](auto & wm, auto & node) { return std::make_shared<MarkerPlanner>(wm, node); }},
    {"sub_attacker_skill",                        [](auto & wm, auto & node) { return std::make_shared<SubAttackerSkillPlanner>(wm, node); }},
    {"waiter",                                    [](auto & wm, auto & node) { return std::make_shared<WaiterPlanner>(wm, node); }},
    {"our_penalty_kick",                          [](auto & wm, auto & node) { return std::make_shared<OurPenaltyKickPlanner>(wm, node); }},
    {"pass_receive",                              [](auto & wm, auto & node) { return std::make_shared<PassReceiverPlanner>(wm, node); }},
    {"their_penalty_kick",                        [](auto & wm, auto & node) { return std::make_shared<TheirPenaltyKickPlanner>(wm, node); }},
    {"offensive",                                 [](auto & wm, auto & node) { return std::make_shared<OffensivePlanner>(wm, node); }},
    {"our_direct_free",                           [](auto & wm, auto & node) { return std::make_shared<OurDirectFreeKickPlanner>(wm, node); }},
    {"simple_ai",                                 [](auto & wm, auto & node) { return std::make_shared<SimpleAIPlanner>(wm, node); }},
    {"simple_kickoff",                            [](auto & wm, auto & node) { return std::make_shared<SimpleKickOffSkillPlanner>(wm, node); }},
    {"simple_placer",                             [](auto & wm, auto & node) { return std::make_shared<SimplePlacerPlanner>(wm, node); }},
    {"test",                                      [](auto & wm, auto & node) { return std::make_shared<TestPlanner>(wm, node); }},
    {"total_defense",                             [](auto & wm, auto & node) { return std::make_shared<TotalDefensePlanner>(wm, node); }},
    {"emplace_robot",                             [](auto & wm, auto & node) { return std::make_shared<EmplaceRobotPlanner>(wm, node); }},
    {"forward",                                   [](auto & wm, auto & node) { return std::make_shared<ForwardPlanner>(wm, node); }},
    {"second_threat_defender",                    [](auto & wm, auto & node) { return std::make_shared<SecondThreatDefenderPlanner>(wm, node); }},
    {"ball_calibration_data_collector",           [](auto & wm, auto & node) { return std::make_shared<BallCalibrationDataCollectorPlanner>(wm, node); }},
    {"center_stop_kick",                          [](auto & wm, auto & node) { return std::make_shared<CenterStopKickPlanner>(wm, node); }}
  };
  // clang-format on
  return factory_map;
}
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
