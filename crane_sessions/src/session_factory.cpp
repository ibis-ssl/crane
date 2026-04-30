// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/session_factory.hpp>
#include <functional>
#include <stdexcept>
#include <unordered_map>

// 全プランナーのインクルード（.cppファイルでのみ必要）
#include <crane_sessions/attacker_skill_session.hpp>
#include <crane_sessions/ball_calibration_data_collector_session.hpp>
#include <crane_sessions/ball_near_by_positioner_skill_session.hpp>
#include <crane_sessions/ball_placement_skill_session.hpp>
#include <crane_sessions/center_stop_kick_session.hpp>
#include <crane_sessions/defender_session.hpp>
#include <crane_sessions/emplace_robot_session.hpp>
#include <crane_sessions/formation_session.hpp>
#include <crane_sessions/forward_session.hpp>
#include <crane_sessions/free_kicker_skill_session.hpp>
#include <crane_sessions/goalie_skill_session.hpp>
#include <crane_sessions/latency_measurement_session.hpp>
#include <crane_sessions/marker_session.hpp>
#include <crane_sessions/our_free_kick_session.hpp>
#include <crane_sessions/our_penalty_kick_session.hpp>
#include <crane_sessions/pass_receiver_session.hpp>
#include <crane_sessions/passable_ball_placement_session.hpp>
#include <crane_sessions/placement_avoidance_session.hpp>
#include <crane_sessions/placement_target_near_by_positioner_skill_session.hpp>
#include <crane_sessions/robot_test_session.hpp>
#include <crane_sessions/second_threat_defender_session.hpp>
#include <crane_sessions/simple_kick_off_skill_session.hpp>
#include <crane_sessions/simple_placer_session.hpp>
#include <crane_sessions/sub_attacker_skill_session.hpp>
#include <crane_sessions/test_session.hpp>
#include <crane_sessions/their_penalty_kick_session.hpp>
#include <crane_sessions/total_defense_session.hpp>
#include <crane_sessions/waiter_session.hpp>

namespace crane
{
using SessionFactory =
  std::function<SessionBase::SharedPtr(WorldModelWrapper::SharedPtr &, rclcpp::Node &)>;

namespace
{
// プランナーファクトリマップの初期化
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define PLANNER_ENTRY(name, PlannerClass) \
  {name, [](auto & wm, auto & node) { return std::make_shared<PlannerClass>(wm, node); }}

auto getSessionFactoryMap() -> const std::unordered_map<std::string, SessionFactory> &
{
  static const std::unordered_map<std::string, SessionFactory> factory_map{
    PLANNER_ENTRY("attacker_skill", AttackerSkillSession),
    PLANNER_ENTRY("ball_nearby_positioner_skill", BallNearByPositionerSkillSession),
    PLANNER_ENTRY(
      "placement_target_nearby_positioner_skill", PlacementTargetNearByPositionerSkillSession),
    PLANNER_ENTRY("ball_placement_avoidance", BallPlacementAvoidanceSession),
    PLANNER_ENTRY("ball_placement_skill", BallPlacementSkillSession),
    PLANNER_ENTRY("passable_ball_placement", PassableBallPlacementSession),
    PLANNER_ENTRY("placement_target_placer", PlacementTargetPlacerSession),
    PLANNER_ENTRY("defender", DefenderSession),
    PLANNER_ENTRY("wing_formation", WingFormationSession),
    PLANNER_ENTRY("ibis_formation", IbisFormationSession),
    PLANNER_ENTRY("goalie_skill", GoalieSkillSession),
    PLANNER_ENTRY("marker", MarkerSession),
    PLANNER_ENTRY("sub_attacker_skill", SubAttackerSkillSession),
    PLANNER_ENTRY("waiter", WaiterSession),
    PLANNER_ENTRY("our_penalty_kick", OurPenaltyKickSession),
    PLANNER_ENTRY("pass_receive", PassReceiverSession),
    PLANNER_ENTRY("their_penalty_kick", TheirPenaltyKickSession),
    PLANNER_ENTRY("our_direct_free", OurDirectFreeKickSession),
    PLANNER_ENTRY("simple_kickoff", SimpleKickOffSkillSession),
    PLANNER_ENTRY("simple_placer", SimplePlacerSession),
    PLANNER_ENTRY("robot_test_session", RobotTestSession),
    PLANNER_ENTRY("test", TestSession),
    PLANNER_ENTRY("total_defense", TotalDefenseSession),
    PLANNER_ENTRY("emplace_robot", EmplaceRobotSession),
    PLANNER_ENTRY("forward", ForwardSession),
    PLANNER_ENTRY("free_kicker_skill", FreeKickerSkillSession),
    PLANNER_ENTRY("second_threat_defender", SecondThreatDefenderSession),
    PLANNER_ENTRY("ball_calibration_data_collector", BallCalibrationDataCollectorSession),
    PLANNER_ENTRY("center_stop_kick", CenterStopKickSession),
    PLANNER_ENTRY("latency_measurement", LatencyMeasurementSession),
  };
  return factory_map;
}
#undef PLANNER_ENTRY
}  // namespace

auto generatePlanner(
  const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> SessionBase::SharedPtr
{
  const auto & factory_map = getSessionFactoryMap();
  auto it = factory_map.find(tactic_name);
  if (it != factory_map.end()) {
    return it->second(world_model, node);
  }
  throw std::runtime_error("Unknown session name: " + tactic_name);
}

auto getAvailablePlannerNames() -> std::vector<std::string>
{
  const auto & factory_map = getSessionFactoryMap();
  std::vector<std::string> names;
  names.reserve(factory_map.size());
  for (const auto & [name, _] : factory_map) {
    names.push_back(name);
  }
  return names;
}

}  // namespace crane
