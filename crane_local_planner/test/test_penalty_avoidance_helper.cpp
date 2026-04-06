// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_local_planner/penalty_avoidance_helper.hpp>

namespace crane
{
namespace
{
constexpr double FIELD_HALF_X = 6.0;
constexpr double PA_DEPTH = 1.8;
constexpr double PA_HALF_WIDTH = 1.8;

auto makeOurPA() -> Box
{
  return Box(Point(-FIELD_HALF_X, -PA_HALF_WIDTH), Point(-FIELD_HALF_X + PA_DEPTH, PA_HALF_WIDTH));
}

auto ourGoal() -> Point { return Point(-FIELD_HALF_X, 0.0); }
auto paSize() -> Point { return Point(PA_DEPTH, PA_HALF_WIDTH * 2.0); }
}  // namespace

TEST(PenaltyAvoidanceHelperTest, NoCrossingDoesNotOverrideTarget)
{
  const auto decision = computePenaltyBypassDecision(
    Point(-2.0, 2.5), Point(-1.0, 2.5), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2, true);
  EXPECT_FALSE(decision.crossing_detected);
  EXPECT_FALSE(decision.target_overridden);
}

TEST(PenaltyAvoidanceHelperTest, CrossingOverridesTargetWithWaypoint)
{
  const auto decision = computePenaltyBypassDecision(
    Point(-3.0, 0.0), Point(-5.9, 0.0), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2, true);
  EXPECT_TRUE(decision.crossing_detected);
  EXPECT_TRUE(decision.target_overridden);
  EXPECT_NEAR(decision.waypoint.x(), -3.9, 1e-6);
  EXPECT_NEAR(std::abs(decision.waypoint.y()), 2.1, 1e-6);
}

TEST(PenaltyAvoidanceHelperTest, OutsideBypassWaypointDoesNotReTriggerCrossing)
{
  // 1st step: crossing target is overridden to a bypass waypoint (bottom side).
  const auto first = computePenaltyBypassDecision(
    Point(-3.0, 0.0), Point(-5.9, -0.6), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2, true);
  ASSERT_TRUE(first.crossing_detected);
  ASSERT_TRUE(first.target_overridden);
  ASSERT_EQ(first.selected_side, PenaltyBypassSide::BOTTOM);

  // 2nd step: once robot is on the bypass waypoint, heading to opposite-side outside target
  // should not be considered as crossing anymore.
  const auto second = computePenaltyBypassDecision(
    first.waypoint, Point(-3.9, 2.5), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2, true);
  EXPECT_FALSE(second.crossing_detected);
  EXPECT_FALSE(second.target_overridden);
}

TEST(PenaltyAvoidanceHelperTest, VerticalSegmentThroughPenaltyIsDetectedAsCrossing)
{
  const auto decision = computePenaltyBypassDecision(
    Point(-4.95, -3.61), Point(-4.75, 3.08), makeOurPA(), ourGoal(), paSize(), 0.3, 0.2, true);
  EXPECT_TRUE(decision.crossing_detected);
  EXPECT_TRUE(decision.target_overridden);
}

// 到達可能性チェック: ロボットPA下方・ターゲットPA上方の2ステップルーティング
// Step1: current→TOP はPA横断あり(到達不可)→ BOTTOM を中間ウェイポイントとして選択
// Step2: BOTTOMウェイポイントから TOP fully_good → TOP 選択でターゲットへ到達
TEST(PenaltyAvoidanceHelperTest, ReachabilityCheckSelectsBOTTOMWhenTargetIsAbovePA)
{
  Box expanded = makeOurPA();
  expanded.min_corner() -= Point(0.3, 0.3);
  expanded.max_corner() += Point(0.3, 0.3);

  const auto step1 = computePenaltyBypassDecision(
    Point(-4.95, -3.61), Point(-4.75, 3.08), makeOurPA(), ourGoal(), paSize(), 0.3, 0.2, true);
  ASSERT_TRUE(step1.crossing_detected);
  EXPECT_EQ(step1.selected_side, PenaltyBypassSide::BOTTOM);
  // current → BOTTOM ウェイポイントは PA を横断しない
  EXPECT_FALSE(intersectsSegmentAABB(Point(-4.95, -3.61), step1.waypoint, expanded));

  // Step 2: BOTTOM ウェイポイントから再計画 → TOP fully_good → TOP 選択
  const auto step2 = computePenaltyBypassDecision(
    step1.waypoint, Point(-4.75, 3.08), makeOurPA(), ourGoal(), paSize(), 0.3, 0.2, true);
  ASSERT_TRUE(step2.crossing_detected);
  EXPECT_EQ(step2.selected_side, PenaltyBypassSide::TOP);
  EXPECT_FALSE(intersectsSegmentAABB(step2.waypoint, Point(-4.75, 3.08), expanded));
}

// 到達可能性チェック: ロボットPA上方・ターゲットPA下方の2ステップルーティング
// Step1: current→BOTTOM はPA横断あり(到達不可)→ TOP を中間ウェイポイントとして選択
// Step2: TOPウェイポイントから BOTTOM fully_good → BOTTOM 選択でターゲットへ到達
TEST(PenaltyAvoidanceHelperTest, ReachabilityCheckSelectsTOPWhenTargetIsBelowPA)
{
  Box expanded = makeOurPA();
  expanded.min_corner() -= Point(0.3, 0.3);
  expanded.max_corner() += Point(0.3, 0.3);

  const auto step1 = computePenaltyBypassDecision(
    Point(-4.75, 3.08), Point(-4.95, -3.61), makeOurPA(), ourGoal(), paSize(), 0.3, 0.2, true);
  ASSERT_TRUE(step1.crossing_detected);
  EXPECT_EQ(step1.selected_side, PenaltyBypassSide::TOP);
  // current → TOP ウェイポイントは PA を横断しない
  EXPECT_FALSE(intersectsSegmentAABB(Point(-4.75, 3.08), step1.waypoint, expanded));

  // Step 2: TOP ウェイポイントから再計画 → BOTTOM fully_good → BOTTOM 選択
  const auto step2 = computePenaltyBypassDecision(
    step1.waypoint, Point(-4.95, -3.61), makeOurPA(), ourGoal(), paSize(), 0.3, 0.2, true);
  ASSERT_TRUE(step2.crossing_detected);
  EXPECT_EQ(step2.selected_side, PenaltyBypassSide::BOTTOM);
  EXPECT_FALSE(intersectsSegmentAABB(step2.waypoint, Point(-4.95, -3.61), expanded));
}

// rosbagシナリオの再現: 味方PA(right側)でのケース
// ロボット(5.318, -3.763)からターゲット(5.194, 3.243)への2ステップルーティング
// Step1: BOTTOM(中間WP)選択 → Step2: TOP選択でターゲットへ到達
TEST(PenaltyAvoidanceHelperTest, RosbagScenario_RobotBelowPA_TargetAbovePA)
{
  const Box our_pa(Point(4.2, -1.8), Point(6.0, 1.8));
  const Point goal(6.0, 0.0);
  const Point pa_size(1.8, 3.6);
  Box expanded = our_pa;
  expanded.min_corner() -= Point(0.3, 0.3);
  expanded.max_corner() += Point(0.3, 0.3);

  // Step 1: PA 下方から横断 → current→TOP 不達のため BOTTOM 選択
  const auto step1 = computePenaltyBypassDecision(
    Point(5.318, -3.763), Point(5.194, 3.243), our_pa, goal, pa_size, 0.3, 0.2, true);
  ASSERT_TRUE(step1.crossing_detected);
  ASSERT_TRUE(step1.target_overridden);
  EXPECT_EQ(step1.selected_side, PenaltyBypassSide::BOTTOM);
  // current → BOTTOM ウェイポイントは PA を横断しない
  EXPECT_FALSE(intersectsSegmentAABB(Point(5.318, -3.763), step1.waypoint, expanded));

  // Step 2: BOTTOM ウェイポイントから再計画 → TOP fully_good → TOP 選択
  const auto step2 = computePenaltyBypassDecision(
    step1.waypoint, Point(5.194, 3.243), our_pa, goal, pa_size, 0.3, 0.2, true);
  ASSERT_TRUE(step2.crossing_detected);
  EXPECT_EQ(step2.selected_side, PenaltyBypassSide::TOP);
  // TOP ウェイポイント → ターゲットは PA を横断しない
  EXPECT_FALSE(intersectsSegmentAABB(step2.waypoint, Point(5.194, 3.243), expanded));
}

}  // namespace crane
