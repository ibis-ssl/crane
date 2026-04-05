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

// 再横断チェック: ターゲットがPA内x範囲でPA上方にある場合、TOP側が選択される
// (コスト最小はBOTTOMだがBOTTOMからターゲットへの経路がPAを再横断するため)
TEST(PenaltyAvoidanceHelperTest, ReCrossingCheckSelectsCorrectSideWhenTargetIsAbovePA)
{
  // ターゲット: PAのx範囲内(x=-4.95)、y上方(3.08) → PA再横断が起きるケース
  // ロボット: PA下方(-3.61) → コストだけならBOTTOMが安い可能性
  const auto decision = computePenaltyBypassDecision(
    Point(-4.95, -3.61), Point(-4.75, 3.08), makeOurPA(), ourGoal(), paSize(), 0.3, 0.2, true);
  ASSERT_TRUE(decision.crossing_detected);
  // ウェイポイントからターゲットへの経路がPAを再横断しないことを確認
  Box expanded = makeOurPA();
  expanded.min_corner() -= Point(0.3, 0.3);
  expanded.max_corner() += Point(0.3, 0.3);
  EXPECT_FALSE(intersectsSegmentAABB(decision.waypoint, Point(-4.75, 3.08), expanded));
}

// 再横断チェック: ロボットPA上方・ターゲットPA下方 → BOTTOM側が選択される
TEST(PenaltyAvoidanceHelperTest, ReCrossingCheckSelectsCorrectSideWhenTargetIsBelowPA)
{
  const auto decision = computePenaltyBypassDecision(
    Point(-4.75, 3.08), Point(-4.95, -3.61), makeOurPA(), ourGoal(), paSize(), 0.3, 0.2, true);
  ASSERT_TRUE(decision.crossing_detected);
  Box expanded = makeOurPA();
  expanded.min_corner() -= Point(0.3, 0.3);
  expanded.max_corner() += Point(0.3, 0.3);
  EXPECT_FALSE(intersectsSegmentAABB(decision.waypoint, Point(-4.95, -3.61), expanded));
}

// rosbagシナリオの再現: 味方PA(xright側)での同等ケース
// ロボット(5.318, -3.763)からターゲット(5.194, 3.243)へ横断
// PAをゴール右側として設定、選択ウェイポイントが再横断しないことを確認
TEST(PenaltyAvoidanceHelperTest, RosbagScenario_RobotBelowPA_TargetAbovePA)
{
  // 味方PA: x=[4.2, 6.0], y=[-1.8, 1.8]、ゴール=(6.0, 0)
  const Box our_pa(Point(4.2, -1.8), Point(6.0, 1.8));
  const Point goal(6.0, 0.0);
  const Point pa_size(1.8, 3.6);

  const auto decision = computePenaltyBypassDecision(
    Point(5.318, -3.763), Point(5.194, 3.243), our_pa, goal, pa_size, 0.3, 0.2, true);

  ASSERT_TRUE(decision.crossing_detected);
  ASSERT_TRUE(decision.target_overridden);

  // ウェイポイントからターゲットへの経路がPAを再横断しないこと
  Box expanded = our_pa;
  expanded.min_corner() -= Point(0.3, 0.3);
  expanded.max_corner() += Point(0.3, 0.3);
  EXPECT_FALSE(intersectsSegmentAABB(decision.waypoint, Point(5.194, 3.243), expanded));

  // ターゲットがPA上方なのでTOP側が選択されること
  EXPECT_EQ(decision.selected_side, PenaltyBypassSide::TOP);
}

}  // namespace crane
