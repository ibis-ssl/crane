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
    Point(-2.0, 2.5), Point(-1.0, 2.5), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2, std::nullopt,
    0.25, true);
  EXPECT_FALSE(decision.crossing_detected);
  EXPECT_FALSE(decision.target_overridden);
}

TEST(PenaltyAvoidanceHelperTest, CrossingOverridesTargetWithWaypoint)
{
  const auto decision = computePenaltyBypassDecision(
    Point(-3.0, 0.0), Point(-5.9, 0.0), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2, std::nullopt,
    0.25, true);
  EXPECT_TRUE(decision.crossing_detected);
  EXPECT_TRUE(decision.target_overridden);
  EXPECT_NEAR(decision.waypoint.x(), -4.0, 1e-6);
  EXPECT_NEAR(std::abs(decision.waypoint.y()), 2.0, 1e-6);
}

TEST(PenaltyAvoidanceHelperTest, LockedSideIsKeptWithinSwitchMargin)
{
  const auto decision = computePenaltyBypassDecision(
    Point(-3.5, 0.0), Point(-5.0, 0.0), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2,
    PenaltyBypassSide::BOTTOM, 5.0, true);
  EXPECT_TRUE(decision.crossing_detected);
  EXPECT_EQ(decision.selected_side, PenaltyBypassSide::BOTTOM);
  EXPECT_TRUE(decision.used_locked_side);
}

TEST(PenaltyAvoidanceHelperTest, BetterSideOverridesLockWhenGapIsLarge)
{
  const auto decision = computePenaltyBypassDecision(
    Point(-5.5, 2.9), Point(-5.7, -2.9), makeOurPA(), ourGoal(), paSize(), 0.1, 0.2,
    PenaltyBypassSide::BOTTOM, 0.1, true);
  EXPECT_TRUE(decision.crossing_detected);
  EXPECT_FALSE(decision.used_locked_side);
  EXPECT_EQ(decision.selected_side, PenaltyBypassSide::TOP);
}

}  // namespace crane
