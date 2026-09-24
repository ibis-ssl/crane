// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_physics/pass_interception.hpp>

namespace crane
{
TEST(PassInterception, IncludesBlockerNearKicker)
{
  EXPECT_LT(
    straightPassInterceptionSlack(
      Point(0, 0), Point(4, 0), {3.0, 0.7}, Point(0.4, 0), Vector2::Zero(), 3.0, 5.5),
    0.0);
}

TEST(PassInterception, IncludesDefenderBeyondReceiver)
{
  EXPECT_LT(
    straightPassInterceptionSlack(
      Point(0, 0), Point(4, 0), {3.0, 0.7}, Point(4.2, 0), Vector2::Zero(), 3.0, 5.5),
    0.0);
}

TEST(PassInterception, DecelerationReducesSafetyMargin)
{
  const auto slack = [](double deceleration) {
    return straightPassInterceptionSlack(
      Point(0, 0), Point(4, 0), {3.0, deceleration}, Point(3, 2), Vector2::Zero(), 3.0, 5.5);
  };
  EXPECT_LT(slack(0.7), slack(0.0));
}

TEST(PassInterception, HigherKickSpeedImprovesSafetyMargin)
{
  const auto slack = [](double speed) {
    return straightPassInterceptionSlack(
      Point(0, 0), Point(4, 0), {speed, 0.7}, Point(3, 2), Vector2::Zero(), 3.0, 5.5);
  };
  EXPECT_GT(slack(5.0), slack(3.0));
}

TEST(PassInterception, DistantEnemyCannotIntercept)
{
  EXPECT_GT(
    straightPassInterceptionSlack(
      Point(0, 0), Point(4, 0), {3.0, 0.7}, Point(2, 6), Vector2::Zero(), 3.0, 5.5),
    0.0);
}

TEST(PassInterception, UnreachableBallIsNotSafe)
{
  EXPECT_LT(
    straightPassInterceptionSlack(
      Point(0, 0), Point(4, 0), {1.0, 0.7}, Point(2, 6), Vector2::Zero(), 3.0, 5.5),
    0.0);
}
}  // namespace crane
