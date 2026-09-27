// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_game_analyzer/threat_evaluator.hpp>
#include <initializer_list>
#include <vector>

namespace crane
{
namespace
{
auto makeThreats(std::initializer_list<double> ratings) -> std::vector<RobotThreat>
{
  std::vector<RobotThreat> threats;
  for (double rating : ratings) {
    RobotThreat threat;
    threat.threat_rating = rating;
    threats.push_back(threat);
  }
  return threats;
}

auto recommend(const std::vector<RobotThreat> & threats, int available_robots) -> int
{
  return ThreatEvaluator{}.calculateRecommendedDefenders(BallThreat{}, threats, available_robots);
}
}  // namespace

TEST(RecommendedDefendersTest, NoThreatStillAssignsOneForBall) { EXPECT_EQ(recommend({}, 8), 1); }

TEST(RecommendedDefendersTest, CountsOnlyThreatsAboveHalf)
{
  // 0.5 ちょうどは高脅威に数えない
  EXPECT_EQ(recommend(makeThreats({0.9, 0.51, 0.5, 0.2}), 8), 3);
}

TEST(RecommendedDefendersTest, CappedAtHalfOfAvailableRobots)
{
  EXPECT_EQ(recommend(makeThreats({0.9, 0.9, 0.9, 0.9, 0.9}), 6), 3);
  EXPECT_EQ(recommend(makeThreats({0.9, 0.9, 0.9, 0.9, 0.9}), 7), 3);
}

TEST(RecommendedDefendersTest, AtLeastOneEvenWithFewRobots)
{
  EXPECT_EQ(recommend(makeThreats({0.9, 0.9}), 1), 1);
  EXPECT_EQ(recommend(makeThreats({0.9, 0.9}), 0), 1);
}

}  // namespace crane
