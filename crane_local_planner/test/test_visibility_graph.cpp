// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_local_planner/visibility_graph.hpp>

namespace crane::visibility_graph
{

TEST(VisibilityGraphTest, DirectPathWithoutObstacles)
{
  VisibilityGraph graph;
  const auto path = graph.plan(Point(0.0, 0.0), Point(2.0, 0.0), {});
  ASSERT_TRUE(path.has_value());
  ASSERT_EQ(path->size(), 2U);
  EXPECT_NEAR(path->front().x(), 0.0, 1e-9);
  EXPECT_NEAR(path->back().x(), 2.0, 1e-9);
  EXPECT_NEAR(pathLength(*path), 2.0, 1e-9);
}

TEST(VisibilityGraphTest, DetoursAroundCircle)
{
  VisibilityGraph graph;
  const std::vector<Obstacle> obstacles = {Obstacle::makeCircle(Point(1.0, 0.0), 0.3)};
  const auto path = graph.plan(Point(0.0, 0.0), Point(2.0, 0.0), obstacles);
  ASSERT_TRUE(path.has_value());
  EXPECT_GT(path->size(), 2U);
  EXPECT_GT(pathLength(*path), 2.0);
  EXPECT_TRUE(graph.isPathVisible(*path, obstacles));
}

TEST(VisibilityGraphTest, RelativeVelocityCapsuleRequiresDetour)
{
  VisibilityGraph graph;
  const Capsule capsule{Segment(Point(0.8, -0.5), Point(0.8, 0.5)), 0.2};
  const std::vector<Obstacle> obstacles = {Obstacle::makeCapsule(capsule, true)};
  const auto path = graph.plan(Point(0.0, 0.0), Point(2.0, 0.0), obstacles);
  ASSERT_TRUE(path.has_value());
  EXPECT_GT(path->size(), 2U);
  EXPECT_TRUE(graph.isPathVisible(*path, obstacles));
}

TEST(VisibilityGraphTest, PredictedCapsuleUsesRelativeVelocityAndBothRadii)
{
  const auto obstacle = makePredictedRobotObstacle(
    Vector2(1.0, 0.0), 0.09, Point(2.0, 1.0), Vector2(-1.0, 0.5), 0.08, 0.5, 0.03);
  ASSERT_EQ(obstacle.type, Obstacle::Type::CAPSULE);
  EXPECT_TRUE(obstacle.is_dynamic_robot);
  EXPECT_NEAR(obstacle.capsule.segment.first.x(), 2.0, 1e-9);
  EXPECT_NEAR(obstacle.capsule.segment.second.x(), 1.0, 1e-9);
  EXPECT_NEAR(obstacle.capsule.segment.second.y(), 1.25, 1e-9);
  EXPECT_NEAR(obstacle.capsule.radius, 0.20, 1e-9);
}

TEST(VisibilityGraphTest, EscapesToNearestOutsidePointWhenInsideDynamicCapsule)
{
  VisibilityGraph graph;
  const auto capsule =
    Obstacle::makeCapsule(Capsule{Segment(Point(-0.5, 0.0), Point(0.5, 0.0)), 0.2}, true);
  const std::vector<Obstacle> obstacles = {capsule};
  const auto escape = graph.nearestDynamicEscape(Point(0.0, 0.05), obstacles);
  ASSERT_TRUE(escape.has_value());
  EXPECT_GT(capsule.signedDistance(*escape), 0.0);
  EXPECT_NEAR(escape->x(), 0.0, 1e-6);
  EXPECT_GT(escape->y(), 0.2);
}

TEST(VisibilityGraphTest, DoesNotMoveTowardCapsuleWhenAlreadyOutside)
{
  VisibilityGraph graph;
  const std::vector<Obstacle> obstacles = {
    Obstacle::makeCapsule(Capsule{Segment(Point(-0.5, 0.0), Point(0.5, 0.0)), 0.2}, true)};
  EXPECT_FALSE(graph.nearestDynamicEscape(Point(0.0, 0.5), obstacles).has_value());
}

TEST(VisibilityGraphTest, ReusesSafePathBeforePeriodicReplan)
{
  EXPECT_EQ(decideReplanAction(true, false, false), ReplanAction::REUSE_RETAINED_PATH);
}

TEST(VisibilityGraphTest, ReplansImmediatelyWhenRetainedPathBecomesUnsafeOrGoalChanges)
{
  EXPECT_EQ(decideReplanAction(false, false, false), ReplanAction::RUN_FULL_REPLAN);
}

TEST(VisibilityGraphTest, ReturnsImmediatelyToDirectPathWhenItBecomesVisible)
{
  EXPECT_EQ(decideReplanAction(true, true, false), ReplanAction::USE_DIRECT_PATH);
}

TEST(VisibilityGraphTest, PeriodicallyReplansAStillSafeDetour)
{
  EXPECT_EQ(decideReplanAction(true, false, true), ReplanAction::RUN_FULL_REPLAN);
}

TEST(VisibilityGraphTest, DetoursPenaltyAreaInFrontOfGoal)
{
  VisibilityGraph graph;
  constexpr double FAR = 20.0;
  const double half_width = 4.7;
  const double half_height = 3.2;

  // フィールド境界
  const std::vector<Obstacle> boundary_obstacles = {
    Obstacle::makeBox(Box(Point(-FAR, half_height), Point(FAR, FAR))),
    Obstacle::makeBox(Box(Point(-FAR, -FAR), Point(FAR, -half_height))),
    Obstacle::makeBox(Box(Point(half_width, -FAR), Point(FAR, FAR))),
    Obstacle::makeBox(Box(Point(-FAR, -FAR), Point(-half_width, FAR))),
  };

  // ゴール裏側（-x方向）を -FAR まで拡張した自陣ペナルティエリア障害物
  // ゴール位置: (-4.5, 0.0), PA前面: -3.5 + 0.1 = -3.4, PA幅: [-1.1, 1.1]
  const Box extended_pa(Point(-FAR, -1.1), Point(-3.4, 1.1));
  auto obstacles = boundary_obstacles;
  obstacles.push_back(Obstacle::makeBox(extended_pa));

  // ペナルティエリアの脇同士を結ぶ経路（(-4.2, 1.5) -> (-4.2, -1.5)）
  const auto path = graph.plan(Point(-4.2, 1.5), Point(-4.2, -1.5), obstacles);
  ASSERT_TRUE(path.has_value());

  // ゴールの後ろ (x < -4.5) を通るノードが一切含まれず、
  // ペナルティエリア前方 (x >= -3.4) を迂回すること
  bool detoured_front = false;
  for (const auto & pt : *path) {
    EXPECT_GE(pt.x(), -4.5);
    if (pt.x() >= -3.4 - 1e-3) {
      detoured_front = true;
    }
  }
  EXPECT_TRUE(detoured_front);
  EXPECT_TRUE(graph.isPathVisible(*path, obstacles));
}

}  // namespace crane::visibility_graph
