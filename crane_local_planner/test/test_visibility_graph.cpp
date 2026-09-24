// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
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

// ---- サブゴール退化（2026-09-20 bag, robot 3）の再発防止 ----

TEST(VisibilityGraphTest, FirstWaypointBeyondSkipsDegenerateProjectionPoint)
{
  // path[1] が現在位置から 1mm の射影点。これを目標にすると終端速度だけが跳ねる
  const std::vector<Point> path{
    Point(0.0, 0.0), Point(0.001, 0.0), Point(0.5, 0.3), Point(1.0, 0.3)};
  const auto choice = firstWaypointBeyond(path, 0.05);
  EXPECT_EQ(choice.index, 2U);
  EXPECT_NEAR(choice.arc_length, 0.001 + std::hypot(0.499, 0.3), 1e-9);
}

TEST(VisibilityGraphTest, FirstWaypointBeyondKeepsSufficientlyFarFirstWaypoint)
{
  const std::vector<Point> path{Point(0.0, 0.0), Point(0.2, 0.0), Point(1.0, 0.0)};
  const auto choice = firstWaypointBeyond(path, 0.05);
  EXPECT_EQ(choice.index, 1U);
  EXPECT_NEAR(choice.arc_length, 0.2, 1e-9);
}

TEST(VisibilityGraphTest, FirstWaypointBeyondFallsBackToGoalOnShortPath)
{
  const std::vector<Point> path{Point(0.0, 0.0), Point(0.01, 0.0), Point(0.02, 0.0)};
  const auto choice = firstWaypointBeyond(path, 0.05);
  EXPECT_EQ(choice.index, 2U);
  EXPECT_NEAR(choice.arc_length, 0.02, 1e-9);

  const std::vector<Point> single{Point(0.0, 0.0)};
  EXPECT_EQ(firstWaypointBeyond(single, 0.05).index, 0U);
  EXPECT_EQ(firstWaypointBeyond({}, 0.05).index, 0U);
}

TEST(VisibilityGraphTest, PlanFromInsideCapsuleYieldsMillimetreSecondWaypoint)
{
  // plan() は始点を障害物の外へ押し出し、元の始点を先頭に残す。1mm 食い込んだ始点からは
  // path[1] が 2mm 先の点になる。この退化を firstWaypointBeyond が読み飛ばせること。
  VisibilityGraph graph;
  const std::vector<Obstacle> obstacles = {
    Obstacle::makeCapsule(Capsule{Segment(Point(0.8, -0.5), Point(0.8, 0.5)), 0.2}, true)};
  const auto path = graph.plan(Point(0.601, 0.0), Point(2.0, 0.0), obstacles);
  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->size(), 3U);
  EXPECT_LT(((*path)[1] - (*path)[0]).norm(), 0.01);
  EXPECT_GE(firstWaypointBeyond(*path, 0.05).index, 2U);
}

TEST(VisibilityGraphTest, TrimPathFromCurrentInsertsProjectionAndDropsPassedWaypoints)
{
  const std::vector<Point> path{Point(0.0, 0.0), Point(1.0, 0.0), Point(1.0, 1.0)};
  const auto trimmed = trimPathFromCurrent(Point(0.5, 0.1), path);
  ASSERT_EQ(trimmed.size(), 4U);
  EXPECT_NEAR(trimmed[0].x(), 0.5, 1e-9);
  EXPECT_NEAR(trimmed[1].x(), 0.5, 1e-9);
  EXPECT_NEAR(trimmed[1].y(), 0.0, 1e-9);
  EXPECT_NEAR(trimmed[2].x(), 1.0, 1e-9);
  EXPECT_NEAR(trimmed[3].y(), 1.0, 1e-9);

  // 経路上にいるときは射影点を挿入しない
  const auto on_path = trimPathFromCurrent(Point(0.5, 0.0), path);
  ASSERT_EQ(on_path.size(), 3U);
  EXPECT_NEAR(on_path[1].x(), 1.0, 1e-9);

  EXPECT_TRUE(trimPathFromCurrent(Point(0.0, 0.0), {Point(0.0, 0.0)}).empty());
}

TEST(VisibilityGraphTest, PointAtDistanceClampsToGoal)
{
  const std::vector<Point> path{Point(0.0, 0.0), Point(1.0, 0.0), Point(1.0, 1.0)};
  EXPECT_NEAR(pointAtDistance(path, 0.5).x(), 0.5, 1e-9);
  EXPECT_NEAR(pointAtDistance(path, 1.5).y(), 0.5, 1e-9);
  EXPECT_NEAR(pointAtDistance(path, 5.0).y(), 1.0, 1e-9);
  EXPECT_NEAR(pointAtDistance({}, 1.0).norm(), 0.0, 1e-9);
}

// ---- 退避判定は自機速度で膨らんだカプセルに反応しない ----

TEST(VisibilityGraphTest, MakeCapsuleDefaultsEscapeShapeToPlanningShape)
{
  const Capsule capsule{Segment(Point(-0.5, 0.0), Point(0.5, 0.0)), 0.2};
  const auto obstacle = Obstacle::makeCapsule(capsule, true);
  EXPECT_NEAR((obstacle.escape_capsule.segment.first - capsule.segment.first).norm(), 0.0, 1e-9);
  EXPECT_NEAR((obstacle.escape_capsule.segment.second - capsule.segment.second).norm(), 0.0, 1e-9);
  EXPECT_NEAR(obstacle.escape_capsule.radius, capsule.radius, 1e-9);
}

TEST(VisibilityGraphTest, EscapeShapeIgnoresEgoVelocity)
{
  // 自機 1 m/s で静止ロボットへ接近。経路計画用カプセルは自機側へ 0.5 m 伸びるが、
  // 退避判定用の形状は伸びない
  const auto obstacle = makePredictedRobotObstacle(
    Vector2(1.0, 0.0), 0.09, Point(0.5, 0.0), Vector2::Zero(), 0.09, 0.5, 0.03);
  EXPECT_NEAR(obstacle.capsule.segment.second.x(), 0.0, 1e-9);
  EXPECT_NEAR(obstacle.escape_capsule.segment.second.x(), 0.5, 1e-9);
  EXPECT_NEAR(obstacle.escape_capsule.radius, 0.21, 1e-9);

  VisibilityGraph graph;
  // 経路計画上は食い込み（迂回経路が必要）のまま
  EXPECT_LT(obstacle.signedDistance(Point(0.0, 0.0)), 0.0);
  // しかし退避はしない
  EXPECT_FALSE(graph.nearestDynamicEscape(Point(0.0, 0.0), {obstacle}).has_value());
}

TEST(VisibilityGraphTest, EscapeStillFiresWhenOtherRobotChargesAtUs)
{
  const auto obstacle = makePredictedRobotObstacle(
    Vector2::Zero(), 0.09, Point(0.5, 0.0), Vector2(-1.0, 0.0), 0.09, 0.5, 0.03);
  VisibilityGraph graph;
  const auto escape = graph.nearestDynamicEscape(Point(0.0, 0.05), {obstacle});
  ASSERT_TRUE(escape.has_value());
  EXPECT_GT(Obstacle::makeCapsule(obstacle.escape_capsule, true).signedDistance(*escape), 0.0);
}

TEST(VisibilityGraphTest, EscapeFiresOnActualFootprintOverlap)
{
  const auto obstacle = makePredictedRobotObstacle(
    Vector2(1.0, 0.0), 0.09, Point(0.5, 0.0), Vector2::Zero(), 0.09, 0.5, 0.03);
  VisibilityGraph graph;
  const auto escape = graph.nearestDynamicEscape(Point(0.4, 0.0), {obstacle});
  ASSERT_TRUE(escape.has_value());
  EXPECT_LT(escape->x(), 0.4);
  EXPECT_GE(Obstacle::makeCapsule(obstacle.escape_capsule, true).signedDistance(*escape), 0.0);
}

TEST(VisibilityGraphTest, EscapeReleaseMarginKeepsEscapingUntilClear)
{
  VisibilityGraph graph;
  const std::vector<Obstacle> obstacles = {
    Obstacle::makeCapsule(Capsule{Segment(Point(-0.5, 0.0), Point(0.5, 0.0)), 0.2}, true)};
  const Point just_outside(0.0, 0.21);
  EXPECT_FALSE(graph.nearestDynamicEscape(just_outside, obstacles).has_value());
  const auto escape = graph.nearestDynamicEscape(just_outside, obstacles, 0.03);
  ASSERT_TRUE(escape.has_value());
  EXPECT_NEAR(escape->x(), 0.0, 1e-6);
  EXPECT_GT(escape->y(), 0.23);
}

TEST(VisibilityGraphTest, EscapeClearanceLeavesRoomBeyondReleaseMargin)
{
  VisibilityGraph graph;
  const std::vector<Obstacle> obstacles = {
    Obstacle::makeCapsule(Capsule{Segment(Point(-0.5, 0.0), Point(0.5, 0.0)), 0.2}, true)};
  // 退避中（解除余裕 0.03）に境界から 0.01 外側にいる。退避先が解除線 0.23 とほぼ同じ距離だと
  // 到達許容誤差（CM4 は 0.01）の内側で止まった機体が解除線を越えられず、退避が固着する
  const auto escape = graph.nearestDynamicEscape(Point(0.0, 0.21), obstacles, 0.03, 0.06);
  ASSERT_TRUE(escape.has_value());
  EXPECT_NEAR(escape->x(), 0.0, 1e-6);
  EXPECT_GE(escape->y(), 0.2 + 0.03 + 0.01 + 0.02);
  // escape_clearance を解除余裕より小さく渡しても、解除余裕の外側までは出す
  const auto minimal = graph.nearestDynamicEscape(Point(0.0, 0.21), obstacles, 0.03, 0.0);
  ASSERT_TRUE(minimal.has_value());
  EXPECT_GT(minimal->y(), 0.23);
}

TEST(VisibilityGraphTest, SelectSubgoalUsesLookaheadWhenVisible)
{
  VisibilityGraph graph;
  const std::vector<Point> path{Point(0.0, 0.0), Point(1.0, 0.0)};
  const auto choice = selectSubgoal(graph, path.front(), path, {}, 0.3, 0.05);
  EXPECT_EQ(choice.mode, SubgoalMode::LOOKAHEAD);
  EXPECT_NEAR(choice.point.x(), 0.3, 1e-9);
  EXPECT_NEAR(choice.point.y(), 0.0, 1e-9);
  EXPECT_NEAR(choice.arc_length, 0.3, 1e-9);
}

TEST(VisibilityGraphTest, SelectSubgoalSkipsDegenerateWaypointWhenNextIsVisible)
{
  VisibilityGraph graph;
  // 円は先読み点への弦だけを遮り、1mm 先の path[1] を飛ばした path[2] への弦は遮らない
  const std::vector<Obstacle> obstacles = {Obstacle::makeCircle(Point(0.4, 0.13), 0.05)};
  const std::vector<Point> path{
    Point(0.0, 0.0), Point(0.001, 0.0), Point(0.5, 0.3), Point(1.0, 0.3)};
  ASSERT_TRUE(graph.isPathVisible(path, obstacles));
  ASSERT_FALSE(graph.isPathVisible({path.front(), pointAtDistance(path, 0.8)}, obstacles));
  const auto choice = selectSubgoal(graph, path.front(), path, obstacles, 0.8, 0.05);
  EXPECT_EQ(choice.mode, SubgoalMode::WAYPOINT_SKIP);
  EXPECT_NEAR(choice.point.x(), 0.5, 1e-9);
  EXPECT_NEAR(choice.point.y(), 0.3, 1e-9);
  EXPECT_NEAR(choice.arc_length, 0.001 + std::hypot(0.499, 0.3), 1e-9);
  EXPECT_TRUE(graph.isPathVisible({path.front(), choice.point}, obstacles));
}

TEST(VisibilityGraphTest, SelectSubgoalDoesNotCutBoxCornerWhenSkippingProjectionPoint)
{
  VisibilityGraph graph;
  const std::vector<Obstacle> obstacles = {
    Obstacle::makeBox(Box(Point(0.0, 0.0), Point(1.0, 1.0)))};
  // 箱の左側 (x<0) にいて、保持経路（箱の上辺 y=1.01 沿い）から 3cm 横にずれている。
  // path[1] は射影点。これを飛ばして path[2] へ直接向かうと弦が箱の角 (0,1) を切る
  const Point current(-0.04, 0.98);
  const std::vector<Point> path{current, Point(-0.04, 1.01), Point(1.01, 1.01), Point(1.5, 0.5)};
  ASSERT_TRUE(graph.isPathVisible(path, obstacles));
  ASSERT_FALSE(graph.isPathVisible({current, path[2]}, obstacles));
  const auto choice = selectSubgoal(graph, current, path, obstacles, 0.3, 0.05);
  EXPECT_EQ(choice.mode, SubgoalMode::PATH_POINT);
  EXPECT_NEAR(choice.arc_length, 0.05, 1e-9);
  EXPECT_NEAR(choice.point.x(), -0.02, 1e-9);
  EXPECT_NEAR(choice.point.y(), 1.01, 1e-9);
  EXPECT_TRUE(graph.isPathVisible({current, choice.point}, obstacles));
}

}  // namespace crane::visibility_graph
