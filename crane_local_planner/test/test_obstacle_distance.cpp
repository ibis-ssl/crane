// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>

#include "crane_local_planner/ateb_types.hpp"
#include "test_helpers.hpp"

using crane::Box;
using crane::Capsule;
using crane::Circle;
using crane::Point;
using crane::Vector2;
using crane::ateb::Obstacle;

// ===== BOX距離テスト =====

TEST(ObstacleDistance, BoxDistanceOutsideEdge)
{
  // ペナルティエリアの左辺（x=4.2）から外側1mの点
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // x=3.2: BOX左辺(x=4.2)から1.0m外側
  const Point p(3.2, 0.0);
  EXPECT_NEAR(obs.distance(p), 1.0, 1e-9);
}

TEST(ObstacleDistance, BoxDistanceOutsideCorner)
{
  // ペナルティエリアの左上角から斜め外側
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 角(4.2, 1.8)から(1.0, 1.0)だけ外側
  const Point p(3.2, 2.8);
  EXPECT_NEAR(obs.distance(p), std::sqrt(2.0), 1e-9);
}

TEST(ObstacleDistance, BoxDistanceInside)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // ペナルティエリア内部の中心近く
  // BOX: x=[4.2, 6.0], y=[-1.8, 1.8]
  // 点(5.0, 0.0): 左辺から0.8m, 右辺から1.0m, 下辺から1.8m, 上辺から1.8m
  // 最近辺: 左辺(0.8m) → 符号付き距離 = -0.8
  const Point p(5.0, 0.0);
  EXPECT_LT(obs.distance(p), 0.0) << "内部の距離は負であるべき";
  EXPECT_NEAR(obs.distance(p), -0.8, 1e-9);
}

TEST(ObstacleDistance, BoxDistanceInsideNearCorner)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 左上角(4.2, 1.8)の近く、0.05m内側
  // 点(4.25, 1.75): 左辺から0.05m, 上辺から0.05m → 最近辺は等距離
  const Point p(4.25, 1.75);
  EXPECT_LT(obs.distance(p), 0.0) << "内部の距離は負であるべき";
  EXPECT_NEAR(obs.distance(p), -0.05, 1e-9);
}

TEST(ObstacleDistance, BoxDistanceOnEdge)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 左辺上の点
  const Point p(4.2, 0.0);
  EXPECT_NEAR(obs.distance(p), 0.0, 1e-9);
}

TEST(ObstacleDistance, BoxDistanceOnCorner)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 左上角ちょうど
  const Point p(4.2, 1.8);
  EXPECT_NEAR(obs.distance(p), 0.0, 1e-9);
}

// ===== BOX勾配テスト =====

TEST(ObstacleDistance, BoxGradientOutsideEdge)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 左辺の外側 → 勾配は(-1, 0)（左向き）
  const Point p(3.2, 0.0);
  const Vector2 grad = obs.distanceGradient(p);
  EXPECT_NEAR(grad.x(), -1.0, 1e-9);
  EXPECT_NEAR(grad.y(), 0.0, 1e-9);
}

TEST(ObstacleDistance, BoxGradientOutsideCorner)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 左上角から等距離外側 → 勾配は(-1/√2, 1/√2)
  const Point p(3.2, 2.8);
  const Vector2 grad = obs.distanceGradient(p);
  EXPECT_NEAR(grad.x(), -1.0 / std::sqrt(2.0), 1e-6);
  EXPECT_NEAR(grad.y(), 1.0 / std::sqrt(2.0), 1e-6);
}

TEST(ObstacleDistance, BoxGradientInsideNearLeftFace)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 内部の左辺近く（左辺が最近辺）→ 勾配は(-1, 0)（左向き外向き）
  // 点(4.22, 0.0): 左辺から0.02m, 他辺から0.78m以上
  const Point p(4.22, 0.0);
  const Vector2 grad = obs.distanceGradient(p);
  EXPECT_NEAR(grad.x(), -1.0, 1e-9);
  EXPECT_NEAR(grad.y(), 0.0, 1e-9);
}

TEST(ObstacleDistance, BoxGradientInsideNearTopFace)
{
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  // 内部の上辺近く → 勾配は(0, 1)（上向き外向き）
  const Point p(5.0, 1.75);
  const Vector2 grad = obs.distanceGradient(p);
  EXPECT_NEAR(grad.x(), 0.0, 1e-9);
  EXPECT_NEAR(grad.y(), 1.0, 1e-9);
}

TEST(ObstacleDistance, BoxGradientIsUnitVector)
{
  // 勾配は常に単位ベクトルであるべき
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  const std::vector<Point> test_points = {
    {3.0, 0.0},   // 左辺外
    {3.0, 3.0},   // 左上角外
    {5.0, 3.0},   // 上辺外
    {7.0, 2.5},   // 右上角外
    {4.5, 0.0},   // 内部中央
    {4.22, 0.0},  // 内部左近く
    {5.9, 0.0},   // 内部右近く
  };
  for (const auto & p : test_points) {
    const Vector2 grad = obs.distanceGradient(p);
    EXPECT_NEAR(grad.norm(), 1.0, 1e-9) << "点(" << p.x() << ", " << p.y() << ")での勾配ノルム";
  }
}

TEST(ObstacleDistance, BoxGradientNumericalConsistency)
{
  // 有限差分との一致検証（distance()とdistanceGradient()の整合性）
  const auto obs = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  constexpr double eps = 1e-5;
  const std::vector<Point> test_points = {
    {3.0, 0.0},   // 辺外
    {3.0, 3.0},   // 角外
    {5.0, 3.0},   // 辺外（上）
    {4.5, 0.0},   // 内部
    {4.22, 0.0},  // 内部、左辺近く
    {5.9, 0.0},   // 内部、右辺近く
  };
  for (const auto & p : test_points) {
    const Vector2 grad = obs.distanceGradient(p);
    const double fd_x =
      (obs.distance(p + Point(eps, 0.0)) - obs.distance(p - Point(eps, 0.0))) / (2.0 * eps);
    const double fd_y =
      (obs.distance(p + Point(0.0, eps)) - obs.distance(p - Point(0.0, eps))) / (2.0 * eps);
    EXPECT_NEAR(grad.x(), fd_x, 1e-4) << "x勾配の不整合 at (" << p.x() << ", " << p.y() << ")";
    EXPECT_NEAR(grad.y(), fd_y, 1e-4) << "y勾配の不整合 at (" << p.x() << ", " << p.y() << ")";
  }
}

// ===== BOX膨張テスト =====

TEST(ObstacleDistance, BoxInflatedCorrectly)
{
  const auto base = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  constexpr double margin = 0.1;
  const auto inflated = base.inflated(margin);
  // 膨張後: x=[4.1, 6.1], y=[-1.9, 1.9]
  EXPECT_NEAR(inflated.box.min_corner().x(), 4.2 - margin, 1e-9);
  EXPECT_NEAR(inflated.box.min_corner().y(), -1.8 - margin, 1e-9);
  EXPECT_NEAR(inflated.box.max_corner().x(), 6.0 + margin, 1e-9);
  EXPECT_NEAR(inflated.box.max_corner().y(), 1.8 + margin, 1e-9);
}

TEST(ObstacleDistance, BoxInflatedDistanceConsistent)
{
  const auto base = Obstacle::makeBox(test_helpers::makeOurPenaltyBox());
  constexpr double margin = 0.1;
  const auto inflated = base.inflated(margin);
  // 膨張前にmarginだけ外側の点は、膨張後に境界上になる
  const Point p(4.1, 0.0);  // 膨張後の左辺
  EXPECT_NEAR(inflated.distance(p), 0.0, 1e-9);
  // 元の障害物ではmarginだけ外側
  EXPECT_NEAR(base.distance(p), margin, 1e-9);
}

// ===== Circle距離テスト（バグ1検出）=====
//
// 問題: bg::distance(p, circle) はCircleをpoint_tagとして扱うため、
// 中心からの距離のみ返し、radiusを引かない。
// カスタムオーバーロード distance(Circle, Geometry1) は
// Circleが第1引数の場合にのみマッチするため、
// distance(p, circle) のときはマッチしない。

TEST(ObstacleDistance, CircleDistanceOutside)
{
  // ロボット半径0.09mの円、中心原点
  const auto obs = Obstacle::makeCircle(Point::Zero(), test_helpers::kRobotRadius);
  // 表面から1m外側の点: (1.09, 0)
  const Point p(test_helpers::kRobotRadius + 1.0, 0.0);
  // 期待値: 1.0m（表面からの距離）
  // [バグ1] 実際には1.09m（中心からの距離）を返す
  EXPECT_NEAR(obs.distance(p), 1.0, 1e-9) << "[バグ1] Circle distance()がradiusを引いていない。"
                                             "中心距離1.09を返すため失敗する";
}

TEST(ObstacleDistance, CircleDistanceInside)
{
  // ロボット半径0.09mの円、中心原点
  const auto obs = Obstacle::makeCircle(Point::Zero(), test_helpers::kRobotRadius);
  // 円内部の点: (0.05, 0) → 表面まで0.04m
  const Point p(0.05, 0.0);
  // 期待値: 負（内部）
  // [バグ1] 実際には正（0.05）を返す（radiusを引かないため常に非負）
  EXPECT_LT(obs.distance(p), 0.0)
    << "[バグ1] Circle distance()は内部の点に対して負値を返すべきだが、"
       "正値(0.05)を返すため失敗する";
}

TEST(ObstacleDistance, CircleDistanceOnSurface)
{
  const auto obs = Obstacle::makeCircle(Point::Zero(), test_helpers::kRobotRadius);
  // 表面上の点: (0.09, 0)
  const Point p(test_helpers::kRobotRadius, 0.0);
  // 期待値: 0
  // [バグ1] 実際には0.09を返す
  EXPECT_NEAR(obs.distance(p), 0.0, 1e-9)
    << "[バグ1] Circle distance()は表面上の点に対して0を返すべきだが、"
       "0.09を返すため失敗する";
}

TEST(ObstacleDistance, CircleGradientNumericalConsistency)
{
  // distanceGradient()は正しく計算されるが、distance()がバグ持ちのため
  // 数値微分との比較で不整合が現れる
  const auto obs = Obstacle::makeCircle(Point::Zero(), test_helpers::kRobotRadius);
  constexpr double eps = 1e-5;

  // 中心から(1, 0)方向の外側の点
  const Point p(0.5, 0.0);
  const Vector2 grad = obs.distanceGradient(p);
  const double fd_x =
    (obs.distance(p + Point(eps, 0.0)) - obs.distance(p - Point(eps, 0.0))) / (2.0 * eps);

  // distanceGradient()は正しく(1, 0)を返す
  EXPECT_NEAR(grad.x(), 1.0, 1e-6);
  EXPECT_NEAR(fd_x, 1.0, 1e-4);
  // この2つは一致する（distance()も表面外では単調増加だが、値がずれている）
}

// ===== Capsule距離テスト（バグ2検出）=====
//
// 問題: bg::distance(p, capsule) はCapsuleをsegment_tagとして扱うため、
// セグメントからの距離のみ返し、radiusを引かない。

TEST(ObstacleDistance, CapsuleDistanceOutside)
{
  // カプセル: セグメント((-1,0)-(1,0))、半径0.3m
  const Capsule cap{crane::Segment(Point(-1.0, 0.0), Point(1.0, 0.0)), 0.3};
  const auto obs = Obstacle::makeCapsule(cap);
  // 上側0.8mの点: セグメントから0.8m → 表面から0.5m
  const Point p(0.0, 0.8);
  // 期待値: 0.5m（表面からの距離 = セグメント距離 - 半径）
  // [バグ2] 実際には0.8m（セグメントからの距離）を返す
  EXPECT_NEAR(obs.distance(p), 0.5, 1e-9) << "[バグ2] Capsule distance()がradiusを引いていない。"
                                             "セグメント距離0.8を返すため失敗する";
}

TEST(ObstacleDistance, CapsuleDistanceInside)
{
  const Capsule cap{crane::Segment(Point(-1.0, 0.0), Point(1.0, 0.0)), 0.3};
  const auto obs = Obstacle::makeCapsule(cap);
  // カプセル内部の点: (0.0, 0.1) → セグメントから0.1m < 半径0.3m
  const Point p(0.0, 0.1);
  // 期待値: 負（内部）
  // [バグ2] 実際には正(0.1)を返す
  EXPECT_LT(obs.distance(p), 0.0)
    << "[バグ2] Capsule distance()は内部の点に対して負値を返すべきだが、"
       "正値(0.1)を返すため失敗する";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
