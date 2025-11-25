// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>

namespace crane
{
// Circleのテスト
TEST(CircleTest, CreateAndMeasure)
{
  crane::Circle circle{.center = Point(0.0, 0.0), .radius = 5.0};

  Point point(10.0, 0.0);
  double distance = bg::distance(circle, point);

  EXPECT_DOUBLE_EQ(distance, 5.0);
}

// Capsuleのテスト
TEST(CapsuleTest, CreateAndMeasure)
{
  Capsule capsule{
    .segment = Segment(Point(0.0, 0.0), Point(10.0, 0.0)),
    .radius = 2.0
  };

  Point point(5.0, 5.0);
  double distance = bg::distance(capsule, point);

  EXPECT_DOUBLE_EQ(distance, 3.0);
}

// geometry_operationsのテスト追加
TEST(GeometryOperationsTest, NormalizeAngle)
{
  // 正の角度の正規化
  EXPECT_NEAR(normalizeAngle(3.5 * M_PI), -0.5 * M_PI, 1e-10);

  // 負の角度の正規化
  EXPECT_NEAR(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI, 1e-10);

  // -π〜πの範囲内の角度は変わらない
  EXPECT_DOUBLE_EQ(normalizeAngle(0.5), 0.5);
  EXPECT_DOUBLE_EQ(normalizeAngle(-0.5), -0.5);
}

TEST(GeometryOperationsTest, GetAngleDiff)
{
  // 単純な差
  EXPECT_DOUBLE_EQ(getAngleDiff(0.5, 0.3), 0.2);

  // -πとπの間の差（境界を超える）
  EXPECT_NEAR(getAngleDiff(M_PI - 0.1, -M_PI + 0.1), -0.2, 1e-10);

  // Pose2D間の角度差
  Pose2D pose1{.pos = Point(0.0, 0.0), .theta = 0.5};
  Pose2D pose2{.pos = Point(0.0, 0.0), .theta = -0.5};
  EXPECT_DOUBLE_EQ(getAngleDiff(pose1, pose2), 1.0);

  // Pose2DとDouble間の角度差
  EXPECT_DOUBLE_EQ(getAngleDiff(pose1, 0.0), 0.5);
  EXPECT_DOUBLE_EQ(getAngleDiff(0.0, pose1), -0.5);
}

TEST(GeometryOperationsTest, GetIntermediateAngle)
{
  // 単純な中間角度
  EXPECT_DOUBLE_EQ(getIntermediateAngle(0.0, 1.0), 0.5);

  // -πとπの間の中間角度（境界を超える）
  EXPECT_NEAR(getIntermediateAngle(M_PI - 0.1, -M_PI + 0.1), M_PI, 1e-10);
}

TEST(GeometryOperationsTest, GetCircle)
{
  // 3点から円を作成
  // (1,0)を中心とする半径1の円
  Point p1(0.0, 0.0);
  Point p2(2.0, 0.0);
  Point p3(1.0, 1.0);
  auto circle = getCircle(p1, p2, p3);

  ASSERT_TRUE(circle.has_value());
  EXPECT_NEAR(circle->center.x(), 1.0, 1e-10);
  EXPECT_NEAR(circle->center.y(), 0.0, 1e-10);
  EXPECT_NEAR(circle->radius, 1.0, 1e-10);

  // 一直線上の3点からは円を作成できない
  Point p4(3.0, 0.0);
  auto invalid_circle = getCircle(p1, p2, p4);
  EXPECT_FALSE(invalid_circle.has_value());
}
}  // namespace crane
