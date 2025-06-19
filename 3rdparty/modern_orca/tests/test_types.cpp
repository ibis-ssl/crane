// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>
#include <modern_orca/types.hpp>

TEST(Vector2DTest, ConstructorAndAccess)
{
  modern_orca::Vector2D v1;
  EXPECT_DOUBLE_EQ(v1.x(), 0.0);
  EXPECT_DOUBLE_EQ(v1.y(), 0.0);

  modern_orca::Vector2D v2(3.0, 4.0);
  EXPECT_DOUBLE_EQ(v2.x(), 3.0);
  EXPECT_DOUBLE_EQ(v2.y(), 4.0);
  EXPECT_DOUBLE_EQ(v2[0], 3.0);
  EXPECT_DOUBLE_EQ(v2[1], 4.0);
}

TEST(Vector2DTest, ArithmeticOperations)
{
  modern_orca::Vector2D v1(1.0, 2.0);
  modern_orca::Vector2D v2(3.0, 4.0);

  auto sum = v1 + v2;
  EXPECT_DOUBLE_EQ(sum.x(), 4.0);
  EXPECT_DOUBLE_EQ(sum.y(), 6.0);

  auto diff = v2 - v1;
  EXPECT_DOUBLE_EQ(diff.x(), 2.0);
  EXPECT_DOUBLE_EQ(diff.y(), 2.0);

  auto scaled = v1 * 2.0;
  EXPECT_DOUBLE_EQ(scaled.x(), 2.0);
  EXPECT_DOUBLE_EQ(scaled.y(), 4.0);

  auto divided = v2 / 2.0;
  EXPECT_DOUBLE_EQ(divided.x(), 1.5);
  EXPECT_DOUBLE_EQ(divided.y(), 2.0);
}

TEST(Vector2DTest, GeometricOperations)
{
  modern_orca::Vector2D v1(3.0, 4.0);
  EXPECT_DOUBLE_EQ(v1.squaredNorm(), 25.0);
  EXPECT_DOUBLE_EQ(v1.norm(), 5.0);

  auto normalized = v1.normalized();
  EXPECT_NEAR(normalized.x(), 0.6, 1e-10);
  EXPECT_NEAR(normalized.y(), 0.8, 1e-10);
  EXPECT_NEAR(normalized.norm(), 1.0, 1e-10);

  modern_orca::Vector2D v2(1.0, 0.0);
  EXPECT_DOUBLE_EQ(v1.dot(v2), 3.0);
  EXPECT_DOUBLE_EQ(v1.cross(v2), -4.0);

  auto perp = v2.perpendicular();
  EXPECT_DOUBLE_EQ(perp.x(), 0.0);
  EXPECT_DOUBLE_EQ(perp.y(), 1.0);
}

TEST(Vector2DTest, AngleAndRotation)
{
  modern_orca::Vector2D v1(1.0, 0.0);
  EXPECT_NEAR(v1.angle(), 0.0, 1e-10);

  modern_orca::Vector2D v2(0.0, 1.0);
  EXPECT_NEAR(v2.angle(), modern_orca::PI / 2.0, 1e-10);

  auto rotated = v1.rotate(modern_orca::PI / 2.0);
  EXPECT_NEAR(rotated.x(), 0.0, 1e-10);
  EXPECT_NEAR(rotated.y(), 1.0, 1e-10);
}

TEST(Vector2DTest, ComparisonAndUtilities)
{
  modern_orca::Vector2D v1(1.0, 2.0);
  modern_orca::Vector2D v2(1.0, 2.0);
  modern_orca::Vector2D v3(1.1, 2.0);

  EXPECT_TRUE(v1 == v2);
  EXPECT_TRUE(v1 != v3);

  modern_orca::Vector2D zero;
  EXPECT_TRUE(zero.isZero());
  EXPECT_FALSE(v1.isZero());
}

TEST(HalfPlaneTest, ConstructorAndDistance)
{
  modern_orca::Vector2D normal(1.0, 0.0);
  modern_orca::Vector2D point(2.0, 0.0);
  modern_orca::HalfPlaneD plane(normal, point);

  EXPECT_DOUBLE_EQ(plane.normal.x(), 1.0);
  EXPECT_DOUBLE_EQ(plane.normal.y(), 0.0);
  EXPECT_DOUBLE_EQ(plane.point.x(), 2.0);
  EXPECT_DOUBLE_EQ(plane.point.y(), 0.0);

  modern_orca::Vector2D test_point(3.0, 0.0);
  EXPECT_NEAR(plane.signedDistance(test_point), 1.0, 1e-10);

  modern_orca::Vector2D test_point2(1.0, 0.0);
  EXPECT_NEAR(plane.signedDistance(test_point2), -1.0, 1e-10);
}

TEST(HalfPlaneTest, ContainmentTest)
{
  modern_orca::HalfPlaneD plane(modern_orca::Vector2D(0.0, 1.0), modern_orca::Vector2D(0.0, 1.0));

  EXPECT_TRUE(plane.contains(modern_orca::Vector2D(0.0, 2.0)));
  EXPECT_TRUE(plane.contains(modern_orca::Vector2D(0.0, 1.0)));
  EXPECT_FALSE(plane.contains(modern_orca::Vector2D(0.0, 0.5)));
}

TEST(HalfPlaneTest, Projection)
{
  modern_orca::HalfPlaneD plane(modern_orca::Vector2D(1.0, 0.0), modern_orca::Vector2D(1.0, 0.0));
  modern_orca::Vector2D test_point(0.5, 2.0);

  auto projected = plane.project(test_point);
  EXPECT_NEAR(projected.x(), 1.0, 1e-10);
  EXPECT_NEAR(projected.y(), 2.0, 1e-10);
}

TEST(LineTest, ConstructorAndPointCalculation)
{
  modern_orca::Vector2D point(0.0, 0.0);
  modern_orca::Vector2D direction(1.0, 1.0);
  modern_orca::LineD line(point, direction);

  auto point_on_line = line.at(1.0);
  EXPECT_NEAR(point_on_line.x(), 1.0 / std::sqrt(2.0), 1e-10);
  EXPECT_NEAR(point_on_line.y(), 1.0 / std::sqrt(2.0), 1e-10);
}

TEST(LineTest, ClosestPointAndDistance)
{
  modern_orca::LineD line = modern_orca::LineD::fromTwoPoints(
    modern_orca::Vector2D(0.0, 0.0), modern_orca::Vector2D(2.0, 0.0));
  modern_orca::Vector2D test_point(1.0, 1.0);

  auto closest = line.closestPoint(test_point);
  EXPECT_NEAR(closest.x(), 1.0, 1e-10);
  EXPECT_NEAR(closest.y(), 0.0, 1e-10);

  EXPECT_NEAR(line.distanceTo(test_point), 1.0, 1e-10);
}

TEST(MathUtilitiesTest, DistanceFunctions)
{
  modern_orca::Vector2D a(0.0, 0.0);
  modern_orca::Vector2D b(3.0, 4.0);

  EXPECT_NEAR(distance(a, b), 5.0, 1e-10);
  EXPECT_NEAR(distanceSquared(a, b), 25.0, 1e-10);
}

TEST(MathUtilitiesTest, VectorOperations)
{
  modern_orca::Vector2D a(1.0, 2.0);
  modern_orca::Vector2D b(3.0, 4.0);

  EXPECT_DOUBLE_EQ(dot(a, b), 11.0);
  EXPECT_DOUBLE_EQ(cross(a, b), -2.0);
}

TEST(MathUtilitiesTest, LinearInterpolation)
{
  modern_orca::Vector2D a(0.0, 0.0);
  modern_orca::Vector2D b(2.0, 4.0);

  auto mid = lerp(a, b, 0.5);
  EXPECT_DOUBLE_EQ(mid.x(), 1.0);
  EXPECT_DOUBLE_EQ(mid.y(), 2.0);

  auto quarter = lerp(a, b, 0.25);
  EXPECT_DOUBLE_EQ(quarter.x(), 0.5);
  EXPECT_DOUBLE_EQ(quarter.y(), 1.0);
}