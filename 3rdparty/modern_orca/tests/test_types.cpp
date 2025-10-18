// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <modern_orca/types.hpp>

TEST(HalfPlaneTest, ConstructorAndDistance)
{
  crane::Vector2 normal(1.0, 0.0);
  crane::Vector2 point(2.0, 0.0);
  crane::modern_orca::HalfPlane plane(normal, point);

  EXPECT_DOUBLE_EQ(plane.normal.x(), 1.0);
  EXPECT_DOUBLE_EQ(plane.normal.y(), 0.0);
  EXPECT_DOUBLE_EQ(plane.point.x(), 2.0);
  EXPECT_DOUBLE_EQ(plane.point.y(), 0.0);

  crane::Vector2 test_point(3.0, 0.0);
  EXPECT_NEAR(plane.signedDistance(test_point), 1.0, 1e-10);

  crane::Vector2 test_point2(1.0, 0.0);
  EXPECT_NEAR(plane.signedDistance(test_point2), -1.0, 1e-10);
}

TEST(HalfPlaneTest, ContainmentTest)
{
  crane::modern_orca::HalfPlane plane(crane::Vector2(0.0, 1.0), crane::Vector2(0.0, 1.0));

  EXPECT_TRUE(plane.contains(crane::Vector2(0.0, 2.0)));
  EXPECT_TRUE(plane.contains(crane::Vector2(0.0, 1.0)));
  EXPECT_FALSE(plane.contains(crane::Vector2(0.0, 0.5)));
}

TEST(HalfPlaneTest, Projection)
{
  crane::modern_orca::HalfPlane plane(crane::Vector2(1.0, 0.0), crane::Vector2(1.0, 0.0));
  crane::Vector2 test_point(0.5, 2.0);

  auto projected = plane.project(test_point);
  EXPECT_NEAR(projected.x(), 1.0, 1e-10);
  EXPECT_NEAR(projected.y(), 2.0, 1e-10);
}

TEST(LineTest, ConstructorAndPointCalculation)
{
  crane::Vector2 point(0.0, 0.0);
  crane::Vector2 direction(1.0, 1.0);
  crane::modern_orca::Line line(point, direction);

  auto point_on_line = line.at(1.0);
  EXPECT_NEAR(point_on_line.x(), 1.0 / std::sqrt(2.0), 1e-10);
  EXPECT_NEAR(point_on_line.y(), 1.0 / std::sqrt(2.0), 1e-10);
}

TEST(LineTest, ClosestPointAndDistance)
{
  crane::modern_orca::Line line =
    crane::modern_orca::Line::fromTwoPoints(crane::Vector2(0.0, 0.0), crane::Vector2(2.0, 0.0));
  crane::Vector2 test_point(1.0, 1.0);

  auto closest = line.closestPoint(test_point);
  EXPECT_NEAR(closest.x(), 1.0, 1e-10);
  EXPECT_NEAR(closest.y(), 0.0, 1e-10);

  EXPECT_NEAR(line.distanceTo(test_point), 1.0, 1e-10);
}

TEST(MathUtilitiesTest, DistanceFunctions)
{
  crane::Vector2 a(0.0, 0.0);
  crane::Vector2 b(3.0, 4.0);

  EXPECT_NEAR(crane::modern_orca::distance(a, b), 5.0, 1e-10);
  EXPECT_NEAR(crane::modern_orca::distanceSquared(a, b), 25.0, 1e-10);
}

TEST(MathUtilitiesTest, VectorOperations)
{
  crane::Vector2 a(1.0, 2.0);
  crane::Vector2 b(3.0, 4.0);

  EXPECT_DOUBLE_EQ(crane::modern_orca::dot(a, b), 11.0);
  EXPECT_DOUBLE_EQ(crane::modern_orca::cross(a, b), -2.0);
}

TEST(MathUtilitiesTest, LinearInterpolation)
{
  crane::Vector2 a(0.0, 0.0);
  crane::Vector2 b(2.0, 4.0);

  auto mid = crane::modern_orca::lerp(a, b, 0.5);
  EXPECT_DOUBLE_EQ(mid.x(), 1.0);
  EXPECT_DOUBLE_EQ(mid.y(), 2.0);

  auto quarter = crane::modern_orca::lerp(a, b, 0.25);
  EXPECT_DOUBLE_EQ(quarter.x(), 0.5);
  EXPECT_DOUBLE_EQ(quarter.y(), 1.0);
}
