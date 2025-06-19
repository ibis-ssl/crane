// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <modern_orca/types.hpp>

using Catch::Approx;

TEST_CASE("modern_orca::Vector2D basic operations", "[modern_orca::Vector2D]")
{
  SECTION("Construction and access")
  {
    modern_orca::modern_orca::Vector2D v1;
    REQUIRE(v1.x() == 0.0);
    REQUIRE(v1.y() == 0.0);

    modern_orca::modern_orca::Vector2D v2(3.0, 4.0);
    REQUIRE(v2.x() == 3.0);
    REQUIRE(v2.y() == 4.0);
    REQUIRE(v2[0] == 3.0);
    REQUIRE(v2[1] == 4.0);
  }

  SECTION("Arithmetic operations")
  {
    modern_orca::Vector2D v1(1.0, 2.0);
    modern_orca::modern_orca::Vector2D v2(3.0, 4.0);

    auto sum = v1 + v2;
    REQUIRE(sum.x() == 4.0);
    REQUIRE(sum.y() == 6.0);

    auto diff = v2 - v1;
    REQUIRE(diff.x() == 2.0);
    REQUIRE(diff.y() == 2.0);

    auto scaled = v1 * 2.0;
    REQUIRE(scaled.x() == 2.0);
    REQUIRE(scaled.y() == 4.0);

    auto divided = v2 / 2.0;
    REQUIRE(divided.x() == 1.5);
    REQUIRE(divided.y() == 2.0);
  }

  SECTION("Geometric operations")
  {
    modern_orca::Vector2D v1(3.0, 4.0);
    REQUIRE(v1.squaredNorm() == 25.0);
    REQUIRE(v1.norm() == Approx(5.0));

    auto normalized = v1.normalized();
    REQUIRE(normalized.x() == Approx(0.6));
    REQUIRE(normalized.y() == Approx(0.8));
    REQUIRE(normalized.norm() == Approx(1.0));

    modern_orca::Vector2D v2(1.0, 0.0);
    REQUIRE(v1.dot(v2) == 3.0);
    REQUIRE(v1.cross(v2) == 4.0);

    auto perp = v2.perpendicular();
    REQUIRE(perp.x() == 0.0);
    REQUIRE(perp.y() == 1.0);
  }

  SECTION("Angle and rotation")
  {
    modern_orca::Vector2D v1(1.0, 0.0);
    REQUIRE(v1.angle() == Approx(0.0));

    modern_orca::Vector2D v2(0.0, 1.0);
    REQUIRE(v2.angle() == Approx(modern_orca::PI / 2.0));

    auto rotated = v1.rotate(modern_orca::PI / 2.0);
    REQUIRE(rotated.x() == Approx(0.0).margin(1e-10));
    REQUIRE(rotated.y() == Approx(1.0));
  }

  SECTION("Comparison and utility")
  {
    modern_orca::Vector2D v1(1.0, 2.0);
    modern_orca::Vector2D v2(1.0, 2.0);
    modern_orca::Vector2D v3(1.1, 2.0);

    REQUIRE(v1 == v2);
    REQUIRE(v1 != v3);

    modern_orca::Vector2D zero;
    REQUIRE(zero.isZero());
    REQUIRE_FALSE(v1.isZero());
  }
}

TEST_CASE("HalfPlane operations", "[HalfPlane]")
{
  SECTION("Construction and distance")
  {
    modern_orca::Vector2D normal(1.0, 0.0);
    modern_orca::Vector2D point(2.0, 0.0);
    modern_orca::HalfPlaneD plane(normal, point);

    REQUIRE(plane.normal.x() == 1.0);
    REQUIRE(plane.normal.y() == 0.0);
    REQUIRE(plane.point.x() == 2.0);
    REQUIRE(plane.point.y() == 0.0);

    modern_orca::Vector2D test_point(3.0, 0.0);
    REQUIRE(plane.signedDistance(test_point) == Approx(1.0));

    modern_orca::Vector2D test_point2(1.0, 0.0);
    REQUIRE(plane.signedDistance(test_point2) == Approx(-1.0));
  }

  SECTION("Containment testing")
  {
    modern_orca::HalfPlaneD plane(modern_orca::Vector2D(0.0, 1.0), modern_orca::Vector2D(0.0, 1.0));

    REQUIRE(plane.contains(modern_orca::Vector2D(0.0, 2.0)));
    REQUIRE(plane.contains(modern_orca::Vector2D(0.0, 1.0)));
    REQUIRE_FALSE(plane.contains(modern_orca::Vector2D(0.0, 0.5)));
  }

  SECTION("Projection")
  {
    modern_orca::HalfPlaneD plane(modern_orca::Vector2D(1.0, 0.0), modern_orca::Vector2D(1.0, 0.0));
    modern_orca::Vector2D test_point(0.5, 2.0);

    auto projected = plane.project(test_point);
    REQUIRE(projected.x() == Approx(1.0));
    REQUIRE(projected.y() == Approx(2.0));
  }
}

TEST_CASE("Line operations", "[Line]")
{
  SECTION("Construction and point calculation")
  {
    modern_orca::Vector2D point(0.0, 0.0);
    modern_orca::Vector2D direction(1.0, 1.0);
    modern_orca::LineD line(point, direction);

    auto point_on_line = line.at(1.0);
    REQUIRE(point_on_line.x() == Approx(1.0 / std::sqrt(2.0)));
    REQUIRE(point_on_line.y() == Approx(1.0 / std::sqrt(2.0)));
  }

  SECTION("Closest point and distance")
  {
    modern_orca::LineD line = modern_orca::LineD::fromTwoPoints(
      modern_orca::Vector2D(0.0, 0.0), modern_orca::Vector2D(2.0, 0.0));
    modern_orca::Vector2D test_point(1.0, 1.0);

    auto closest = line.closestPoint(test_point);
    REQUIRE(closest.x() == Approx(1.0));
    REQUIRE(closest.y() == Approx(0.0));

    REQUIRE(line.distanceTo(test_point) == Approx(1.0));
  }
}

TEST_CASE("Mathematical utility functions", "[utilities]")
{
  SECTION("Distance functions")
  {
    modern_orca::Vector2D a(0.0, 0.0);
    modern_orca::Vector2D b(3.0, 4.0);

    REQUIRE(distance(a, b) == Approx(5.0));
    REQUIRE(distanceSquared(a, b) == Approx(25.0));
  }

  SECTION("Vector operations")
  {
    modern_orca::Vector2D a(1.0, 2.0);
    modern_orca::Vector2D b(3.0, 4.0);

    REQUIRE(dot(a, b) == 11.0);
    REQUIRE(cross(a, b) == -2.0);
  }

  SECTION("Linear interpolation")
  {
    modern_orca::Vector2D a(0.0, 0.0);
    modern_orca::Vector2D b(2.0, 4.0);

    auto mid = lerp(a, b, 0.5);
    REQUIRE(mid.x() == 1.0);
    REQUIRE(mid.y() == 2.0);

    auto quarter = lerp(a, b, 0.25);
    REQUIRE(quarter.x() == 0.5);
    REQUIRE(quarter.y() == 1.0);
  }
}
