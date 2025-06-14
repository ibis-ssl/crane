// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_basics/geometry_operations.hpp>

namespace crane
{
TEST(Vector2dOperationsTest, SquaredNormAndScalarMultiply)
{
  crane::Vector2d vec;
  vec << 2.0, 3.0;

  // Test squaredNorm
  EXPECT_DOUBLE_EQ(vec.squaredNorm(), 2.0 * 2.0 + 3.0 * 3.0);  // 4 + 9 = 13

  // Test scalar * Vector2d
  crane::Vector2d res_sv = 2.5 * vec;
  EXPECT_DOUBLE_EQ(res_sv.x(), 2.5 * 2.0);
  EXPECT_DOUBLE_EQ(res_sv.y(), 2.5 * 3.0);

  // Test Vector2d * scalar (should already exist and work)
  crane::Vector2d res_vs = vec * 2.5;
  EXPECT_DOUBLE_EQ(res_vs.x(), 2.0 * 2.5);
  EXPECT_DOUBLE_EQ(res_vs.y(), 3.0 * 2.5);
}
}  // namespace crane
