// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>
#include <crane_geometry/geometry_operations.hpp>

// Circleのテスト
TEST(CircleTest, CreateAndMeasure) {
  Circle circle;
  circle.center << 0, 0;
  circle.radius = 5.0;

  Point point(10, 0);
  double distance = bg::distance(circle, point);

  EXPECT_DOUBLE_EQ(distance, 5.0);
}

// Capsuleのテスト
TEST(CapsuleTest, CreateAndMeasure) {
  Capsule capsule;
  capsule.segment.first << 0, 0;
  capsule.segment.second << 10, 0;
  capsule.radius = 2.0;

  Point point(5, 5);
  double distance = bg::distance(capsule, point);

  EXPECT_DOUBLE_EQ(distance, 3.0);
}

// メイン関数
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
