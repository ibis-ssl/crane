// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_utils/time.hpp>

namespace crane
{
TEST(TimeTest, GetDiffSec)
{
  auto now = std::chrono::high_resolution_clock::now();
  auto later = now + std::chrono::milliseconds(100);

  double diff = getDiffSec(now, later);
  EXPECT_NEAR(diff, 0.1, 0.01);

  // 順序が逆でも絶対値なので同じ
  double diff2 = getDiffSec(later, now);
  EXPECT_NEAR(diff2, 0.1, 0.01);
}

// getElapsedSecのテストはtickに依存するため省略
// ScopedTimerのテストも外部依存が多いため省略

TEST(TimeTest, TemplateInstantiation)
{
  using TestClock = std::chrono::steady_clock;

  auto start = TestClock::now();
  auto end = start + std::chrono::seconds(1);

  double diff = getDiffSec<TestClock>(start, end);
  EXPECT_DOUBLE_EQ(diff, 1.0);
}

TEST(TimeTest, RclcppTimeDiffAndElapsed)
{
  rclcpp::Time t1(10, 0, RCL_ROS_TIME);
  rclcpp::Time t2(12, 500000000, RCL_ROS_TIME);  // 12.5s

  EXPECT_NEAR(getDiffSec(t1, t2), 2.5, 1e-6);
  EXPECT_NEAR(getDiffSec(t2, t1), 2.5, 1e-6);

  EXPECT_NEAR(getElapsedSec(t1, t2), 2.5, 1e-6);
  EXPECT_NEAR(getElapsedSec(t2, t1), -2.5, 1e-6);

  EXPECT_TRUE(isTimeout(t1, 2.0, t2));
  EXPECT_FALSE(isTimeout(t1, 3.0, t2));

  EXPECT_TRUE(isValidTime(t1));
  EXPECT_FALSE(isValidTime(rclcpp::Time(0, 0, RCL_ROS_TIME)));
}

TEST(TimeTest, BuiltinInterfacesTime)
{
  builtin_interfaces::msg::Time msg_t1;
  msg_t1.sec = 5;
  msg_t1.nanosec = 0;

  builtin_interfaces::msg::Time msg_t2;
  msg_t2.sec = 8;
  msg_t2.nanosec = 0;

  rclcpp::Time rcl_now(10, 0, RCL_ROS_TIME);

  EXPECT_NEAR(getDiffSec(msg_t1, msg_t2), 3.0, 1e-6);
  EXPECT_NEAR(getDiffSec(msg_t1, rcl_now), 5.0, 1e-6);
  EXPECT_NEAR(getDiffSec(rcl_now, msg_t1), 5.0, 1e-6);

  EXPECT_NEAR(getElapsedSec(msg_t1, rcl_now), 5.0, 1e-6);
  EXPECT_TRUE(isTimeout(msg_t1, 4.0, rcl_now));
  EXPECT_FALSE(isTimeout(msg_t1, 6.0, rcl_now));

  EXPECT_TRUE(isValidTime(msg_t1));

  builtin_interfaces::msg::Time zero_time;
  zero_time.sec = 0;
  zero_time.nanosec = 0;
  EXPECT_FALSE(isValidTime(zero_time));
}
}  // namespace crane
