// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_comm/time.hpp>

namespace crane
{
// time.hppの関数テスト
TEST(TimeTest, GetDiffSec)
{
  auto now = std::chrono::high_resolution_clock::now();
  auto later = now + std::chrono::milliseconds(100);

  // 差分は約0.1秒
  double diff = getDiffSec(now, later);
  EXPECT_NEAR(diff, 0.1, 0.01);

  // 順序が逆でも絶対値なので同じ
  double diff2 = getDiffSec(later, now);
  EXPECT_NEAR(diff2, 0.1, 0.01);
}

// getElapsedSecのテストはtickに依存するため省略
// ScopedTimerのテストも外部依存が多いため省略

// テンプレート関数のインスタンス化テスト
TEST(TimeTest, TemplateInstantiation)
{
  using TestClock = std::chrono::steady_clock;

  auto start = TestClock::now();
  auto end = start + std::chrono::seconds(1);

  double diff = getDiffSec<TestClock>(start, end);
  EXPECT_DOUBLE_EQ(diff, 1.0);
}
}  // namespace crane
