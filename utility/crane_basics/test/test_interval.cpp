// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_geometry/interval.hpp>

namespace crane
{
// Intervalクラスのテスト
TEST(IntervalTest, AppendIntervals)
{
  Interval interval;

  // 単一の区間を追加
  interval.append(1.0, 3.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 2.0);

  // 重複しない区間を追加
  interval.append(5.0, 7.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 4.0);

  // 重複する区間を追加
  interval.append(2.0, 6.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 6.0);

  // 最大区間の確認
  auto largest = interval.getLargestInterval();
  EXPECT_DOUBLE_EQ(largest.first, 1.0);
  EXPECT_DOUBLE_EQ(largest.second, 7.0);
}

TEST(IntervalTest, EraseIntervals)
{
  Interval interval;

  // 初期区間を設定
  interval.append(0.0, 10.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 10.0);

  // 内部区間を削除
  interval.erase(3.0, 5.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 8.0);

  // 端の区間を削除
  interval.erase(-1.0, 2.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 6.0);

  // 最大区間の確認
  auto largest = interval.getLargestInterval();
  EXPECT_DOUBLE_EQ(largest.first, 5.0);
  EXPECT_DOUBLE_EQ(largest.second, 10.0);
}
}  // namespace crane
