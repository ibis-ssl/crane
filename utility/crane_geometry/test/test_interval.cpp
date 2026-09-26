// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_geometry/interval.hpp>
#include <utility>

namespace crane
{
TEST(IntervalTest, AppendIntervals)
{
  Interval interval;

  // 単一の区間を追加
  interval.append(1.0, 3.0);
  EXPECT_EQ(interval.getLargestInterval(), std::make_pair(1.0, 3.0));

  // 重複しない区間を追加（結合されず、同じ幅なら先の区間が最大として残る）
  interval.append(5.0, 7.0);
  EXPECT_EQ(interval.getLargestInterval(), std::make_pair(1.0, 3.0));

  // 重複する区間を追加すると、両側の区間とまとめて 1 つになる
  interval.append(2.0, 6.0);
  EXPECT_EQ(interval.getLargestInterval(), std::make_pair(1.0, 7.0));
}

TEST(IntervalTest, EraseIntervals)
{
  Interval interval;

  // 初期区間を設定
  interval.append(0.0, 10.0);
  EXPECT_EQ(interval.getLargestInterval(), std::make_pair(0.0, 10.0));

  // 内部区間を削除すると [0, 3] と [5, 10] に分かれる
  interval.erase(3.0, 5.0);
  EXPECT_EQ(interval.getLargestInterval(), std::make_pair(5.0, 10.0));

  // 端の区間を削除すると [0, 3] が [2, 3] に縮む
  interval.erase(-1.0, 2.0);
  EXPECT_EQ(interval.getLargestInterval(), std::make_pair(5.0, 10.0));

  // [5, 10] を消すと、縮んだ [2, 3] だけが残る
  interval.erase(5.0, 10.0);
  EXPECT_EQ(interval.getLargestInterval(), std::make_pair(2.0, 3.0));
}

// 境界がちょうど接する区間を追加した場合、1つの連続区間としてマージされることを確認する
TEST(IntervalTest, AppendTouchingBoundaryMerges)
{
  Interval interval;

  interval.append(1.0, 8.0);
  interval.append(8.0, 19.0);

  auto largest = interval.getLargestInterval();
  EXPECT_DOUBLE_EQ(largest.first, 1.0);
  EXPECT_DOUBLE_EQ(largest.second, 19.0);
}

// erase範囲の境界が既存区間自身の境界と厳密に一致する場合でも、
// 区間が正しく縮まることを確認する（単一区間・単一eraseで再現する最小ケース）
TEST(IntervalTest, EraseExactBoundaryMatchShrinksInterval)
{
  Interval interval;

  interval.append(1.0, 19.0);
  interval.erase(1.0, 7.0);

  auto largest = interval.getLargestInterval();
  EXPECT_DOUBLE_EQ(largest.first, 7.0);
  EXPECT_DOUBLE_EQ(largest.second, 19.0);
}
}  // namespace crane
