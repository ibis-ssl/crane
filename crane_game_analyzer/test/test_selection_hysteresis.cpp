// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_game_analyzer/selection_hysteresis.hpp>
#include <memory>

namespace crane
{
namespace
{
/// AttackerMetric と同じ設定。即切替の2条件だけ差し替えられるようにする。
auto makeHysteresis(double emergency_switch_ratio, double absolute_threshold)
  -> SelectionHysteresis<int>
{
  // 経過時間に依存しない条件だけを検証したいので、テスト内では時間を進めない。
  // 判定直後は force_switch_timeout(3.0s) も min_hold_duration_sec(0.5s) も
  // 満たされないため、即切替の分岐だけが結果を決める。
  return SelectionHysteresis<int>(
    SelectionHysteresis<int>::Config{
      .min_hold_duration_sec = 0.5,
      .min_improvement_ratio = 0.15,
      .emergency_switch_ratio = emergency_switch_ratio,
      .force_switch_timeout = 3.0,
      .absolute_threshold = absolute_threshold},
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
}

constexpr double kAttackerRatio = 1.5;
constexpr double kAttackerAbsolute = 2.0;
}  // namespace

// 実測された振動その1（小スコア領域）: 1.50 → 2.28。
// 倍率は 1.52 で条件を満たすが、差は 0.78 しかない。
// 比率だけで即切替を許すとここが通り、切替間隔が 0.14〜0.32 秒まで縮む。
TEST(SelectionHysteresisTest, SmallAbsoluteGainDoesNotBypassHoldDuration)
{
  auto hysteresis = makeHysteresis(kAttackerRatio, kAttackerAbsolute);
  ASSERT_TRUE(hysteresis.shouldSwitch(1, 1.50, 0.0));  // 初回選択
  EXPECT_FALSE(hysteresis.shouldSwitch(2, 2.28, 1.50));
  EXPECT_EQ(hysteresis.currentId(), 1);
}

// 実測された振動その2（大スコア領域）: 21.40 → 23.94。
// 差は 2.54 で絶対条件を満たすが、改善はわずか 11.9%。
// 絶対差だけで即切替を許すとここが通り、切替間隔が 0.16 秒になる。
TEST(SelectionHysteresisTest, SmallRelativeGainDoesNotBypassHoldDuration)
{
  auto hysteresis = makeHysteresis(kAttackerRatio, kAttackerAbsolute);
  ASSERT_TRUE(hysteresis.shouldSwitch(1, 21.40, 0.0));
  EXPECT_FALSE(hysteresis.shouldSwitch(2, 23.94, 21.40));
  EXPECT_EQ(hysteresis.currentId(), 1);
}

// 倍率と絶対差の両方を満たすなら即切替する（実測 3.94 → 7.20）。
TEST(SelectionHysteresisTest, ClearlySuperiorCandidateSwitchesImmediately)
{
  auto hysteresis = makeHysteresis(kAttackerRatio, kAttackerAbsolute);
  ASSERT_TRUE(hysteresis.shouldSwitch(1, 3.94, 0.0));
  EXPECT_TRUE(hysteresis.shouldSwitch(2, 7.20, 3.94));
  EXPECT_EQ(hysteresis.currentId(), 2);
}

// 片方しか設定していない利用者の挙動は変えない（未設定の条件は課さない）。
TEST(SelectionHysteresisTest, RatioOnlyConfigKeepsLegacyBehavior)
{
  auto hysteresis = makeHysteresis(kAttackerRatio, 0.0);
  ASSERT_TRUE(hysteresis.shouldSwitch(1, 1.50, 0.0));
  EXPECT_TRUE(hysteresis.shouldSwitch(2, 2.28, 1.50));  // 差 0.78 でも倍率だけで通る
  EXPECT_EQ(hysteresis.currentId(), 2);
}

TEST(SelectionHysteresisTest, AbsoluteOnlyConfigKeepsLegacyBehavior)
{
  auto hysteresis = makeHysteresis(0.0, kAttackerAbsolute);
  ASSERT_TRUE(hysteresis.shouldSwitch(1, 21.40, 0.0));
  EXPECT_TRUE(hysteresis.shouldSwitch(2, 23.94, 21.40));  // 11.9% でも絶対差だけで通る
  EXPECT_EQ(hysteresis.currentId(), 2);
}

// 即切替を一切設定しなければ、保持時間を待つ。
TEST(SelectionHysteresisTest, NoImmediateSwitchConfiguredHoldsUntilDuration)
{
  auto hysteresis = makeHysteresis(0.0, 0.0);
  ASSERT_TRUE(hysteresis.shouldSwitch(1, 1.00, 0.0));
  EXPECT_FALSE(hysteresis.shouldSwitch(2, 100.0, 1.00));
  EXPECT_EQ(hysteresis.currentId(), 1);
}

// 同一IDなら常に切替なし（内部状態を汚さない）。
TEST(SelectionHysteresisTest, SameIdNeverSwitches)
{
  auto hysteresis = makeHysteresis(kAttackerRatio, kAttackerAbsolute);
  ASSERT_TRUE(hysteresis.shouldSwitch(1, 1.50, 0.0));
  EXPECT_FALSE(hysteresis.shouldSwitch(1, 99.0, 1.50));
  EXPECT_EQ(hysteresis.currentId(), 1);
}
}  // namespace crane
