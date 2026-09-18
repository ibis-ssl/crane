// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_physics/pass_feasibility.hpp>

namespace crane
{
namespace
{
constexpr double kMargin = 0.3;
}  // namespace

// ── isReceiveFeasible: マージンと比較方向を固定 ───────────────────

TEST(PassFeasibilityTest, IsReceiveFeasible_ReceiverArrivesFirst)
{
  // 受け手 1.0s + margin 0.3s = 1.3s <= ボール 2.0s → 先着可
  EXPECT_TRUE(isReceiveFeasible(1.0, 2.0, kMargin));
}

TEST(PassFeasibilityTest, IsReceiveFeasible_MarginBlocks)
{
  // 受け手 1.8s + margin 0.3s = 2.1s > ボール 2.0s → 先着不可
  EXPECT_FALSE(isReceiveFeasible(1.8, 2.0, kMargin));
}

TEST(PassFeasibilityTest, IsReceiveFeasible_BoundaryIsInclusive)
{
  // 等号は先着可（<=）。1.7 + 0.3 = 2.0 == 2.0
  EXPECT_TRUE(isReceiveFeasible(1.7, 2.0, kMargin));
}

TEST(PassFeasibilityTest, IsReceiveFeasible_BoundaryJustOver)
{
  // 境界を僅かに超えると不可（比較方向の確認）
  EXPECT_FALSE(isReceiveFeasible(1.7 + 1e-9, 2.0, kMargin));
}

TEST(PassFeasibilityTest, IsReceiveFeasible_ZeroMargin)
{
  EXPECT_TRUE(isReceiveFeasible(2.0, 2.0, 0.0));
  EXPECT_FALSE(isReceiveFeasible(2.0 + 1e-9, 2.0, 0.0));
}

// ── feasibleReceivePoint: 到達可否 + 先着の合成 ───────────────────

TEST(PassFeasibilityTest, FeasibleReceivePoint_ReceiverAtPointIsFeasible)
{
  ReceiveFeasibilityParams params;  // 既定
  // 受け手が受領点に静止 → receiver_travel_time = 0 → ボール到達時間 > margin なら可
  const Point origin(0.0, 0.0);
  const Point point(2.0, 0.0);
  const auto r = feasibleReceivePoint(origin, point, point, Vector2(0.0, 0.0), params);
  EXPECT_TRUE(r.ball_reachable);
  EXPECT_NEAR(r.receiver_travel_time, 0.0, 1e-9);
  EXPECT_TRUE(r.feasible);
}

TEST(PassFeasibilityTest, FeasibleReceivePoint_FarReceiverInfeasible)
{
  ReceiveFeasibilityParams params;
  const Point origin(0.0, 0.0);
  const Point point(2.0, 0.0);
  // 受け手は受領点から 12m 離れて静止 → 到達が遅くボールに間に合わない
  const auto r = feasibleReceivePoint(origin, point, Point(-10.0, 0.0), Vector2(0.0, 0.0), params);
  EXPECT_GT(r.receiver_travel_time, r.ball_travel_time);
  EXPECT_FALSE(r.feasible);
}

TEST(PassFeasibilityTest, FeasibleReceivePoint_BallUnreachableIsInfeasible)
{
  ReceiveFeasibilityParams params;
  // 100m 先はクランプ後の最大初速でも届かない（reachable=false）→ feasible=false
  const Point origin(0.0, 0.0);
  const Point point(100.0, 0.0);
  const auto r = feasibleReceivePoint(origin, point, point, Vector2(0.0, 0.0), params);
  EXPECT_FALSE(r.ball_reachable);
  EXPECT_FALSE(r.feasible);
}
// 受け手モデルが敵の迎撃モデルより低い能力だと、実力では届く受領点まで
// 「間に合わない」として捨ててしまう。実際にそうなっており（受け手 3.0/4.0 に対し
// 敵 3.0/5.5）、パス距離 3.0m で受け手の到達半径 1.05m・敵 1.65m という逆転が
// 起きていた。同じ距離に対する所要時間で不等号を固定する。
TEST(PassFeasibilityTest, ReceiverIsModeledAtLeastAsCapableAsInterceptingEnemy)
{
  const ReceiveFeasibilityParams params;
  // PassRatingConfig::enemy_slack の既定（pass_rating.hpp）を書き写した値。
  // crane_physics は crane_msg_wrappers に依存できない（依存方向が逆）ため実体を
  // 参照できず、これは凍結コピーである。向こうを変えてもこのテストは気付かない。
  constexpr double kEnemyAcceleration = 3.0;
  constexpr double kEnemyVelocity = 5.5;
  for (double distance = 0.5; distance <= 3.0; distance += 0.5) {
    const double receiver_time = getTravelTimeTrapezoidal(
      Point(0.0, 0.0), Vector2(0.0, 0.0), Point(distance, 0.0), params.receiver_max_acceleration,
      params.receiver_max_velocity);
    const double enemy_time = getTravelTimeTrapezoidal(
      Point(0.0, 0.0), Vector2(0.0, 0.0), Point(distance, 0.0), kEnemyAcceleration, kEnemyVelocity);
    EXPECT_LE(receiver_time, enemy_time) << "distance=" << distance;
  }
}
}  // namespace crane
