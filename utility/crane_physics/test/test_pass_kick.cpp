// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <crane_physics/pass_kick.hpp>
#include <memory>
#include <vector>

namespace crane
{
namespace
{
constexpr double kA = 0.7;  // ボール減速度 [m/s^2]

auto makeEnemy(double x, double y) -> RobotInfo::SharedPtr
{
  auto r = std::make_shared<RobotInfo>();
  r->pose = Pose2D{.pos = Point(x, y), .theta = 0.0};
  r->vel = Velocity2D{.linear = Point(0.0, 0.0), .omega = 0.0};
  return r;
}
}  // namespace

// ── requiredInitialSpeed ──────────────────────────────────────────

TEST(PassKickTest, RequiredInitialSpeed_ZeroDistance)
{
  // 距離0なら到達速度がそのまま必要初速
  EXPECT_NEAR(requiredInitialSpeed(0.0, 2.5, kA), 2.5, 1e-9);
}

TEST(PassKickTest, RequiredInitialSpeed_ZeroArrival)
{
  // 到達速度0 → v0 = sqrt(2 a d)
  EXPECT_NEAR(requiredInitialSpeed(2.0, 0.0, kA), std::sqrt(2.0 * kA * 2.0), 1e-9);
}

// ── arrivalSpeed（requiredInitialSpeed の逆関数）──────────────────

TEST(PassKickTest, ArrivalSpeed_RoundTrip)
{
  const double d = 3.0, v_arr = 2.0;
  const double v0 = requiredInitialSpeed(d, v_arr, kA);
  EXPECT_NEAR(arrivalSpeed(d, v0, kA), v_arr, 1e-9);
}

TEST(PassKickTest, ArrivalSpeed_Unreachable)
{
  // 初速が小さすぎて届かない → 0
  EXPECT_DOUBLE_EQ(arrivalSpeed(10.0, 1.0, kA), 0.0);
}

// ── rollingTravelTime ─────────────────────────────────────────────

TEST(PassKickTest, RollingTravelTime_Basic)
{
  const double d = 3.0, v_arr = 2.0;
  const double v0 = requiredInitialSpeed(d, v_arr, kA);
  // t = (v0 - v_arr) / a
  EXPECT_NEAR(rollingTravelTime(d, v0, kA), (v0 - v_arr) / kA, 1e-9);
}

TEST(PassKickTest, RollingTravelTime_Unreachable)
{
  EXPECT_TRUE(std::isinf(rollingTravelTime(10.0, 1.0, kA)));
}

TEST(PassKickTest, RollingTravelTime_NoDeceleration)
{
  // 減速なし → 等速 d / v0
  EXPECT_NEAR(rollingTravelTime(4.0, 2.0, 0.0), 2.0, 1e-9);
}

TEST(PassKickTest, RollingTravelTime_ZeroDistance)
{
  EXPECT_DOUBLE_EQ(rollingTravelTime(0.0, 3.0, kA), 0.0);
}

// ── planStraightPass ──────────────────────────────────────────────

TEST(PassKickTest, PlanStraightPass_Nominal)
{
  const auto plan = planStraightPass(3.0, 2.0, kA, 1.0, 6.0);
  EXPECT_TRUE(plan.reachable);
  EXPECT_NEAR(plan.initial_speed, requiredInitialSpeed(3.0, 2.0, kA), 1e-9);
  EXPECT_NEAR(plan.arrival_speed, 2.0, 1e-9);
  EXPECT_GT(plan.travel_time, 0.0);
}

TEST(PassKickTest, PlanStraightPass_MaxClamp_Unreachable)
{
  // 遠距離＋低い上限 → 上限にクランプされ届かない
  const auto plan = planStraightPass(10.0, 2.0, kA, 1.0, 3.0);
  EXPECT_DOUBLE_EQ(plan.initial_speed, 3.0);
  EXPECT_DOUBLE_EQ(plan.arrival_speed, 0.0);
  EXPECT_FALSE(plan.reachable);
  EXPECT_TRUE(std::isinf(plan.travel_time));
}

TEST(PassKickTest, PlanStraightPass_MinClamp_FasterArrival)
{
  // 近距離＋高い下限 → 下限にクランプされ望ましい到達速度より速く着く
  const auto plan = planStraightPass(0.5, 2.0, kA, 3.0, 6.0);
  EXPECT_DOUBLE_EQ(plan.initial_speed, 3.0);
  EXPECT_GT(plan.arrival_speed, 2.0);
  EXPECT_TRUE(plan.reachable);
}

// ── planPassKick（チップ/直進の統合）──────────────────────────────

TEST(PassKickTest, PlanPassKick_NoEnemies_Straight)
{
  const auto plan = planPassKick(Point(0, 0), Point(3, 0), {}, 2.0, kA, 1.0, 6.0);
  EXPECT_FALSE(plan.is_chip);
  EXPECT_TRUE(plan.feasible);
  EXPECT_TRUE(plan.straight.reachable);
}

TEST(PassKickTest, PlanPassKick_BlockedEnemy_Chip)
{
  // パスライン(0,0)-(4,0)上に敵(2,0.1) → 遮蔽ありチップ
  std::vector<crane::RobotInfo::SharedPtr> enemies = {makeEnemy(2.0, 0.1)};
  const auto plan = planPassKick(Point(0, 0), Point(4, 0), enemies, 2.0, kA, 1.0, 6.0, 0.2, 0.2);
  EXPECT_TRUE(plan.is_chip);
  // ball から遮蔽点(2,0)までの距離 2.0 + margin 0.2
  EXPECT_NEAR(plan.chip_distance, 2.2, 1e-6);
  EXPECT_TRUE(plan.feasible);
}

TEST(PassKickTest, PlanPassKick_ClearLine_Straight)
{
  // 敵(2,1.0)はパスラインから十分離れている → 直進
  std::vector<crane::RobotInfo::SharedPtr> enemies = {makeEnemy(2.0, 1.0)};
  const auto plan = planPassKick(Point(0, 0), Point(4, 0), enemies, 2.0, kA, 1.0, 6.0, 0.2, 0.2);
  EXPECT_FALSE(plan.is_chip);
  EXPECT_TRUE(plan.feasible);
}
}  // namespace crane
