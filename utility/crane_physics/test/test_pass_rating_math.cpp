// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <crane_physics/pass_rating_math.hpp>

namespace crane
{
namespace
{
// 他項を中立（スコアに影響しない）にした基準 terms。
// distance_factor=1.0, bonus=0, penalty=0, their_goal_factor=1.0,
// intercept=1.0, shadow=1.0 → score=1.0 になる。
auto neutralTerms() -> PassScoreTerms
{
  PassScoreTerms t;
  t.pass_distance = 2.0;                  // 1.5〜4.0m → distance_factor=1.0
  t.their_goal_angle_width = 0.0;         // bonus=0
  t.own_goal_angle_width = 0.0;           // penalty=0
  t.normed_distance_to_their_goal = 0.0;  // their_goal_factor=1.0
  t.worst_enemy_slack = 1.0;              // intercept=clamp(1.0)=1.0
  t.slack_scale = 1.0;
  t.shadow_score = 1.0;
  t.in_penalty_area = false;
  return t;
}
}  // namespace

// ── 距離帯（山型）────────────────────────────────────────────────

TEST(PassRatingMathTest, DistanceFactor_Zones)
{
  auto t = neutralTerms();
  t.pass_distance = 0.75;  // <1.5 → 0.75/1.5 = 0.5
  EXPECT_DOUBLE_EQ(combinePassScore(t).distance_factor, 0.5);
  EXPECT_DOUBLE_EQ(combinePassScore(t).score, 0.5);

  t.pass_distance = 2.0;  // 最適ゾーン → 1.0
  EXPECT_DOUBLE_EQ(combinePassScore(t).distance_factor, 1.0);

  t.pass_distance = 5.0;  // >4.0 → max(0.2, 1-1*0.15)=0.85
  EXPECT_DOUBLE_EQ(combinePassScore(t).distance_factor, 0.85);

  t.pass_distance = 10.0;  // >4.0 → max(0.2, 1-6*0.15=0.1)=0.2 (下限)
  EXPECT_DOUBLE_EQ(combinePassScore(t).distance_factor, 0.2);
}

// ── 敵ゴール見通しボーナス / 自ゴールペナルティの clamp ──────────

TEST(PassRatingMathTest, GoalAngleBonus_ClampSaturation)
{
  auto t = neutralTerms();
  t.their_goal_angle_width = M_PI / 12.;  // ratio=1.0 → clamp(1.0,0,0.5)=0.5
  EXPECT_DOUBLE_EQ(combinePassScore(t).goal_angle_bonus, 0.5);

  t.their_goal_angle_width = -1.0;  // 負 → 0
  EXPECT_DOUBLE_EQ(combinePassScore(t).goal_angle_bonus, 0.0);
}

TEST(PassRatingMathTest, OwnGoalPenalty_ClampSaturation)
{
  auto t = neutralTerms();
  t.own_goal_angle_width = 10.0 * (M_PI / 12.);  // ratio>>0.5 → 0.5
  EXPECT_DOUBLE_EQ(combinePassScore(t).own_goal_penalty, 0.5);
}

// ── 敵インターセプト係数の clamp（符号込み）───────────────────────

TEST(PassRatingMathTest, InterceptScore_Clamp)
{
  auto t = neutralTerms();
  t.worst_enemy_slack = -0.5;  // 敵が奪える → clamp(負)=0 → score 0
  EXPECT_DOUBLE_EQ(combinePassScore(t).intercept_score, 0.0);
  EXPECT_DOUBLE_EQ(combinePassScore(t).score, 0.0);

  t.worst_enemy_slack = 2.0;  // 十分安全 → clamp(2.0)=1.0
  EXPECT_DOUBLE_EQ(combinePassScore(t).intercept_score, 1.0);

  t.worst_enemy_slack = 0.3;
  t.slack_scale = 0.6;  // 0.3/0.6=0.5
  EXPECT_DOUBLE_EQ(combinePassScore(t).intercept_score, 0.5);
}

// ── ペナルティエリアは他項に関わらず 0 ───────────────────────────

TEST(PassRatingMathTest, PenaltyArea_ForcesZero)
{
  auto t = neutralTerms();
  t.their_goal_angle_width = M_PI / 12.;  // 本来スコアを押し上げる項
  t.in_penalty_area = true;
  EXPECT_DOUBLE_EQ(combinePassScore(t).score, 0.0);
}

// ── 全項合成：演算順（+= と *= の混在）を固定する ──────────────────

TEST(PassRatingMathTest, FullCombine_OrderPinned)
{
  PassScoreTerms t;
  t.pass_distance = 3.0;                  // distance_factor=1.0
  t.their_goal_angle_width = M_PI / 12.;  // bonus=0.5
  t.own_goal_angle_width = 0.0;           // penalty=0
  t.normed_distance_to_their_goal = 0.2;  // their_goal_factor=1-0.1=0.9
  t.worst_enemy_slack = 0.5;              // intercept=0.5
  t.slack_scale = 1.0;
  t.shadow_score = 0.8;
  t.in_penalty_area = false;

  // score = ((1.0*1.0 + 0.5 - 0.0) * 0.9) * 0.5 * 0.8 = 1.35*0.5*0.8 = 0.54
  const auto r = combinePassScore(t);
  EXPECT_DOUBLE_EQ(r.distance_factor, 1.0);
  EXPECT_DOUBLE_EQ(r.goal_angle_bonus, 0.5);
  EXPECT_DOUBLE_EQ(r.their_goal_factor, 0.9);
  EXPECT_DOUBLE_EQ(r.intercept_score, 0.5);
  EXPECT_DOUBLE_EQ(r.score, 0.54);
}
}  // namespace crane
