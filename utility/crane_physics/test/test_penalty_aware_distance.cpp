// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_physics/penalty_aware_distance.hpp>

namespace crane
{

// SSL Division A のフィールド標準値
// ペナルティエリア: depth=1.8m, width=3.6m
// フィールド: 12m x 9m、ゴール位置 x=±6m
static constexpr double FIELD_HALF_X = 6.0;
static constexpr double PA_DEPTH = 1.8;
static constexpr double PA_HALF_WIDTH = 1.8;

// テスト用ペナルティエリア設定（自陣: x<0側, 敵陣: x>0側）
static Box makeOurPA()
{
  // 自陣ゴール = (-6, 0), PAは x: -6 ~ -4.2, y: -1.8 ~ 1.8
  return Box(Point(-FIELD_HALF_X, -PA_HALF_WIDTH), Point(-FIELD_HALF_X + PA_DEPTH, PA_HALF_WIDTH));
}

static Box makeTheirPA()
{
  // 敵陣ゴール = (6, 0), PAは x: 4.2 ~ 6, y: -1.8 ~ 1.8
  return Box(Point(FIELD_HALF_X - PA_DEPTH, -PA_HALF_WIDTH), Point(FIELD_HALF_X, PA_HALF_WIDTH));
}

static Point ourGoal() { return Point(-FIELD_HALF_X, 0.0); }
static Point theirGoal() { return Point(FIELD_HALF_X, 0.0); }
static Point paSize() { return Point(PA_DEPTH, PA_HALF_WIDTH * 2.0); }

// ユーティリティ: テスト用 estimatePenaltyAwareDistance の呼び出しヘルパー
double estimate(const Point & start, const Point & target)
{
  return estimatePenaltyAwareDistance(
    start, target, makeOurPA(), ourGoal(), makeTheirPA(), theirGoal(), paSize());
}

/// ケース1: 経路がペナルティエリアを横切らない → ユークリッド距離と一致
TEST(PenaltyAwareDistanceTest, NoIntersection_ReturnsEuclidean)
{
  // フィールド中央付近の2点（ペナルティエリアから離れている）
  Point start(0.0, 0.0);
  Point target(1.0, 0.0);

  double result = estimate(start, target);
  double euclidean = (target - start).norm();

  EXPECT_DOUBLE_EQ(result, euclidean);
}

/// ケース2: 経路が自陣ペナルティエリアを横切る場合 → ユークリッドより大きい
TEST(PenaltyAwareDistanceTest, CrossesOurPA_ReturnsLargerThanEuclidean)
{
  // 自陣ゴール裏 → フィールド中央 (直線がPAを横切る)
  Point start(-5.5, 2.5);    // PA外の上方
  Point target(-5.5, -2.5);  // PA外の下方（直線がPAを横切る）

  double result = estimate(start, target);
  double euclidean = (target - start).norm();

  // 迂回が発生するので距離は増大するはず
  EXPECT_GT(result, euclidean);
}

/// ケース3: 経路が敵陣ペナルティエリアを横切る場合 → ユークリッドより大きい
TEST(PenaltyAwareDistanceTest, CrossesTheirPA_ReturnsLargerThanEuclidean)
{
  // 敵陣PA付近の上側 → 下側（直線がPAを横切る）
  Point start(5.5, 2.5);
  Point target(5.5, -2.5);

  double result = estimate(start, target);
  double euclidean = (target - start).norm();

  EXPECT_GT(result, euclidean);
}

/// ケース4: ペナルティエリアを横切らないフィールド端の移動
TEST(PenaltyAwareDistanceTest, AlongSideLine_NoDetour)
{
  // y軸方向の移動（x=0）ならPAを通らない
  Point start(0.0, -3.0);
  Point target(0.0, 3.0);

  double result = estimate(start, target);
  double euclidean = (target - start).norm();

  EXPECT_DOUBLE_EQ(result, euclidean);
}

/// ケース5: 対称性テスト（上/下どちらの迂回が選択されるか）
TEST(PenaltyAwareDistanceTest, ChoosesShorterDetour)
{
  // ロボットが上側にいてターゲットが下側 → 上側の角を迂回する方が近い
  Point start(-3.0, 3.0);   // フィールド中央、y>0側
  Point target(-5.5, 0.0);  // 自陣PA前端付近

  double result = estimate(start, target);

  // 上側の角を経由する距離
  constexpr double surr_off = 0.2;
  Point our_goal = ourGoal();
  Point pa_sz = paSize();
  Point corner_upper =
    our_goal +
    Point(std::copysign(pa_sz.x() + surr_off, -our_goal.x()), pa_sz.y() * 0.5 + surr_off);
  Point corner_lower =
    our_goal +
    Point(std::copysign(pa_sz.x() + surr_off, -our_goal.x()), -(pa_sz.y() * 0.5 + surr_off));

  double dist_upper = (corner_upper - start).norm() + (target - corner_upper).norm();
  double dist_lower = (corner_lower - start).norm() + (target - corner_lower).norm();

  // 結果は2つの迂回距離の小さい方と一致するはず（交差する場合）
  Segment path(start, target);
  Box our_pa = makeOurPA();
  our_pa.min_corner() -= Point(0.1, 0.1);
  our_pa.max_corner() += Point(0.1, 0.1);
  if (bg::intersects(path, our_pa)) {
    EXPECT_DOUBLE_EQ(result, std::min(dist_upper, dist_lower));
  } else {
    EXPECT_DOUBLE_EQ(result, (target - start).norm());
  }
}

/// ケース6: 距離ゼロの場合
TEST(PenaltyAwareDistanceTest, ZeroDistance)
{
  Point pos(0.0, 0.0);
  double result = estimate(pos, pos);
  EXPECT_DOUBLE_EQ(result, 0.0);
}

}  // namespace crane
