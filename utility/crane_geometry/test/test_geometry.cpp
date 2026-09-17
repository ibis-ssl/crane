// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <random>
#include <vector>

namespace crane
{
// Circleのテスト
TEST(CircleTest, CreateAndMeasure)
{
  crane::Circle circle{.center = Point(0.0, 0.0), .radius = 5.0};

  Point point(10.0, 0.0);
  double distance = bg::distance(circle, point);

  EXPECT_DOUBLE_EQ(distance, 5.0);
}

// Capsuleのテスト
TEST(CapsuleTest, CreateAndMeasure)
{
  Capsule capsule{.segment = Segment(Point(0.0, 0.0), Point(10.0, 0.0)), .radius = 2.0};

  Point point(5.0, 5.0);
  double distance = bg::distance(capsule, point);

  EXPECT_DOUBLE_EQ(distance, 3.0);
}

// geometry_operationsのテスト追加
TEST(GeometryOperationsTest, NormalizeAngle)
{
  // 正の角度の正規化
  EXPECT_NEAR(normalizeAngle(3.5 * M_PI), -0.5 * M_PI, 1e-10);

  // 負の角度の正規化
  EXPECT_NEAR(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI, 1e-10);

  // -π〜πの範囲内の角度は変わらない
  EXPECT_DOUBLE_EQ(normalizeAngle(0.5), 0.5);
  EXPECT_DOUBLE_EQ(normalizeAngle(-0.5), -0.5);
}

TEST(GeometryOperationsTest, GetAngleDiff)
{
  // 単純な差
  EXPECT_DOUBLE_EQ(getAngleDiff(0.5, 0.3), 0.2);

  // -πとπの間の差（境界を超える）
  EXPECT_NEAR(getAngleDiff(M_PI - 0.1, -M_PI + 0.1), -0.2, 1e-10);

  // Pose2D間の角度差
  Pose2D pose1{.pos = Point(0.0, 0.0), .theta = 0.5};
  Pose2D pose2{.pos = Point(0.0, 0.0), .theta = -0.5};
  EXPECT_DOUBLE_EQ(getAngleDiff(pose1, pose2), 1.0);

  // Pose2DとDouble間の角度差
  EXPECT_DOUBLE_EQ(getAngleDiff(pose1, 0.0), 0.5);
  EXPECT_DOUBLE_EQ(getAngleDiff(0.0, pose1), -0.5);
}

TEST(GeometryOperationsTest, GetIntermediateAngle)
{
  // 単純な中間角度
  EXPECT_DOUBLE_EQ(getIntermediateAngle(0.0, 1.0), 0.5);

  // -πとπの間の中間角度（境界を超える）
  EXPECT_NEAR(getIntermediateAngle(M_PI - 0.1, -M_PI + 0.1), M_PI, 1e-10);
}

TEST(GeometryOperationsTest, GetCircle)
{
  // 3点から円を作成
  // (1,0)を中心とする半径1の円
  Point p1(0.0, 0.0);
  Point p2(2.0, 0.0);
  Point p3(1.0, 1.0);
  auto circle = getCircle(p1, p2, p3);

  ASSERT_TRUE(circle.has_value());
  EXPECT_NEAR(circle->center.x(), 1.0, 1e-10);
  EXPECT_NEAR(circle->center.y(), 0.0, 1e-10);
  EXPECT_NEAR(circle->radius, 1.0, 1e-10);

  // 一直線上の3点からは円を作成できない
  Point p4(3.0, 0.0);
  auto invalid_circle = getCircle(p1, p2, p4);
  EXPECT_FALSE(invalid_circle.has_value());
}

TEST(GeometryOperationsTest, Deg2RadAndRad2Deg)
{
  EXPECT_DOUBLE_EQ(deg2rad(0.0), 0.0);
  EXPECT_DOUBLE_EQ(rad2deg(0.0), 0.0);

  EXPECT_NEAR(deg2rad(180.0), M_PI, 1e-10);
  EXPECT_NEAR(rad2deg(M_PI), 180.0, 1e-10);

  EXPECT_NEAR(deg2rad(90.0), M_PI_2, 1e-10);
  EXPECT_NEAR(rad2deg(M_PI_2), 90.0, 1e-10);

  EXPECT_NEAR(deg2rad(-45.0), -M_PI / 4.0, 1e-10);
  EXPECT_NEAR(rad2deg(-M_PI / 4.0), -45.0, 1e-10);

  EXPECT_NEAR(deg2rad(360.0), 2.0 * M_PI, 1e-10);
  EXPECT_NEAR(rad2deg(2.0 * M_PI), 360.0, 1e-10);

  // float型サポートの確認
  constexpr float f_deg = 60.0f;
  constexpr float f_rad = deg2rad(f_deg);
  EXPECT_NEAR(f_rad, static_cast<float>(M_PI / 3.0), 1e-5f);
  EXPECT_NEAR(rad2deg(f_rad), f_deg, 1e-5f);
}

TEST(GeometryOperationsTest, RotateVector)
{
  Vector2 v(1.0, 0.0);

  // 90度回転
  auto v_90 = rotate(v, M_PI_2);
  EXPECT_NEAR(v_90.x(), 0.0, 1e-10);
  EXPECT_NEAR(v_90.y(), 1.0, 1e-10);

  // 180度回転
  auto v_180 = rotate(v, M_PI);
  EXPECT_NEAR(v_180.x(), -1.0, 1e-10);
  EXPECT_NEAR(v_180.y(), 0.0, 1e-10);

  // -90度回転
  auto v_neg90 = rotate(v, -M_PI_2);
  EXPECT_NEAR(v_neg90.x(), 0.0, 1e-10);
  EXPECT_NEAR(v_neg90.y(), -1.0, 1e-10);

  // 0度回転
  auto v_0 = rotate(v, 0.0);
  EXPECT_DOUBLE_EQ(v_0.x(), 1.0);
  EXPECT_DOUBLE_EQ(v_0.y(), 0.0);
}

TEST(GeometryOperationsTest, ClampNorm)
{
  Vector2 v(3.0, 4.0);  // norm = 5.0

  // 上限より大きい場合 -> 長さが max_norm に縮小され、方向は維持
  auto clamped = clampNorm(v, 2.5);
  EXPECT_NEAR(clamped.norm(), 2.5, 1e-10);
  EXPECT_NEAR(clamped.x(), 1.5, 1e-10);
  EXPECT_NEAR(clamped.y(), 2.0, 1e-10);

  // 上限以下の場合は変化なし
  auto not_clamped = clampNorm(v, 6.0);
  EXPECT_DOUBLE_EQ(not_clamped.x(), 3.0);
  EXPECT_DOUBLE_EQ(not_clamped.y(), 4.0);

  // 上限が0以下の場合はゼロベクトル
  auto zero = clampNorm(v, 0.0);
  EXPECT_DOUBLE_EQ(zero.x(), 0.0);
  EXPECT_DOUBLE_EQ(zero.y(), 0.0);
}

TEST(GeometryOperationsTest, ClampPoint)
{
  Point p(10.0, -8.0);

  // 対称範囲 [-5, 5] x [-5, 5]
  auto p_sym = clampPoint(p, 5.0, 5.0);
  EXPECT_DOUBLE_EQ(p_sym.x(), 5.0);
  EXPECT_DOUBLE_EQ(p_sym.y(), -5.0);

  // 任意範囲 [min_x, max_x] x [min_y, max_y]
  auto p_range = clampPoint(p, 0.0, 8.0, -5.0, 5.0);
  EXPECT_DOUBLE_EQ(p_range.x(), 8.0);
  EXPECT_DOUBLE_EQ(p_range.y(), -5.0);

  // Box によるクランプ
  Box box{Point(-2.0, -3.0), Point(4.0, 6.0)};
  auto p_box = clampPoint(p, box);
  EXPECT_DOUBLE_EQ(p_box.x(), 4.0);
  EXPECT_DOUBLE_EQ(p_box.y(), -3.0);
}

// getIntersections(Circle, Segment) のテスト
//
// このAPIは「返る点が円周上にあり、かつ線分上にある」ことが本質的な不変条件であり、
// 過去に垂線足ベースの実装がこれを破っていた（円周から外れた点や、線分から浮いた点を返す）。
// 個別の期待値だけでなく、必ず下の不変条件ヘルパで検証すること。
namespace
{
void expectOnCircleAndSegment(
  const Circle & circle, const Segment & segment, const std::vector<Point> & intersections)
{
  for (const auto & p : intersections) {
    EXPECT_NEAR((p - circle.center).norm(), circle.radius, 1e-6)
      << "交点が円周上にない: (" << p.x() << ", " << p.y() << ")";
    EXPECT_NEAR(bg::distance(segment, p), 0.0, 1e-6)
      << "交点が線分上にない: (" << p.x() << ", " << p.y() << ")";
  }
}
}  // namespace

// 垂線足が線分の内側に落ちる基本ケース
TEST(GeometryOperationsTest, GetIntersectionsCircleSegmentBasic)
{
  Circle circle{.center = Point(5.0, 1.0), .radius = 2.0};
  Segment segment{Point(0.0, 0.0), Point(10.0, 0.0)};

  auto intersections = getIntersections(circle, segment);
  ASSERT_EQ(intersections.size(), 2u);
  expectOnCircleAndSegment(circle, segment, intersections);
  // 始点側から終点側の順で並ぶ
  EXPECT_NEAR(intersections[0].x(), 5.0 - std::sqrt(3.0), 1e-9);
  EXPECT_NEAR(intersections[0].y(), 0.0, 1e-9);
  EXPECT_NEAR(intersections[1].x(), 5.0 + std::sqrt(3.0), 1e-9);
  EXPECT_NEAR(intersections[1].y(), 0.0, 1e-9);
}

// 垂線足が線分の外に落ちるケース（線分上の最近接点を使う実装が破綻する）
TEST(GeometryOperationsTest, GetIntersectionsCircleSegmentFootOutsideSegment)
{
  Circle circle{.center = Point(-6.0, 0.0), .radius = 2.5};
  Segment segment{Point(-5.0, 0.5), Point(-4.0, 3.5)};

  auto intersections = getIntersections(circle, segment);
  ASSERT_EQ(intersections.size(), 1u);
  expectOnCircleAndSegment(circle, segment, intersections);
  EXPECT_NEAR(intersections[0].x(), -4.5, 1e-9);
  EXPECT_NEAR(intersections[0].y(), 2.0, 1e-9);
}

// 円中心が線分の延長上にあり、円が線分を横切るケース
TEST(GeometryOperationsTest, GetIntersectionsCircleSegmentCenterBeyondSegment)
{
  Circle circle{.center = Point(2.0, 0.0), .radius = 1.5};
  Segment segment{Point(0.0, 0.0), Point(1.0, 0.0)};

  auto intersections = getIntersections(circle, segment);
  ASSERT_EQ(intersections.size(), 1u);
  expectOnCircleAndSegment(circle, segment, intersections);
  EXPECT_NEAR(intersections[0].x(), 0.5, 1e-9);
  EXPECT_NEAR(intersections[0].y(), 0.0, 1e-9);
}

// 交点がちょうど線分の端点に乗るケース
TEST(GeometryOperationsTest, GetIntersectionsCircleSegmentOnEndpoint)
{
  Circle circle{.center = Point(0.0, 0.0), .radius = 1.0};
  Segment segment{Point(1.0, 0.0), Point(3.0, 0.0)};

  auto intersections = getIntersections(circle, segment);
  ASSERT_EQ(intersections.size(), 1u);
  expectOnCircleAndSegment(circle, segment, intersections);
  EXPECT_NEAR(intersections[0].x(), 1.0, 1e-9);
  EXPECT_NEAR(intersections[0].y(), 0.0, 1e-9);
}

// 接するケースは重解なので1点に畳む
TEST(GeometryOperationsTest, GetIntersectionsCircleSegmentTangent)
{
  Circle circle{.center = Point(5.0, 2.0), .radius = 2.0};
  Segment segment{Point(0.0, 0.0), Point(10.0, 0.0)};

  auto intersections = getIntersections(circle, segment);
  ASSERT_EQ(intersections.size(), 1u);
  expectOnCircleAndSegment(circle, segment, intersections);
  EXPECT_NEAR(intersections[0].x(), 5.0, 1e-9);
  EXPECT_NEAR(intersections[0].y(), 0.0, 1e-9);
}

// 交点なしのケース
TEST(GeometryOperationsTest, GetIntersectionsCircleSegmentNoIntersection)
{
  Circle circle{.center = Point(0.0, 0.0), .radius = 5.0};

  // 線分が完全に円の内部
  EXPECT_TRUE(getIntersections(circle, Segment{Point(-1.0, 0.0), Point(1.0, 0.0)}).empty());
  // 線分が完全に円の外部
  EXPECT_TRUE(getIntersections(circle, Segment{Point(6.0, 0.0), Point(8.0, 0.0)}).empty());
  // 長さ0の線分（円周上に乗っていても方向が定義できないので交点なし）
  EXPECT_TRUE(getIntersections(circle, Segment{Point(5.0, 0.0), Point(5.0, 0.0)}).empty());
}

// ランダムな配置でも不変条件が破れないことを確認する（シード固定）
TEST(GeometryOperationsTest, GetIntersectionsCircleSegmentInvariants)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> pos(-6.0, 6.0);
  std::uniform_real_distribution<double> rad(0.1, 4.0);

  for (int i = 0; i < 2000; ++i) {
    Circle circle{.center = Point(pos(rng), pos(rng)), .radius = rad(rng)};
    Segment segment{Point(pos(rng), pos(rng)), Point(pos(rng), pos(rng))};

    auto intersections = getIntersections(circle, segment);
    ASSERT_LE(intersections.size(), 2u);
    expectOnCircleAndSegment(circle, segment, intersections);
  }
}

}  // namespace crane
