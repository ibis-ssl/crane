// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>

#include "crane_local_planner/ateb_spatial_optimizer.hpp"
#include "crane_local_planner/ateb_types.hpp"
#include "test_helpers.hpp"

using crane::Point;
using crane::ateb::ElasticBand;
using crane::ateb::HomotopyClass;
using crane::ateb::Obstacle;
using crane::ateb::SpatialOptimizer;

namespace
{

SpatialOptimizer makeSO()
{
  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  return so;
}

// 直線ホモトピーを作成（始点から終点への直線）
HomotopyClass makeLinearHomotopy(const Point & start, const Point & goal, int n_waypoints = 5)
{
  HomotopyClass homotopy;
  for (int i = 0; i < n_waypoints; ++i) {
    const double alpha = static_cast<double>(i) / (n_waypoints - 1);
    homotopy.waypoints.push_back(start * (1.0 - alpha) + goal * alpha);
  }
  homotopy.cost = (goal - start).norm();
  return homotopy;
}

// ペナルティエリアを上側から迂回するウェイポイント（角を経由）
HomotopyClass makeAroundPenaltyHomotopy(const Point & start, const Point & goal)
{
  HomotopyClass homotopy;
  // PA(inflation後): x=[4.1, 6.1], y=[-1.9, 1.9]
  // 上側迂回: 左上角の上を通る
  homotopy.waypoints = {start, {3.5, 2.5}, {5.0, 2.5}, goal};
  homotopy.cost = 0.0;
  for (size_t i = 1; i < homotopy.waypoints.size(); ++i) {
    homotopy.cost += (homotopy.waypoints[i] - homotopy.waypoints[i - 1]).norm();
  }
  return homotopy;
}

}  // namespace

// ===== 基本テスト =====

TEST(SpatialOptimizer, StraightBandNoObstacles)
{
  // 障害物なし、直線ホモトピー → バンドはほぼ直線を維持
  const auto so = makeSO();
  const Point start(-2.0, 0.0);
  const Point goal(2.0, 0.0);
  const auto homotopy = makeLinearHomotopy(start, goal);
  const auto band = so.optimize(homotopy, start, goal, {});

  EXPECT_TRUE(band.isValid());
  EXPECT_EQ(band.nodes.front().pos, start);
  EXPECT_EQ(band.nodes.back().pos, goal);

  // 全ノードがstart-goal直線近傍にあること（y≈0）
  for (const auto & node : band.nodes) {
    EXPECT_NEAR(node.pos.y(), 0.0, 0.05) << "直線バンドのy座標が大きくずれている: " << node.pos.y();
  }
}

TEST(SpatialOptimizer, BandAvoidsBoxObstacle)
{
  // ペナルティエリアBOXを迂回するバンドの最適化
  const auto so = makeSO();
  const Point start(3.0, 0.0);
  const Point goal(5.5, 3.5);
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};

  const auto homotopy = makeAroundPenaltyHomotopy(start, goal);
  const auto band = so.optimize(homotopy, start, goal, obstacles);

  EXPECT_TRUE(band.isValid());
  EXPECT_EQ(band.nodes.front().pos, start);
  EXPECT_EQ(band.nodes.back().pos, goal);

  // 全ノードが障害物の外側（またはsafety_margin内）にあること
  const auto & obs = obstacles[0];
  for (size_t i = 0; i < band.nodes.size(); ++i) {
    const double d = obs.distance(band.nodes[i].pos);
    EXPECT_GE(d, -0.05)  // わずかな侵入は許容（最適化の限界）
      << "ノード" << i << "がペナルティエリアに侵入: dist=" << d << " pos=("
      << band.nodes[i].pos.x() << ", " << band.nodes[i].pos.y() << ")";
  }
}

TEST(SpatialOptimizer, BandNearPenaltyCorner)
{
  // ペナルティエリアの角近傍でのスムーズネス vs 障害物トレードオフ
  // スムーズネス目的が角にノードを引き込もうとするが、障害物目的が押し出すはず
  const auto so = makeSO();
  const Point start(3.5, 2.5);
  const Point goal(5.5, 2.5);  // PA上辺の上側を水平移動
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};

  const auto homotopy = makeLinearHomotopy(start, goal);
  const auto band = so.optimize(homotopy, start, goal, obstacles);

  EXPECT_TRUE(band.isValid());

  // PA上辺(inflation後 y=1.9)より上にあるべき
  for (const auto & node : band.nodes) {
    EXPECT_GE(node.pos.y(), 1.9 - 0.05) << "ノードがPA上辺を下回っている: y=" << node.pos.y();
  }
}

TEST(SpatialOptimizer, NodeInsideBoxProducesResidual)
{
  // PA内部にノードを持つバンドは非ゼロの障害物残差を持つべき
  // これによりoptimizeステップがノードを外側に押し出す
  const auto so = makeSO();
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};

  // PA内部を通るホモトピーを作成（正常な迂回ではない）
  HomotopyClass inside_homotopy;
  inside_homotopy.waypoints = {
    {3.0, 0.0},  // PA外
    {5.0, 0.0},  // PA内（BOXはx=[4.1,6.1]なので内部）
    {7.0, 0.0}   // PA外（実際はフィールド外だが距離テスト用）
  };
  inside_homotopy.cost = 4.0;

  // 最適化後はPA内部を避ける方向に更新される
  const auto band = so.optimize(inside_homotopy, {3.0, 0.0}, {7.0, 0.0}, obstacles);
  EXPECT_TRUE(band.isValid());

  // 中間ノードは最終的にPA外側に移動すべきだが、
  // 5イテレーションでは完全収束しない可能性あり
  // この行動を確認するために残差チェック
  const auto & obs = obstacles[0];
  int nodes_inside = 0;
  for (size_t i = 1; i + 1 < band.nodes.size(); ++i) {
    if (obs.distance(band.nodes[i].pos) < 0.0) {
      ++nodes_inside;
    }
  }
  // 最適化後、PA内部にノードが残る場合は問題（バグの影響）
  if (nodes_inside > 0) {
    ADD_FAILURE() << nodes_inside
                  << "個のノードがペナルティエリア内に残っている。"
                     "5イテレーションでは不十分かもしれない。max_iterationsを増やすべき";
  }
}

TEST(SpatialOptimizer, ObstacleSafetyMarginRespected)
{
  // 最適化後、全ノードがobstacle_safety_margin以上の距離を持つべき
  auto cfg = test_helpers::makeDefaultSOConfig();
  cfg.max_iterations = 20;  // より多くのイテレーション
  SpatialOptimizer so;
  so.configure(cfg);

  const Point start(3.0, 0.0);
  const Point goal(5.5, 3.5);
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const auto homotopy = makeAroundPenaltyHomotopy(start, goal);

  const auto band = so.optimize(homotopy, start, goal, obstacles);
  EXPECT_TRUE(band.isValid());

  const auto & obs = obstacles[0];
  for (size_t i = 1; i + 1 < band.nodes.size(); ++i) {
    const double d = obs.distance(band.nodes[i].pos);
    EXPECT_GE(d, cfg.obstacle_safety_margin - 1e-4)
      << "ノード" << i << "がobstacle_safety_margin(" << cfg.obstacle_safety_margin
      << "m)未満: dist=" << d;
  }
}

TEST(SpatialOptimizer, NarrowPassageConvergence)
{
  // ペナルティエリアと上壁の間の狭路でのバンド収束
  const auto so = makeSO();
  std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const auto walls = test_helpers::makeFieldWalls();
  obstacles.insert(obstacles.end(), walls.begin(), walls.end());

  const Point start(3.0, 3.5);
  const Point goal(6.5, 3.5);

  // 狭路を通るホモトピー
  HomotopyClass homotopy;
  homotopy.waypoints = {start, {4.0, 3.5}, {5.0, 3.5}, goal};
  homotopy.cost = 3.5;

  const auto band = so.optimize(homotopy, start, goal, obstacles);
  EXPECT_TRUE(band.isValid());

  // バンドが有効な経路を保つこと（クラッシュしない）
  EXPECT_EQ(band.nodes.front().pos, start);
  EXPECT_EQ(band.nodes.back().pos, goal);
}

TEST(SpatialOptimizer, GradientDiscontinuityAtCorner)
{
  // BOX角点での勾配不連続による数値的不安定性テスト
  // 角点ではdistanceGradient()が-45度か+45度かを浮動小数点精度で決める
  const auto so = makeSO();
  const auto & obs = test_helpers::makeOurPenaltyObstacle();

  // PA(inflation後)の左上角(4.1, 1.9)上のノードを持つバンド
  HomotopyClass homotopy;
  homotopy.waypoints = {
    {3.0, 0.0},
    {4.1, 1.9},  // 角点ちょうど
    {5.5, 3.5}};
  homotopy.cost = 4.0;

  // クラッシュや無限ループが発生しないこと
  EXPECT_NO_THROW({
    const auto band = so.optimize(homotopy, {3.0, 0.0}, {5.5, 3.5}, {obs});
    EXPECT_TRUE(band.isValid());
  });
}

TEST(SpatialOptimizer, ReoptimizeWithChangedGoal)
{
  // 既存バンドをウォームスタートとして、終点変更後に再最適化
  const auto so = makeSO();
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const Point start(3.0, 0.0);
  const Point goal1(5.5, 3.5);
  const Point goal2(5.5, -3.5);  // 上から下へ変更

  // 最初の最適化
  const auto homotopy1 = makeAroundPenaltyHomotopy(start, goal1);
  auto band = so.optimize(homotopy1, start, goal1, obstacles);
  EXPECT_TRUE(band.isValid());

  // 終点を変更して再最適化
  band.nodes.back().pos = goal2;
  const auto reoptimized = so.reoptimize(band, obstacles);
  EXPECT_TRUE(reoptimized.isValid());
  EXPECT_EQ(reoptimized.nodes.back().pos, goal2);
}

TEST(SpatialOptimizer, CircleObstacleOptimizationBug)
{
  // [バグ1の影響] Circleのdistance()がradiusを引かないため、
  // バンドノードがCircle障害物に侵入しても残差が正しく計算されない
  const auto so = makeSO();
  constexpr double r = 0.5;  // 大きいCircle障害物
  const auto obs = Obstacle::makeCircle(Point(0.0, 0.0), r);
  const std::vector<Obstacle> obstacles = {obs};

  // Circle中心を通る直線経路
  HomotopyClass homotopy;
  homotopy.waypoints = {{-2.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}};
  homotopy.cost = 4.0;

  const auto band = so.optimize(homotopy, {-2.0, 0.0}, {2.0, 0.0}, obstacles);
  EXPECT_TRUE(band.isValid());

  // 中間ノードがCircle表面の外側にあるべき
  // [バグ1] distance()がradiusを引かないため、中間ノードが円内部に留まる可能性
  int nodes_inside = 0;
  for (size_t i = 1; i + 1 < band.nodes.size(); ++i) {
    const double actual_dist = (band.nodes[i].pos - obs.circle.center).norm() - obs.circle.radius;
    if (actual_dist < 0.0) {
      ++nodes_inside;
    }
  }
  EXPECT_EQ(nodes_inside, 0) << "[バグ1確認] " << nodes_inside
                             << "個のノードがCircle内部に残っている。"
                                "Obstacle::distance()がCircleのradiusを引かないため回避できない";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
