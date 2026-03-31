// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "crane_local_planner/ateb_cbf_filter.hpp"
#include "crane_local_planner/ateb_spatial_optimizer.hpp"
#include "crane_local_planner/ateb_types.hpp"
#include "crane_local_planner/ateb_visibility_graph.hpp"
#include "test_helpers.hpp"

using crane::Point;
using crane::Vector2;
using crane::ateb::CBFFilter;
using crane::ateb::ElasticBand;
using crane::ateb::HomotopyClass;
using crane::ateb::Obstacle;
using crane::ateb::SpatialOptimizer;
using crane::ateb::VisibilityGraph;

namespace
{

// VG → SO → CBF フルパイプライン実行
// @return CBFフィルタ後の速度ベクトル
Vector2 runFullPipeline(
  const Point & start, const Point & goal, const Vector2 & approach_vel,
  const std::vector<Obstacle> & obstacles, double max_vel = 2.0)
{
  // Phase 1: Visibility Graph
  VisibilityGraph vg;
  vg.configure(test_helpers::makeDefaultVGConfig());
  const auto homotopies = vg.extract(start, goal, obstacles);
  if (homotopies.empty()) {
    return Vector2::Zero();
  }

  // Phase 2: Spatial Optimizer
  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  ElasticBand best_band;
  for (const auto & homotopy : homotopies) {
    const auto band = so.optimize(homotopy, start, goal, obstacles);
    if (!best_band.isValid() || band.total_cost < best_band.total_cost) {
      best_band = band;
    }
  }
  if (!best_band.isValid() || best_band.nodes.size() < 2) {
    return approach_vel;
  }

  // Phase 3: 速度方向（バンドの最初のセグメント方向）
  const Vector2 direction = (best_band.nodes[1].pos - best_band.nodes[0].pos).normalized();
  const Vector2 nominal_vel = direction * std::min(max_vel, approach_vel.norm());

  // Phase 4: CBF Filter
  CBFFilter cbf;
  cbf.configure(test_helpers::makeDefaultCBFConfig());
  // 静的障害物としてBOX障害物を渡す（Circle/Capsuleは動的として別管理）
  std::vector<Obstacle> static_obs;
  std::vector<std::pair<Point, Vector2>> dynamic_obs;
  for (const auto & obs : obstacles) {
    if (obs.type == Obstacle::Type::BOX) {
      static_obs.push_back(obs);
    } else {
      dynamic_obs.emplace_back(obs.circle.center, Vector2::Zero());
    }
  }
  return cbf.filter(start, nominal_vel, dynamic_obs, static_obs);
}

// バンドの全ノードが指定BOX障害物の外側にあるか確認
bool bandClearsBox(const ElasticBand & band, const crane::Box & box, double margin = 0.0)
{
  const auto obs = Obstacle::makeBox(box);
  for (const auto & node : band.nodes) {
    if (obs.distance(node.pos) < margin) {
      return false;
    }
  }
  return true;
}

}  // namespace

// ===== ペナルティエリア横断テスト =====

TEST(ATEBIntegration, CrossPenaltyAreaSide)
{
  // PA横を上下に移動: (3.0, -3.0) → (3.0, 3.0)
  // PA(x=[4.2,6.0])は右側にあるので直線を妨げないが、
  // ここではPA左辺のすぐ右を上下に通る経路をテスト
  const std::vector<Obstacle> obstacles = {
    test_helpers::makeOurPenaltyObstacle(),
    test_helpers::makeTheirPenaltyObstacle(),
  };

  const Vector2 u = runFullPipeline({3.0, -3.0}, {3.0, 3.0}, {0.0, 2.0}, obstacles);

  EXPECT_TRUE(std::isfinite(u.x()));
  EXPECT_TRUE(std::isfinite(u.y()));
  // y方向への移動成分があること
  EXPECT_GT(u.y(), -0.5) << "上方向へ移動できるべき";
}

TEST(ATEBIntegration, ChaseNearPenaltyCorner)
{
  // PA角近傍での経路追跡
  // (3.5, 2.5) → (4.5, 2.5): PA上辺の上側を移動
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const Point start(3.5, 2.5);
  const Point goal(4.5, 2.5);

  VisibilityGraph vg;
  vg.configure(test_helpers::makeDefaultVGConfig());
  const auto homotopies = vg.extract(start, goal, obstacles);
  ASSERT_FALSE(homotopies.empty());

  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  const auto band = so.optimize(homotopies[0], start, goal, obstacles);
  EXPECT_TRUE(band.isValid());

  // バンドがPA上辺(inflation後 y=1.9)より上にあること
  for (const auto & node : band.nodes) {
    EXPECT_GE(node.pos.y(), 1.9 - 0.05) << "ノードがPA上辺を下回った: y=" << node.pos.y();
  }
}

TEST(ATEBIntegration, NarrowFieldWallCorridor)
{
  // PAと上壁の間の狭路を通過
  // PA上辺(inflation後): y=1.9, 壁(y=4.5 - inflation)
  std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const auto walls = test_helpers::makeFieldWalls();
  obstacles.insert(obstacles.end(), walls.begin(), walls.end());

  const Vector2 u = runFullPipeline({3.5, 3.5}, {5.5, 3.5}, {1.0, 0.0}, obstacles);

  EXPECT_TRUE(std::isfinite(u.x()));
  EXPECT_TRUE(std::isfinite(u.y()));
}

TEST(ATEBIntegration, GoalieInsidePenaltyArea)
{
  // ゴーリーの目標がPA内部（意図的なケース）
  // [バグ6] 目標PA内部 → extract()がgoalをPA外に射影後、元のPA内部goalに復元
  //        → バンドの最後のノードがPA内部に戻される
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const Point start(3.0, 0.0);
  const Point goal_inside(5.5, 0.0);  // PA内部

  VisibilityGraph vg;
  vg.configure(test_helpers::makeDefaultVGConfig());
  const auto homotopies = vg.extract(start, goal_inside, obstacles);
  ASSERT_FALSE(homotopies.empty());

  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  const auto band = so.optimize(homotopies[0], start, goal_inside, obstacles);
  EXPECT_TRUE(band.isValid());

  // バンドの終点がPA内部
  const double goal_dist = obstacles[0].distance(band.nodes.back().pos);
  if (goal_dist < 0.0) {
    // ゴーリーシナリオでは許容されるが、中間ノードはPA外であるべき
    int inside_count = 0;
    for (size_t i = 1; i + 1 < band.nodes.size(); ++i) {
      if (obstacles[0].distance(band.nodes[i].pos) < 0.0) {
        ++inside_count;
      }
    }
    EXPECT_EQ(inside_count, 0) << "[バグ6確認] " << inside_count << "個の中間ノードがPA内部にある";
  }
}

TEST(ATEBIntegration, OppositeFieldEndToEnd)
{
  // フィールド対角を横断: (-5.0, 3.0) → (5.0, -3.0)
  // 両方のペナルティエリアを避けながら長距離移動
  const std::vector<Obstacle> obstacles = {
    test_helpers::makeOurPenaltyObstacle(),
    test_helpers::makeTheirPenaltyObstacle(),
  };

  VisibilityGraph vg;
  vg.configure(test_helpers::makeDefaultVGConfig());
  const auto homotopies = vg.extract({-5.0, 3.0}, {5.0, -3.0}, obstacles);
  ASSERT_FALSE(homotopies.empty());

  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  const auto band = so.optimize(homotopies[0], {-5.0, 3.0}, {5.0, -3.0}, obstacles);
  EXPECT_TRUE(band.isValid());

  // 両方のPAをクリアすること
  EXPECT_TRUE(bandClearsBox(band, test_helpers::makeOurPenaltyBox(), -0.1))
    << "バンドが自陣PAに侵入している";
  EXPECT_TRUE(bandClearsBox(band, test_helpers::makeTheirPenaltyBox(), -0.1))
    << "バンドが敵陣PAに侵入している";
}

TEST(ATEBIntegration, StationaryNearPenaltyEdge)
{
  // start == goal（PA近傍で静止）→ ゼロ速度またはわずかな速度
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const Point pos(4.15, 0.0);  // PA左辺(inflation後x=4.1)の外側0.05m

  const Vector2 u = runFullPipeline(pos, pos, Vector2::Zero(), obstacles, 0.1);
  EXPECT_TRUE(std::isfinite(u.x()));
  EXPECT_TRUE(std::isfinite(u.y()));
  EXPECT_NEAR(u.norm(), 0.0, 0.5) << "始点=終点のとき速度は小さいはず";
}

TEST(ATEBIntegration, PenaltyPlusMultipleRobots)
{
  // PA + 複数ロボット（Circle）混在のシナリオ
  std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  // PA角付近にロボットを配置
  obstacles.push_back(Obstacle::makeCircle(Point(3.8, 2.2), test_helpers::kRobotRadius));
  obstacles.push_back(Obstacle::makeCircle(Point(4.5, 2.5), test_helpers::kRobotRadius));
  obstacles.push_back(Obstacle::makeCircle(Point(3.5, -2.0), test_helpers::kRobotRadius));

  const Vector2 u = runFullPipeline({2.0, 0.0}, {5.5, 3.5}, {1.0, 1.0}, obstacles);

  EXPECT_TRUE(std::isfinite(u.x()));
  EXPECT_TRUE(std::isfinite(u.y()));
  EXPECT_GT(u.norm(), 0.0) << "障害物を避けながら前進する速度が得られるべき";
}

// ===== PAコーナー回り込みの詳細テスト =====

TEST(ATEBIntegration, PathQualityAroundPenaltyCorner)
{
  // PAを迂回する経路の品質：不必要に遠回りしていないか
  const std::vector<Obstacle> obstacles = {test_helpers::makeOurPenaltyObstacle()};
  const Point start(3.0, 0.0);
  const Point goal(5.5, 3.5);

  VisibilityGraph vg;
  vg.configure(test_helpers::makeDefaultVGConfig());
  const auto homotopies = vg.extract(start, goal, obstacles);
  ASSERT_FALSE(homotopies.empty());

  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  const auto band = so.optimize(homotopies[0], start, goal, obstacles);
  EXPECT_TRUE(band.isValid());

  // 経路長の上限（直線距離の3倍以上は問題あり）
  const double direct_dist = (goal - start).norm();
  EXPECT_LT(band.total_cost, direct_dist * 3.0)
    << "PAを迂回する経路が直線の3倍以上に長い: " << band.total_cost << " vs " << direct_dist;
}

// ===== ディフェンダーPA縮小回避テスト =====
// Rosbag rosbag2_2026_03_31-18_24_54 で観測された問題を再現:
// ロボット2,3,5がPA下方から上方目標へ直線移動し、PA内を縦断していた

TEST(ATEBIntegration, DefenderCrossPenaltyAreaWithContraction)
{
  // ロボット2のシナリオ再現: (5.0, -3.0) → (5.6, 2.0)
  // 縮小PA(0.5m contraction): offset=0.1-0.5=-0.4 → x=[4.6,5.6], y=[-1.4,1.4]
  // この範囲をVG/SOが迂回する経路を見つけるべき
  const auto contracted_obs = test_helpers::makeOurPenaltyObstacleContracted(0.5);
  const std::vector<Obstacle> obstacles = {contracted_obs};

  const Point start(5.0, -3.0);
  const Point goal(5.6, 2.0);

  VisibilityGraph vg;
  vg.configure(test_helpers::makeDefaultVGConfig());
  const auto homotopies = vg.extract(start, goal, obstacles);
  ASSERT_FALSE(homotopies.empty()) << "縮小PA障害物でも経路が見つかるべき";

  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  ElasticBand best_band;
  for (const auto & h : homotopies) {
    const auto band = so.optimize(h, start, goal, obstacles);
    if (!best_band.isValid() || band.total_cost < best_band.total_cost) {
      best_band = band;
    }
  }
  ASSERT_TRUE(best_band.isValid());

  // 中間ノードが縮小PA内部を通らないこと
  int nodes_inside = 0;
  for (size_t i = 1; i + 1 < best_band.nodes.size(); ++i) {
    if (contracted_obs.distance(best_band.nodes[i].pos) < 0.0) {
      ++nodes_inside;
    }
  }
  EXPECT_EQ(nodes_inside, 0) << nodes_inside << "個の中間ノードが縮小PA内部に残っている";
}

TEST(ATEBIntegration, DefenderCanPassPenaltyEdge)
{
  // 縮小PAでは辺付近の通行が許可される:
  // PA辺(x=4.2)に近い始点から、PA上辺(y=1.8)のすぐ外側の目標へ
  // 縮小PA(offset=-0.4): x=[4.6,5.6] → x=4.3 は縮小PA外なので通行可能
  const auto contracted_obs = test_helpers::makeOurPenaltyObstacleContracted(0.5);
  const std::vector<Obstacle> obstacles = {contracted_obs};

  // PA左辺(x=4.2)の外側から上へ移動: PA辺付近は通行可能
  const Point start(4.3, -2.5);
  const Point goal(4.3, 2.5);

  VisibilityGraph vg;
  vg.configure(test_helpers::makeDefaultVGConfig());
  const auto homotopies = vg.extract(start, goal, obstacles);
  ASSERT_FALSE(homotopies.empty());

  // 直線経路(x=4.3)は縮小PA(x=[4.6,5.6])の外側なので迂回不要
  // → 最短経路が直線か確認
  SpatialOptimizer so;
  so.configure(test_helpers::makeDefaultSOConfig());
  const auto band = so.optimize(homotopies[0], start, goal, obstacles);
  EXPECT_TRUE(band.isValid());

  // ノードが縮小PA外にあること（x=4.3 < 4.6 なのでPA外）
  for (size_t i = 1; i + 1 < band.nodes.size(); ++i) {
    EXPECT_GE(contracted_obs.distance(band.nodes[i].pos), 0.0)
      << "ノード" << i << "が縮小PA内部: x=" << band.nodes[i].pos.x();
  }
}

// ===== コーナー通過判定テスト =====
// Rosbag rosbag2_2026_03_31-21_25_07 で観測された問題:
// ロボット1がPA上コーナー手前0.5m地点で「通過済み」と誤判定され、
// 下コーナーへ目標が切り替わり振動が発生していた。
// 修正: 距離閾値判定 → ドット積による「コーナーをターゲット方向に抜けたか」判定

TEST(ATEBIntegration, CornerPassDetectionBeforeCorner)
{
  // rosbagシナリオ再現:
  //   ロボット1が (4.0, 1.5) からコーナー(4.0, 2.0) 経由で目標(4.72, 2.76) へ移動中
  //   コーナー手前0.5m地点 → 旧距離判定はTRUEと誤判定し、下コーナーへ目標を切り替えていた
  const Point corner(4.0, 2.0);
  const Point target(4.72, 2.76);
  const Point robot_before_corner(4.0, 1.5);  // コーナーから0.5m手前

  // 新: ドット積チェック → コーナーよりターゲット側にいるか
  // (target - corner) · (robot - corner) > 0 → 通過済み
  const bool passed_new = (target - corner).dot(robot_before_corner - corner) > 0;
  EXPECT_FALSE(passed_new) << "コーナー手前ではまだ通過していないはず";

  // 旧コードの挙動を確認（バグ再現）: 距離<=0.5m でTRUEになる
  const bool passed_old = (robot_before_corner - corner).norm() <= 0.5;
  EXPECT_TRUE(passed_old) << "旧距離判定はコーナー手前0.5mでTRUEと誤判定する（これが振動の原因）";
}

TEST(ATEBIntegration, CornerPassDetectionAfterCorner)
{
  // ロボットがコーナーを通過した直後 → ドット積チェックでTRUEになる
  const Point corner(4.0, 2.0);
  const Point target(4.72, 2.76);
  const Point robot_after_corner(4.1, 2.1);  // コーナーを通過した直後

  const bool passed_new = (target - corner).dot(robot_after_corner - corner) > 0;
  EXPECT_TRUE(passed_new) << "コーナー通過後はドット積チェックでTRUEになるはず";
}

TEST(ATEBIntegration, CornerPassDetectionBelowLowerCorner)
{
  // ロボットが下コーナー(4.0, -2.0)の下側にいる場合:
  // 上コーナー(4.0, 2.0)はまだ通過していない
  // 下コーナー(4.0, -2.0)もまだ通過していない（ロボットはさらに下にいる）
  const Point corner_upper(4.0, 2.0);
  const Point corner_lower(4.0, -2.0);
  const Point target(4.72, 2.76);

  // ロボットが下コーナーの下側: (4.0, -3.0)
  const Point robot_below(4.0, -3.0);

  // 上コーナーはまだ通過していない: (target - c1)=(0.72, 0.76), (robot - c1)=(0, -5.0) → 負
  const bool c1_passed = (target - corner_upper).dot(robot_below - corner_upper) > 0;
  EXPECT_FALSE(c1_passed) << "下側ロボットは上コーナーを通過していないはず";

  // 下コーナーもまだ通過していない: (target - c2)=(0.72, 4.76), (robot - c2)=(0, -1.0) → 負
  const bool c2_passed = (target - corner_lower).dot(robot_below - corner_lower) > 0;
  EXPECT_FALSE(c2_passed) << "下コーナーの下側ロボットは下コーナーも通過していないはず";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
