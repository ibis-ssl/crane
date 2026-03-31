// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <vector>

#include "crane_local_planner/ateb_cbf_filter.hpp"
#include "crane_local_planner/ateb_spatial_optimizer.hpp"
#include "crane_local_planner/ateb_types.hpp"
#include "crane_local_planner/ateb_visibility_graph.hpp"

namespace test_helpers
{

// Division A フィールド寸法
constexpr double kFieldHalfX = 6.0;
constexpr double kFieldHalfY = 4.5;
constexpr double kPenaltyDepth = 1.8;      // x方向の深さ
constexpr double kPenaltyHalfWidth = 1.8;  // y方向の半幅
constexpr double kRobotRadius = 0.09;
constexpr double kInflationRadius = 0.09;
constexpr double kPenaltyOffsetInplay = 0.1;
constexpr double kPenaltyOffsetStop = 0.3;
constexpr double kFar = 20.0;

// 自陣ペナルティエリア: x=[4.2, 6.0], y=[-1.8, 1.8]
inline crane::Box makeOurPenaltyBox()
{
  return crane::Box(
    crane::Point(kFieldHalfX - kPenaltyDepth, -kPenaltyHalfWidth),
    crane::Point(kFieldHalfX, kPenaltyHalfWidth));
}

// 敵陣ペナルティエリア: x=[-6.0, -4.2], y=[-1.8, 1.8]
inline crane::Box makeTheirPenaltyBox()
{
  return crane::Box(
    crane::Point(-kFieldHalfX, -kPenaltyHalfWidth),
    crane::Point(-kFieldHalfX + kPenaltyDepth, kPenaltyHalfWidth));
}

// skip_inflation=trueで膨張済みペナルティBOX障害物を作成
inline crane::ateb::Obstacle makeInflatedPenaltyObstacle(const crane::Box & base, double offset)
{
  auto obs = crane::ateb::Obstacle::makeBox(base);
  obs = obs.inflated(offset);
  obs.skip_inflation = true;
  return obs;
}

// 自陣ペナルティエリア障害物（膨張済み）
inline crane::ateb::Obstacle makeOurPenaltyObstacle(double offset = kPenaltyOffsetInplay)
{
  return makeInflatedPenaltyObstacle(makeOurPenaltyBox(), offset);
}

// 敵陣ペナルティエリア障害物（膨張済み）
inline crane::ateb::Obstacle makeTheirPenaltyObstacle(double offset = kPenaltyOffsetInplay)
{
  return makeInflatedPenaltyObstacle(makeTheirPenaltyBox(), offset);
}

// フィールド境界壁（4面）
inline std::vector<crane::ateb::Obstacle> makeFieldWalls()
{
  std::vector<crane::ateb::Obstacle> walls;
  // 上壁
  walls.push_back(
    crane::ateb::Obstacle::makeBox(
      crane::Box(crane::Point(-kFar, kFieldHalfY), crane::Point(kFar, kFar))));
  // 下壁
  walls.push_back(
    crane::ateb::Obstacle::makeBox(
      crane::Box(crane::Point(-kFar, -kFar), crane::Point(kFar, -kFieldHalfY))));
  // 右壁
  walls.push_back(
    crane::ateb::Obstacle::makeBox(
      crane::Box(crane::Point(kFieldHalfX, -kFar), crane::Point(kFar, kFar))));
  // 左壁
  walls.push_back(
    crane::ateb::Obstacle::makeBox(
      crane::Box(crane::Point(-kFar, -kFar), crane::Point(-kFieldHalfX, kFar))));
  return walls;
}

// PA縮小障害物（ディフェンダー用）
inline crane::ateb::Obstacle makeOurPenaltyObstacleContracted(
  double contraction = 0.5, double offset = kPenaltyOffsetInplay)
{
  return makeInflatedPenaltyObstacle(makeOurPenaltyBox(), offset - contraction);
}

// デフォルト設定ファクトリ
inline crane::ateb::VisibilityGraph::Config makeDefaultVGConfig()
{
  crane::ateb::VisibilityGraph::Config cfg;
  cfg.inflation_radius = kInflationRadius;
  cfg.max_homotopy_classes = 3;
  return cfg;
}

inline crane::ateb::SpatialOptimizer::Config makeDefaultSOConfig()
{
  crane::ateb::SpatialOptimizer::Config cfg;
  cfg.max_iterations = 5;
  cfg.smoothness_weight = 1.0;
  cfg.obstacle_weight = 10.0;
  cfg.path_length_weight = 0.3;
  cfg.obstacle_safety_margin = 0.02;
  cfg.band_node_count = 15;
  return cfg;
}

inline crane::ateb::CBFFilter::Config makeDefaultCBFConfig()
{
  crane::ateb::CBFFilter::Config cfg;
  cfg.alpha = 1.0;
  cfg.robot_radius = kRobotRadius;
  cfg.safety_margin = 0.03;
  cfg.max_correction = 3.0;
  cfg.max_iterations = 10;
  return cfg;
}

// バンド内の全ノードが障害物から安全な距離を保っているかチェック
inline bool bandAvoidsObstacle(
  const crane::ateb::ElasticBand & band, const crane::ateb::Obstacle & obs, double margin = 0.0)
{
  for (const auto & node : band.nodes) {
    if (obs.distance(node.pos) < margin) {
      return false;
    }
  }
  return true;
}

// 経路の全ウェイポイントがBOX障害物の外側にあるかチェック
inline bool waypointsOutsideObstacle(
  const crane::ateb::HomotopyClass & homotopy, const crane::ateb::Obstacle & obs)
{
  for (const auto & wp : homotopy.waypoints) {
    if (obs.distance(wp) < 0.0) {
      return false;
    }
  }
  return true;
}

}  // namespace test_helpers

#endif  // TEST_HELPERS_HPP_
