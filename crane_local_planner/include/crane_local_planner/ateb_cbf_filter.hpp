// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__ATEB_CBF_FILTER_HPP_
#define CRANE_LOCAL_PLANNER__ATEB_CBF_FILTER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <utility>
#include <vector>

#include "ateb_types.hpp"

namespace crane::ateb
{

/// CBF (Control Barrier Function) ベースの安全速度フィルタ
///
/// 名目速度 u_nom に対して、ロボット間の衝突を防ぐための
/// 最小限の速度修正を行う。
///
/// 最適化問題: min ||u - u_nom||^2
///   s.t. 2*(p_ego - p_j)^T * u >= -alpha*h_j + 2*(p_ego - p_j)^T * v_j
///        (すべての障害物 j に対して)
///   where h_j = ||p_ego - p_j||^2 - (r_j + margin)^2
///
/// 2変数QPを半平面制約の射影法で解く（外部ライブラリ不要）
class CBFFilter
{
public:
  struct Config
  {
    double alpha = 1.0;            ///< CBFクラスK関数パラメータ
    double robot_radius = 0.060;   ///< ロボット半径 [m]
    double safety_margin = 0.030;  ///< 追加安全マージン [m]
    double max_correction = 3.0;   ///< 最大速度修正量 [m/s]
    int max_iterations = 10;       ///< 射影法最大反復回数
  };

  void configure(const Config & config) { config_ = config; }

  /// 速度命令を安全制約に従ってフィルタリングする
  ///
  /// @param ego_pos 自機位置
  /// @param u_nominal 名目速度ベクトル
  /// @param dynamic_obstacles 動的障害物 (位置, 速度) のペアリスト
  /// @param static_obstacles 静的障害物リスト
  /// @return フィルタ後の速度ベクトル
  [[nodiscard]] Vector2 filter(
    const Point & ego_pos, const Vector2 & u_nominal,
    const std::vector<std::pair<Point, Vector2>> & dynamic_obstacles,
    const std::vector<Obstacle> & static_obstacles) const;

private:
  Config config_;

  struct HalfPlaneConstraint
  {
    Vector2 normal;  ///< a (単位ベクトルでなくてもよい)
    double bound;    ///< b: a^T * u >= b
  };

  /// 全制約を構築する
  [[nodiscard]] std::vector<HalfPlaneConstraint> buildConstraints(
    const Point & ego_pos, const std::vector<std::pair<Point, Vector2>> & dynamic_obstacles,
    const std::vector<Obstacle> & static_obstacles) const;

  /// 半平面制約のQPを射影法で解く
  /// min ||u - u_nom||^2  s.t.  a_i^T * u >= b_i  for all i
  [[nodiscard]] Vector2 solveProjectionQP(
    const Vector2 & u_nom, const std::vector<HalfPlaneConstraint> & constraints) const;
};

}  // namespace crane::ateb

#endif  // CRANE_LOCAL_PLANNER__ATEB_CBF_FILTER_HPP_
