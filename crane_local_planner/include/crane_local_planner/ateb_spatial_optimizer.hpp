// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__ATEB_SPATIAL_OPTIMIZER_HPP_
#define CRANE_LOCAL_PLANNER__ATEB_SPATIAL_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <crane_geometry/boost_geometry.hpp>
#include <vector>

#include "ateb_types.hpp"

namespace crane::ateb
{

/// 解析的TEB (A-TEB) 空間最適化器
///
/// ホモトピークラス経路を初期値として、Gauss-Newton法 / LMA法で
/// 弾性バンドを局所最適化する。
///
/// グリッドマップを一切使用せず、幾何プリミティブへの解析的距離関数と
/// そのヤコビアンを直接用いることで、高速・高精度な収束を実現する。
///
/// 最小化する目的関数（残差形式）:
///   f_smooth: w_s * (node[i-1] + node[i+1] - 2*node[i])  (滑らかさ)
///   f_obs:    w_o * max(0, d_min - dist(node, obs))       (障害物回避)
///   f_len:    w_l * (node[i+1] - node[i])                 (経路長)
class SpatialOptimizer
{
public:
  struct Config
  {
    int max_iterations = 5;                ///< 最大Gauss-Newton反復回数
    double smoothness_weight = 1.0;        ///< 滑らかさ重み w_s
    double obstacle_weight = 10.0;         ///< 障害物回避重み w_o
    double path_length_weight = 0.3;       ///< 経路長重み w_l
    double obstacle_safety_margin = 0.02;  ///< 障害物からの追加マージン [m]
    double convergence_threshold = 1e-4;   ///< 収束判定 [m]
    double lma_damping = 1e-3;             ///< LMA ダンピング係数
    int band_node_count = 15;              ///< バンドのノード数
  };

  void configure(const Config & config) { config_ = config; }

  /// ホモトピークラス経路をリサンプルして弾性バンドを初期化し、最適化する
  ///
  /// @param homotopy 初期ホモトピークラス
  /// @param start 始点（固定）
  /// @param goal 終点（固定）
  /// @param obstacles 障害物リスト
  /// @return 最適化後の弾性バンド
  [[nodiscard]] ElasticBand optimize(
    const HomotopyClass & homotopy, const Point & start, const Point & goal,
    const std::vector<Obstacle> & obstacles) const;

  /// 既存のバンドをウォームスタートとして最適化（障害物変化時）
  [[nodiscard]] ElasticBand reoptimize(
    const ElasticBand & initial_band, const std::vector<Obstacle> & obstacles) const;

private:
  Config config_;

  /// ホモトピー経路をband_node_count個の等間隔ノードにリサンプル
  [[nodiscard]] ElasticBand initializeBand(
    const HomotopyClass & homotopy, const Point & start, const Point & goal) const;

  /// Gauss-Newton/LMAの1ステップ: バンドのノード位置を更新する
  /// @return 最大ノード移動量
  [[nodiscard]] double optimizeStep(
    ElasticBand & band, const std::vector<Obstacle> & obstacles) const;

  /// 残差ベクトルとヤコビアン行列を構築
  void buildResidualAndJacobian(
    const ElasticBand & band, const std::vector<Obstacle> & obstacles, Eigen::VectorXd & residuals,
    Eigen::MatrixXd & jacobian) const;
};

}  // namespace crane::ateb

#endif  // CRANE_LOCAL_PLANNER__ATEB_SPATIAL_OPTIMIZER_HPP_
