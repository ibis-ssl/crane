// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__ATEB_TIME_PARAMETERIZER_HPP_
#define CRANE_LOCAL_PLANNER__ATEB_TIME_PARAMETERIZER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <utility>
#include <vector>

#include "ateb_types.hpp"

namespace crane::ateb
{

/// ER-Force式時間最適速度プロファイル生成器
///
/// ElasticBandの空間パスに対して、bang-bang制御原理（常にa = a_max）に
/// 基づく時間最適な速度プロファイルを付与する。
///
/// アルゴリズム:
/// 1. 空間パスを弧長で等間隔にリサンプル
/// 2. 前方パス（加速制限）: 初期速度から各点の最大到達速度を計算
/// 3. 後方パス（減速制限）: 終端速度から各点の最大接近速度を計算
/// 4. 統合: v[i] = min(forward[i], backward[i])
/// 5. 曲率による加速度制限: 曲がり角では求心加速度分を差し引く
class TimeParameterizer
{
public:
  struct Config
  {
    double max_velocity = 5.0;      ///< 最大速度 [m/s]
    double max_acceleration = 5.0;  ///< 最大加速度 [m/s^2]
    double max_deceleration = 2.5;  ///< 最大減速度 [m/s^2]
    int sample_count = 50;          ///< 弧長サンプル数
    double min_ds = 0.001;          ///< 最小サンプル間隔 [m]
  };

  void configure(const Config & config) { config_ = config; }

  /// 空間パスを時間最適軌道にパラメータ化する
  ///
  /// @param band 最適化済み弾性バンド
  /// @param initial_velocity 初期速度ベクトル
  /// @param terminal_velocity 終端速度ベクトル
  /// @param max_vel 最大速度（名目値のオーバーライド用）
  /// @param max_acc 最大加速度（名目値のオーバーライド用）
  /// @param max_brk 最大減速度（名目値のオーバーライド用）
  /// @return 時間最適軌道
  [[nodiscard]] TimeOptimalTrajectory parameterize(
    const ElasticBand & band, const Vector2 & initial_velocity, const Vector2 & terminal_velocity,
    double max_vel, double max_acc, double max_brk) const;

private:
  Config config_;

  /// 弾性バンドを弧長でリサンプルした点列を返す
  /// 戻り値: (弧長, 位置) のペアのベクトル
  [[nodiscard]] std::vector<std::pair<double, Point>> computeArcLengthTable(
    const ElasticBand & band) const;

  /// 曲率を近似計算する（隣接3点から）
  [[nodiscard]] double computeCurvature(const Point & p0, const Point & p1, const Point & p2) const;

  /// 前方パス: 初速度から各サンプル点の最大速度を計算
  [[nodiscard]] std::vector<double> forwardPass(
    const std::vector<std::pair<double, Point>> & arc_table, double v0, double v_max,
    double a_max) const;

  /// 後方パス: 終端速度から各サンプル点の最大速度を計算
  [[nodiscard]] std::vector<double> backwardPass(
    const std::vector<std::pair<double, Point>> & arc_table, double vf, double v_max,
    double a_brk) const;

  /// 速度プロファイルをTimeOptimalTrajectoryに変換
  [[nodiscard]] TimeOptimalTrajectory buildTrajectory(
    const std::vector<std::pair<double, Point>> & arc_table,
    const std::vector<double> & speeds) const;
};

}  // namespace crane::ateb

#endif  // CRANE_LOCAL_PLANNER__ATEB_TIME_PARAMETERIZER_HPP_
