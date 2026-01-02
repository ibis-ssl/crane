// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__BANG_BANG_TRAJECTORY_HPP_
#define CRANE_PHYSICS__BANG_BANG_TRAJECTORY_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace crane
{

/**
 * @brief 軌道セグメントを表す構造体
 *
 * Bang-Bang軌道は最大3つのセグメントで構成される:
 * - 三角形プロファイル: 加速 → 減速 (2セグメント)
 * - 台形プロファイル: 加速 → 定速 → 減速 (3セグメント)
 */
struct BBTrajectoryPart
{
  double end_time = 0.0;
  double acceleration = 0.0;
  double start_vel = 0.0;
  double start_pos = 0.0;
};

/**
 * @brief 1次元の時間最適軌道（Bang-Bang制御）を生成するクラス
 */
class BangBangTrajectory1D
{
public:
  BangBangTrajectory1D() noexcept
  {
    for (int i = 0; i < MAX_PARTS; i++) {
      parts[i] = BBTrajectoryPart();
    }
  }

  [[nodiscard]] double getPosition(const double t) const noexcept
  {
    const auto ctx = getActivePartContext(t);
    return ctx.part.start_pos + (ctx.part.start_vel * ctx.dt) +
           (0.5 * ctx.part.acceleration * ctx.dt * ctx.dt);
  }

  [[nodiscard]] double getVelocity(const double t) const noexcept
  {
    if (std::max(0.0, t) >= getTotalTime()) {
      return 0.0;
    }
    const auto ctx = getActivePartContext(t);
    return ctx.part.start_vel + (ctx.part.acceleration * ctx.dt);
  }

  [[nodiscard]] double getAcceleration(const double t) const noexcept
  {
    if (std::max(0.0, t) >= getTotalTime()) {
      return 0.0;
    }
    return getActivePartContext(t).part.acceleration;
  }

  [[nodiscard]] double getTotalTime() const noexcept
  {
    if (num_parts == 0) return 0.0;
    return parts[num_parts - 1].end_time;
  }

  /**
   * @brief 軌道を生成する
   * @param initial_pos 初期位置
   * @param target_pos 目標位置
   * @param initial_vel 初期速度
   * @param max_vel 最大速度
   * @param max_acc 最大加速度
   */
  BangBangTrajectory1D & generate(
    const double initial_pos, const double target_pos, const double initial_vel, const double max_vel,
    const double max_acc)
  {
    // フルブレーキ時に停止する位置を計算（プロファイル選択の基準）
    const double braking_pos = computeBrakingPosition(initial_pos, initial_vel, max_acc);

    if (braking_pos <= target_pos) {
      // 目標が停止位置より前方 → 前進プロファイル
      const double triangular_end_pos =
        computeTriangularEndPosition(initial_pos, initial_vel, max_vel, max_acc);

      if (triangular_end_pos >= target_pos) {
        // 三角形プロファイル（最大速度に達しない短距離移動）
        calculateTriangularProfile(initial_pos, initial_vel, target_pos, max_acc);
      } else {
        // 台形プロファイル（最大速度に達する長距離移動）
        calculateTrapezoidalProfile(initial_pos, initial_vel, max_vel, target_pos, max_acc);
      }
    } else {
      // 目標が停止位置より後方 → 逆方向への移動が必要
      const double triangular_end_pos =
        computeTriangularEndPosition(initial_pos, initial_vel, -max_vel, max_acc);

      if (triangular_end_pos <= target_pos) {
        // 三角形プロファイル（逆方向）
        calculateTriangularProfile(initial_pos, initial_vel, target_pos, -max_acc);
      } else {
        // 台形プロファイル（逆方向）
        calculateTrapezoidalProfile(initial_pos, initial_vel, -max_vel, target_pos, max_acc);
      }
    }
    return *this;
  }

private:
  static constexpr int MAX_PARTS = 3;
  std::array<BBTrajectoryPart, MAX_PARTS> parts;
  int num_parts = 0;

  struct PartContext
  {
    const BBTrajectoryPart & part;
    double dt;
  };

  [[nodiscard]] int findPartIndex(const double t) const noexcept
  {
    if (num_parts == 0) return 0;
    for (int i = 0; i < num_parts; i++) {
      if (t < parts[i].end_time) {
        return i;
      }
    }
    return num_parts - 1;
  }

  [[nodiscard]] PartContext getActivePartContext(const double t) const noexcept
  {
    if (num_parts == 0) {
      return {parts[0], 0.0};
    }

    const double traj_time = std::max(0.0, std::min(t, getTotalTime()));
    const int idx = findPartIndex(traj_time);
    const double start_time = (idx == 0) ? 0.0 : parts[idx - 1].end_time;
    return {parts[idx], traj_time - start_time};
  }

  [[nodiscard]] const BBTrajectoryPart & findPart(const double t) const noexcept
  {
    return parts[findPartIndex(t)];
  }

  /**
   * @brief フルブレーキ時の停止位置を計算
   * @param initial_pos 初期位置
   * @param initial_vel 初期速度
   * @param max_acc 最大加速度（正の値）
   * @return 速度が0になった時点の位置
   */
  [[nodiscard]] double computeBrakingPosition(
    const double initial_pos, const double initial_vel, const double max_acc) const noexcept
  {
    // 速度の符号と逆方向に最大加速度をかける
    const double acceleration = (initial_vel <= 0.0) ? max_acc : -max_acc;
    const double time_to_zero = -initial_vel / acceleration;
    return initial_pos + (0.5 * initial_vel * time_to_zero);
  }

  /**
   * @brief 三角形プロファイルで移動した場合の終端位置を計算
   *
   * 初期速度からピーク速度まで加速し、その後0まで減速した場合の位置。
   * generate()でどのプロファイルを選択するかの判定に使用。
   *
   * @param initial_pos 初期位置
   * @param initial_vel 初期速度
   * @param peak_vel 目標ピーク速度
   * @param max_acc 最大加速度（正の値）
   * @return 三角形プロファイル終了時の位置
   */
  [[nodiscard]] double computeTriangularEndPosition(
    const double initial_pos, const double initial_vel, const double peak_vel,
    const double max_acc) const noexcept
  {
    // 加速/減速の方向を決定
    const double acc1 = (peak_vel >= initial_vel) ? max_acc : -max_acc;
    const double acc2 = -acc1;

    // 加速区間
    const double accel_time = (peak_vel - initial_vel) / acc1;
    const double accel_end_pos = initial_pos + (0.5 * (initial_vel + peak_vel) * accel_time);

    // 減速区間
    const double decel_time = -peak_vel / acc2;
    return accel_end_pos + (0.5 * peak_vel * decel_time);
  }

  /**
   * @brief 三角形プロファイル（加速→減速）を計算
   *
   * 数式導出:
   * - 加速区間: v(t1) = v0 + a*t1 = v_peak, x1 = x0 + (v0+v_peak)/2 * t1
   * - 減速区間: v(t2) = v_peak - a*t2 = 0, x_final = x1 + v_peak/2 * t2
   * - t2 = v_peak/a, v_peak = a*t2 を代入して整理すると:
   *   t2 = sqrt((a*(x_final-x0) + 0.5*v0^2) / a^2)
   */
  void calculateTriangularProfile(
    const double initial_pos, const double initial_vel, const double final_pos,
    const double acceleration)
  {
    // 減速時間t2を求める式: t2^2 = (a*distance + 0.5*v0^2) / a^2
    double sqrt_val;
    if (acceleration > 0) {
      // 正方向移動: 加速(+a) → 減速(-a)
      sqrt_val =
        ((acceleration * (final_pos - initial_pos)) + (0.5 * initial_vel * initial_vel)) /
        (acceleration * acceleration);
    } else {
      // 負方向移動: 減速(-a) → 加速(+a)
      sqrt_val =
        ((-acceleration * (initial_pos - final_pos)) + (0.5 * initial_vel * initial_vel)) /
        (acceleration * acceleration);
    }

    const double decel_time = (sqrt_val > 0.0) ? std::sqrt(sqrt_val) : 0.0;
    const double peak_velocity = acceleration * decel_time;
    const double accel_time = (peak_velocity - initial_vel) / acceleration;
    const double accel_end_pos = initial_pos + ((initial_vel + peak_velocity) * 0.5 * accel_time);

    parts[0].end_time = accel_time;
    parts[0].acceleration = acceleration;
    parts[0].start_vel = initial_vel;
    parts[0].start_pos = initial_pos;
    parts[1].end_time = accel_time + decel_time;
    parts[1].acceleration = -acceleration;
    parts[1].start_vel = peak_velocity;
    parts[1].start_pos = accel_end_pos;
    num_parts = 2;
  }

  /**
   * @brief 台形プロファイル（加速→定速→減速）を計算
   *
   * 最大速度に達する長距離移動用。
   * 3つの区間: 加速(acc1) → 定速(0) → 減速(acc3)
   */
  void calculateTrapezoidalProfile(
    const double initial_pos, const double initial_vel, const double cruise_vel,
    const double final_pos, const double max_acc)
  {
    // 加速方向: 初速度から巡航速度に向かう方向
    const double acc1 = (initial_vel > cruise_vel) ? -max_acc : max_acc;
    // 減速方向: 巡航速度から停止に向かう方向
    const double acc3 = (cruise_vel > 0) ? -max_acc : max_acc;

    // 各区間の時間を計算
    const double accel_time = (cruise_vel - initial_vel) / acc1;
    const double decel_time = -cruise_vel / acc3;

    // 各区間の終端位置を計算
    const double accel_end_pos = initial_pos + (0.5 * (initial_vel + cruise_vel) * accel_time);
    const double cruise_end_pos = final_pos - (0.5 * cruise_vel * decel_time);
    const double cruise_time = (cruise_end_pos - accel_end_pos) / cruise_vel;

    // Part 0: 加速区間
    parts[0].end_time = accel_time;
    parts[0].acceleration = acc1;
    parts[0].start_vel = initial_vel;
    parts[0].start_pos = initial_pos;

    // Part 1: 定速区間
    parts[1].end_time = accel_time + cruise_time;
    parts[1].acceleration = 0;
    parts[1].start_vel = cruise_vel;
    parts[1].start_pos = accel_end_pos;

    // Part 2: 減速区間
    parts[2].end_time = accel_time + cruise_time + decel_time;
    parts[2].acceleration = acc3;
    parts[2].start_vel = cruise_vel;
    parts[2].start_pos = cruise_end_pos;

    num_parts = 3;
  }
};

/**
 * @brief 2次元の同期された時間最適軌道を生成するクラス
 * X軸とY軸の動作時間を同期させ、直線的な移動を実現
 *
 * 同期メカニズム:
 * alpha角（加速度配分角）を二分探索で調整し、X/Y軸の到達時間を一致させる。
 * - cos(alpha) * acc = X軸への加速度配分
 * - sin(alpha) * acc = Y軸への加速度配分
 * - alpha = π/4 で等分配、X軸移動が長いとalphaを減少させてX軸に多く配分
 */
class BangBangTrajectory2D
{
public:
  BangBangTrajectory1D x;
  BangBangTrajectory1D y;

  /// 二分探索の初期刻み幅
  static constexpr double BINARY_SEARCH_INITIAL_INCREMENT = M_PI / 8.0;
  /// 二分探索の初期alpha角（等分配）
  static constexpr double BINARY_SEARCH_INITIAL_ALPHA = M_PI / 4.0;
  /// 二分探索の収束判定閾値
  static constexpr double BINARY_SEARCH_EPSILON = 1e-7;
  /// デフォルトの同期精度 [秒]
  static constexpr double DEFAULT_SYNC_ACCURACY = 0.001;

  BangBangTrajectory2D() = default;

  [[nodiscard]] Eigen::Vector2d getPosition(const double t) const noexcept
  {
    return Eigen::Vector2d(x.getPosition(t), y.getPosition(t));
  }

  [[nodiscard]] Eigen::Vector2d getVelocity(const double t) const noexcept
  {
    return Eigen::Vector2d(x.getVelocity(t), y.getVelocity(t));
  }

  [[nodiscard]] Eigen::Vector2d getAcceleration(const double t) const noexcept
  {
    return Eigen::Vector2d(x.getAcceleration(t), y.getAcceleration(t));
  }

  [[nodiscard]] double getTotalTime() const noexcept
  {
    return std::max(x.getTotalTime(), y.getTotalTime());
  }

  /**
   * @brief 同期された2次元軌道を生成する
   * @param s0 初期位置
   * @param s1 目標位置
   * @param v0 初期速度
   * @param vmax 最大速度
   * @param acc 最大加速度
   * @param accuracy 同期精度
   */
  BangBangTrajectory2D & generate(
    const Eigen::Vector2d & s0, const Eigen::Vector2d & s1, const Eigen::Vector2d & v0,
    const double vmax, const double acc, const double accuracy = DEFAULT_SYNC_ACCURACY)
  {
    const double s0x = s0.x();
    const double s0y = s0.y();
    const double s1x = s1.x();
    const double s1y = s1.y();
    const double v0x = v0.x();
    const double v0y = v0.y();

    double inc = BINARY_SEARCH_INITIAL_INCREMENT;
    double alpha = BINARY_SEARCH_INITIAL_ALPHA;

    // 二分探索でX/Y軸の到達時間が一致するalpha角を探索
    while (inc > BINARY_SEARCH_EPSILON) {
      const double sin_alpha = std::sin(alpha);
      const double cos_alpha = std::cos(alpha);

      x.generate(s0x, s1x, v0x, vmax * cos_alpha, acc * cos_alpha);
      y.generate(s0y, s1y, v0y, vmax * sin_alpha, acc * sin_alpha);

      const double time_diff = std::abs(x.getTotalTime() - y.getTotalTime());
      if (time_diff < accuracy) {
        break;
      }

      // X軸の方が時間がかかる場合、X軸への配分を増やす（alpha減少）
      if (x.getTotalTime() > y.getTotalTime()) {
        alpha -= inc;
      } else {
        alpha += inc;
      }

      inc *= 0.5;
    }
    return *this;
  }
};

}  // namespace crane

#endif  // CRANE_PHYSICS__BANG_BANG_TRAJECTORY_HPP_
