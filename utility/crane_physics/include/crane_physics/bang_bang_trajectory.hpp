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
 *
 * 加速フェーズと減速フェーズで異なる加速度（max_acc / max_brk）を使用できる。
 * これによりロボットの加速能力と制動能力を個別に反映した軌道計画が可能。
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
   * @param max_acc 加速フェーズの最大加速度
   * @param max_brk 減速フェーズの最大加速度（0以下のときmax_accと同じ値を使用）
   */
  BangBangTrajectory1D & generate(
    const double initial_pos, const double target_pos, const double initial_vel,
    const double max_vel, const double max_acc, const double max_brk = 0.0)
  {
    // max_accが実用上ゼロ（2D二分探索でalphaが端点に近い場合）のときは
    // 静止軌道として扱いゼロ除算を回避する
    if (max_acc < 1e-9) {
      parts[0].end_time = 0.0;
      parts[0].acceleration = 0.0;
      parts[0].start_vel = 0.0;
      parts[0].start_pos = initial_pos;
      num_parts = 1;
      return *this;
    }

    // max_brk <= 0 のときはmax_accと同じ値（後方互換性）
    const double brk = (max_brk > 0.0) ? max_brk : max_acc;

    // フルブレーキ時に停止する位置を計算（プロファイル選択の基準）
    const double braking_pos = computeBrakingPosition(initial_pos, initial_vel, brk);

    if (braking_pos <= target_pos) {
      // 目標が停止位置より前方 → 前進プロファイル
      const double triangular_end_pos =
        computeTriangularEndPosition(initial_pos, initial_vel, max_vel, max_acc, brk);

      if (triangular_end_pos >= target_pos) {
        // 三角形プロファイル（最大速度に達しない短距離移動）
        calculateTriangularProfile(initial_pos, initial_vel, target_pos, max_acc, brk);
      } else {
        // 台形プロファイル（最大速度に達する長距離移動）
        calculateTrapezoidalProfile(initial_pos, initial_vel, max_vel, target_pos, max_acc, brk);
      }
    } else {
      // 目標が停止位置より後方 → 逆方向への移動が必要
      const double triangular_end_pos =
        computeTriangularEndPosition(initial_pos, initial_vel, -max_vel, max_acc, brk);

      if (triangular_end_pos <= target_pos) {
        // 三角形プロファイル（逆方向）
        calculateTriangularProfile(initial_pos, initial_vel, target_pos, -max_acc, brk);
      } else {
        // 台形プロファイル（逆方向）
        calculateTrapezoidalProfile(initial_pos, initial_vel, -max_vel, target_pos, max_acc, brk);
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
   * @param max_brk 制動加速度（正の値）
   * @return 速度が0になった時点の位置
   */
  [[nodiscard]] double computeBrakingPosition(
    const double initial_pos, const double initial_vel, const double max_brk) const noexcept
  {
    const double acceleration = (initial_vel <= 0.0) ? max_brk : -max_brk;
    const double time_to_zero = -initial_vel / acceleration;
    return initial_pos + (0.5 * initial_vel * time_to_zero);
  }

  /**
   * @brief 三角形プロファイルで移動した場合の終端位置を計算
   *
   * 初期速度からピーク速度まで加速し、その後0まで減速した場合の位置。
   * generate()でどのプロファイルを選択するかの判定に使用。
   * 減速フェーズにはmax_brkを使用する。
   *
   * @param initial_pos 初期位置
   * @param initial_vel 初期速度
   * @param peak_vel 目標ピーク速度
   * @param max_acc 加速度（正の値）
   * @param max_brk 制動加速度（正の値）
   * @return 三角形プロファイル終了時の位置
   */
  [[nodiscard]] double computeTriangularEndPosition(
    const double initial_pos, const double initial_vel, const double peak_vel, const double max_acc,
    const double max_brk) const noexcept
  {
    const double acc1 = (peak_vel >= initial_vel) ? max_acc : -max_acc;

    // 加速区間
    const double accel_time = (peak_vel - initial_vel) / acc1;
    const double accel_end_pos = initial_pos + (0.5 * (initial_vel + peak_vel) * accel_time);

    // 減速区間（max_brkを使用）
    const double decel_time = std::abs(peak_vel) / max_brk;
    return accel_end_pos + (0.5 * peak_vel * decel_time);
  }

  /**
   * @brief 三角形プロファイル（加速→減速）を計算
   *
   * 加速フェーズでmax_acc、減速フェーズでmax_brkを使用する。
   *
   * 数式導出（正方向の場合）:
   * - 加速区間: v0 → v_peak (加速度 a), t1 = (v_peak - v0) / a
   * - 減速区間: v_peak → 0 (制動加速度 b), t2 = v_peak / b
   * - 距離 = (v_peak² - v0²) / (2a) + v_peak² / (2b)
   * - v_peak = sqrt(2ab / (a+b) * (distance + v0² / (2a)))
   */
  void calculateTriangularProfile(
    const double initial_pos, const double initial_vel, const double final_pos,
    const double acceleration,  // +max_acc or -max_acc（符号で方向を表す）
    const double max_brk)       // 制動加速度（正の値）
  {
    const double max_acc_mag = std::abs(acceleration);
    const double direction_sign = (acceleration > 0.0) ? 1.0 : -1.0;
    // distance は常に非負
    const double distance = direction_sign * (final_pos - initial_pos);

    // 非対称BangBang: v_peak² = 2ab/(a+b) * (distance + v0²/(2a))
    const double v_peak_sq_numerator =
      2.0 * max_acc_mag * max_brk * (distance + initial_vel * initial_vel / (2.0 * max_acc_mag));
    const double v_peak_sq = v_peak_sq_numerator / (max_acc_mag + max_brk);
    const double v_peak_mag = (v_peak_sq > 0.0) ? std::sqrt(v_peak_sq) : 0.0;
    const double peak_velocity = direction_sign * v_peak_mag;

    const double decel_time = v_peak_mag / max_brk;
    const double accel_time = (peak_velocity - initial_vel) / acceleration;
    const double accel_end_pos = initial_pos + ((initial_vel + peak_velocity) * 0.5 * accel_time);

    parts[0].end_time = accel_time;
    parts[0].acceleration = acceleration;  // +max_acc or -max_acc
    parts[0].start_vel = initial_vel;
    parts[0].start_pos = initial_pos;
    parts[1].end_time = accel_time + decel_time;
    parts[1].acceleration = -direction_sign * max_brk;  // -max_brk or +max_brk
    parts[1].start_vel = peak_velocity;
    parts[1].start_pos = accel_end_pos;
    num_parts = 2;
  }

  /**
   * @brief 台形プロファイル（加速→定速→減速）を計算
   *
   * 最大速度に達する長距離移動用。
   * 加速フェーズでmax_acc、減速フェーズでmax_brkを使用する。
   * 3つの区間: 加速(acc1) → 定速(0) → 減速(acc3)
   */
  void calculateTrapezoidalProfile(
    const double initial_pos, const double initial_vel, const double cruise_vel,
    const double final_pos, const double max_acc, const double max_brk)
  {
    // 加速方向: 初速度から巡航速度に向かう方向
    const double acc1 = (initial_vel > cruise_vel) ? -max_acc : max_acc;
    // 減速方向: 巡航速度から停止に向かう方向（max_brkを使用）
    const double acc3 = (cruise_vel > 0) ? -max_brk : max_brk;

    // 各区間の時間を計算
    const double accel_time = (cruise_vel - initial_vel) / acc1;
    const double decel_time = -cruise_vel / acc3;  // = |cruise_vel| / max_brk

    // 各区間の終端位置を計算
    const double accel_end_pos = initial_pos + (0.5 * (initial_vel + cruise_vel) * accel_time);
    // 減速開始位置: 目標から減速距離分手前（max_brkで計算）
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

    // Part 2: 減速区間（max_brkを使用）
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
  double max_vel = 0.0;

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
    Eigen::Vector2d vel(x.getVelocity(t), y.getVelocity(t));
    const double vel_norm_sq = vel.squaredNorm();
    if (max_vel > 0.0 && vel_norm_sq > max_vel * max_vel) {
      return vel * (max_vel / std::sqrt(vel_norm_sq));
    }
    return vel;
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
   * @param acc 加速フェーズの最大加速度
   * @param brk 減速フェーズの最大加速度（0以下のときaccと同じ値を使用）
   * @param accuracy 同期精度
   */
  BangBangTrajectory2D & generate(
    const Eigen::Vector2d & s0, const Eigen::Vector2d & s1, const Eigen::Vector2d & v0,
    const double vmax, const double acc, const double brk = 0.0,
    const double accuracy = DEFAULT_SYNC_ACCURACY)
  {
    max_vel = vmax;
    const double effective_brk = (brk > 0.0) ? brk : acc;
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

      x.generate(s0x, s1x, v0x, vmax * cos_alpha, acc * cos_alpha, effective_brk * cos_alpha);
      y.generate(s0y, s1y, v0y, vmax * sin_alpha, acc * sin_alpha, effective_brk * sin_alpha);

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
