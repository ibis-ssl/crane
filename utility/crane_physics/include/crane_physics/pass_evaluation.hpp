// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__PASS_EVALUATION_HPP_
#define CRANE_PHYSICS__PASS_EVALUATION_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/robot_info.hpp>
#include <memory>
#include <vector>

namespace crane
{

/**
 * @brief パスラインに対する敵ロボットのシャドウ（遮蔽）効果を評価
 *
 * パス起点からターゲット位置へのパスラインに対して、敵ロボットが形成する
 * 「影」を評価し、パスの成功率を0.0〜1.0のペナルティで返す。
 *
 * アルゴリズム（シンプル版）:
 * - パスラインに近い敵を検出（距離閾値: robot_radius）
 * - 敵がパスターゲット方向に遮蔽を形成するか角度で判定
 * - 遮蔽度に応じてペナルティを累積
 *
 * @param ball_pos パス起点（ボール位置）
 * @param target_pos パスターゲット位置
 * @param enemies 敵ロボットのリスト
 * @param robot_radius ロボットの半径 [m]（デフォルト: 0.09m）
 * @param shadow_threshold パスラインへの距離閾値 [m]（デフォルト: 0.3m）
 * @return シャドウペナルティ（0.0 = 完全遮蔽, 1.0 = 遮蔽なし）
 */
inline auto evaluatePassShadow(
  const Point & ball_pos, const Point & target_pos,
  const std::vector<std::shared_ptr<RobotInfo>> & enemies, const double robot_radius = 0.09,
  const double shadow_threshold = 0.5) -> double
{
  const Segment pass_line{ball_pos, target_pos};
  const Vector2 pass_dir = (target_pos - ball_pos).normalized();
  const double pass_distance = (target_pos - ball_pos).norm();

  if (pass_distance < 1e-6) {
    return 1.0;  // パス距離がゼロの場合は評価不要
  }

  double penalty_factor = 1.0;

  for (const auto & enemy : enemies) {
    // 敵がパスラインに十分近いかチェック
    auto result = getClosestPointAndDistance(enemy->pose.pos, pass_line);

    if (result.distance > shadow_threshold) {
      continue;  // パスラインから遠い敵は無視
    }

    // 敵がパス起点とターゲットの間にいるかチェック
    const double projection = (result.closest_point - ball_pos).norm();
    if (projection < 0.0 || projection > pass_distance) {
      continue;  // パスライン上にない敵は無視
    }

    // 敵による遮蔽角度を計算
    const double dist_to_ball = (enemy->pose.pos - ball_pos).norm();
    if (dist_to_ball < 1e-6) {
      penalty_factor *= 0.0;  // ボール直上の敵は完全遮蔽
      continue;
    }

    // パスライン上にロボット半径以内の敵は完全遮蔽
    if (result.distance <= robot_radius) {
      penalty_factor *= 0.0;
      continue;
    }

    // 遮蔽角度: atan2(robot_radius, dist_to_ball)
    const double shadow_angle = std::atan2(robot_radius, dist_to_ball);

    // パスターゲット方向が遮蔽コーン内にあるかチェック
    const Vector2 enemy_dir = (enemy->pose.pos - ball_pos).normalized();
    const double angle_to_target = std::acos(std::clamp(pass_dir.dot(enemy_dir), -1.0, 1.0));

    if (angle_to_target < shadow_angle) {
      // 遮蔽度に応じてペナルティを適用
      // 完全遮蔽(角度0)で0.0、遮蔽角度の境界で1.0
      const double shadow_ratio = 1.0 - (angle_to_target / shadow_angle);
      const double shadow_penalty = 1.0 - shadow_ratio * 0.95;  // 最大95%減
      penalty_factor *= shadow_penalty;
    }
  }

  return penalty_factor;
}

}  // namespace crane

#endif  // CRANE_PHYSICS__PASS_EVALUATION_HPP_
