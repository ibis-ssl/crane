// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__PASS_RATING_MATH_HPP_
#define CRANE_PHYSICS__PASS_RATING_MATH_HPP_

#include <algorithm>
#include <cmath>

namespace crane
{
/// パス候補スコアの入力スカラ（world_model 依存の値は呼び出し側で算出して渡す）
struct PassScoreTerms
{
  double pass_distance = 0.0;                  ///< パス距離 [m]
  double their_goal_angle_width = 0.0;         ///< 受領点から見た敵ゴールの見通し角幅 [rad]
  double own_goal_angle_width = 0.0;           ///< 受領点から見た自ゴールの見通し角幅 [rad]
  double normed_distance_to_their_goal = 0.0;  ///< 敵ゴールへの正規化距離
  double worst_enemy_slack = 1.0;              ///< 敵の安全余裕の最小値（正=安全, 敵なしは 1.0）
  double slack_scale = 1.0;                    ///< intercept_score 正規化スケール [s]
  double shadow_score = 1.0;                   ///< パスライン遮蔽係数
  bool in_penalty_area = false;                ///< 受領点がペナルティエリア内か
};

/// パス候補スコアと内訳（可視化・上位判断で参照）
struct PassRating
{
  double score = 0.0;              ///< 最終スコア
  double distance_factor = 0.0;    ///< 距離係数（乗算）
  double goal_angle_bonus = 0.0;   ///< 敵ゴール見通しボーナス（加算）
  double own_goal_penalty = 0.0;   ///< 自ゴール危険度ペナルティ（減算）
  double their_goal_factor = 0.0;  ///< 敵ゴール接近係数（乗算）
  double intercept_score = 0.0;    ///< 敵インターセプト係数（乗算）
  double shadow_score = 0.0;       ///< パスライン遮蔽係数（乗算）
  bool in_penalty_area = false;    ///< true なら score を 0 に落とす
};

/**
 * @brief パス候補スコアを入力スカラから合成する純関数
 *
 * PassTargetMetric::calcScore の演算列を逐語的に再現する。加算(+=)・減算(-=)と
 * 乗算(*=)が混在するため float 完全一致には順序保存が必須であり、ここで一元化する。
 * 挙動を変えないこと。改良は上位の別マイルストーンで行う。
 */
inline auto combinePassScore(const PassScoreTerms & t) -> PassRating
{
  PassRating r;

  // 距離評価（山型：1.5〜4.0mが最適ゾーン）
  if (t.pass_distance < 1.5) {
    r.distance_factor = t.pass_distance / 1.5;  // 0m→0.0, 1.5m→1.0
  } else if (t.pass_distance <= 4.0) {
    r.distance_factor = 1.0;  // 最適ゾーン
  } else {
    r.distance_factor = std::max(0.2, 1.0 - (t.pass_distance - 4.0) * 0.15);  // 漸減
  }

  r.goal_angle_bonus = std::clamp(t.their_goal_angle_width / (M_PI / 12.), 0.0, 0.5);
  r.own_goal_penalty = std::clamp(t.own_goal_angle_width / (M_PI / 12.), 0.0, 0.5);
  r.their_goal_factor = 1.0 - t.normed_distance_to_their_goal * 0.5;
  r.intercept_score = std::clamp(t.worst_enemy_slack / t.slack_scale, 0.0, 1.0);
  r.shadow_score = t.shadow_score;
  r.in_penalty_area = t.in_penalty_area;

  double score = 1.0;
  score *= r.distance_factor;
  score += r.goal_angle_bonus;
  score -= r.own_goal_penalty;
  score *= r.their_goal_factor;
  score *= r.intercept_score;
  score *= r.shadow_score;
  if (t.in_penalty_area) {
    score = 0.0;
  }
  r.score = score;

  return r;
}
}  // namespace crane

#endif  // CRANE_PHYSICS__PASS_RATING_MATH_HPP_
