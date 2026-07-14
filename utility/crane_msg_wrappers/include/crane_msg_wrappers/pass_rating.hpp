// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__PASS_RATING_HPP_
#define CRANE_MSG_WRAPPERS__PASS_RATING_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_physics/pass_rating_math.hpp>
#include <crane_physics/slack_time_config.hpp>

namespace crane
{
// 前方宣言（実装は world_model_wrapper.hpp）
struct WorldModelWrapper;

/// ratePassCandidate の設定
struct PassRatingConfig
{
  double slack_scale = 1.0;  ///< intercept_score 正規化スケール [s]
  /// 敵インターセプト評価の物理パラメータ（PassTargetMetric の既定と一致）
  SlackTimeConfig enemy_slack{
    .robot_max_acceleration = 3.0,
    .robot_max_velocity = 5.5,
  };
};

/**
 * @brief パス候補（起点→受領点）を評価しスコアと内訳を返す
 *
 * PassTargetMetric::calcScore と同一の評価。world_model から角度幅・敵slack・遮蔽等の
 * スカラを集め、crane_physics の combinePassScore で合成する。receiver 走行時間や
 * リードパス（受領点≠受け手現在位置）の扱いは含まない（上位マイルストーンで拡張）。
 */
auto ratePassCandidate(
  WorldModelWrapper * world_model, const Point & pass_origin, const Point & target,
  const PassRatingConfig & config = {}) -> PassRating;
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__PASS_RATING_HPP_
