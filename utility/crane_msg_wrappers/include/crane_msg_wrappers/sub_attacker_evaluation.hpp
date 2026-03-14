// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__SUB_ATTACKER_EVALUATION_HPP_
#define CRANE_MSG_WRAPPERS__SUB_ATTACKER_EVALUATION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>

namespace crane
{

/**
 * @brief SubAttacker配置の評価スコアを計算する
 *
 * ゴール可視角度・敵ブロック率・パス距離・シュートライン等を総合評価する。
 *
 * @param p 評価する候補位置
 * @param world_model ワールドモデル参照
 * @return スコア（高いほど良い配置）
 */
double evaluateSubAttackerPosition(const Point & p, const WorldModelWrapper & world_model);

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__SUB_ATTACKER_EVALUATION_HPP_
