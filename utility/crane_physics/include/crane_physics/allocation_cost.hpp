// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__ALLOCATION_COST_HPP_
#define CRANE_PHYSICS__ALLOCATION_COST_HPP_

namespace crane
{

/**
 * @brief ロボット割当コスト計算の設定パラメータ
 */
struct AllocationCostConfig
{
  // ヒステリシスボーナス（同一Session継続時のコスト減少）[m]
  double hysteresis_bonus = 1.5;
};

}  // namespace crane
#endif  // CRANE_PHYSICS__ALLOCATION_COST_HPP_
