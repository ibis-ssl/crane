// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__POINT_CHECKER_HPP_
#define CRANE_MSG_WRAPPERS__POINT_CHECKER_HPP_

#include <crane_geometry/boost_geometry.hpp>

namespace crane
{
// Forward declaration
struct WorldModelWrapper;

/**
 * @brief 点がフィールド・ペナルティエリア・自陣の内側にあるかを判定するクラス
 *
 * WorldModelWrapperから分離されたため、必要な場所でのみインクルード可能。
 */
class PointChecker
{
public:
  explicit PointChecker(WorldModelWrapper * world_model);

  // フィールド境界チェック
  [[nodiscard]] auto isFieldInside(const Point & p, double offset = 0.) const -> bool;

  // ペナルティエリアチェック
  [[nodiscard]] auto isEnemyPenaltyArea(const Point & p, double offset = 0.) const -> bool;
  [[nodiscard]] auto isFriendPenaltyArea(const Point & p, double offset = 0.) const -> bool;
  [[nodiscard]] auto isPenaltyArea(const Point & p, double offset = 0.) const -> bool;

  // ハーフフィールドチェック
  [[nodiscard]] auto isInOurHalf(const Point & p, double offset = 0.) const -> bool;

private:
  WorldModelWrapper * world_model_;
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__POINT_CHECKER_HPP_
