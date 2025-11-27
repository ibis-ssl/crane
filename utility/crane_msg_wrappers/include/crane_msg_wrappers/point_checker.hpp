// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__POINT_CHECKER_HPP_
#define CRANE_MSG_WRAPPERS__POINT_CHECKER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_physics/robot_info.hpp>
#include <functional>
#include <memory>
#include <ranges>
#include <vector>

namespace crane
{
// Forward declaration
struct WorldModelWrapper;
using RobotList = std::vector<std::shared_ptr<RobotInfo>>;

/**
 * @brief 点が特定の条件を満たすかどうかをチェックするクラス
 *
 * 複数のチェッカーを追加し、すべての条件を満たすかどうかを判定できる。
 * WorldModelWrapperから分離されたため、必要な場所でのみインクルード可能。
 */
class PointChecker
{
public:
  explicit PointChecker(std::shared_ptr<WorldModelWrapper> & world_model);
  explicit PointChecker(WorldModelWrapper * world_model);

  // フィールド境界チェック
  [[nodiscard]] auto isFieldInside(const Point & p, double offset = 0.) const -> bool;

  auto addFieldInsideChecker(double offset = 0.) -> void;
  auto addFieldOutsideChecker(double offset = 0.) -> void;

  // ボールプレースメントエリアチェック
  [[nodiscard]] auto isBallPlacementArea(const Point & p, double offset = 0.) const -> bool;

  auto addBallPlacementAreaInsideChecker(double offset = 0.) -> void;
  auto addBallPlacementAreaOutsideChecker(double offset = 0.) -> void;

  // ペナルティエリアチェック
  [[nodiscard]] auto isEnemyPenaltyArea(const Point & p, double offset = 0.) const -> bool;
  [[nodiscard]] auto isFriendPenaltyArea(const Point & p, double offset = 0.) const -> bool;
  [[nodiscard]] auto isPenaltyArea(const Point & p, double offset = 0.) const -> bool;

  auto addEnemyPenaltyAreaInsideChecker(double offset = 0.) -> void;
  auto addEnemyPenaltyAreaOutsideChecker(double offset = 0.) -> void;
  auto addFriendPenaltyAreaInsideChecker(double offset = 0.) -> void;
  auto addFriendPenaltyAreaOutsideChecker(double offset = 0.) -> void;
  auto addPenaltyAreaInsideChecker(double offset = 0.) -> void;
  auto addPenaltyAreaOutsideChecker(double offset = 0.) -> void;

  // 複合判定
  [[nodiscard]] auto isFieldInsideAndOutsidePenaltyArea(
    const Point & p, double field_offset = 0., double penalty_offset = 0.) const -> bool;

  // ハーフフィールドチェック
  [[nodiscard]] auto isInOurHalf(const Point & p, double offset = 0.) const -> bool;
  auto addInOurHalfChecker(double offset = 0.) -> void;

  // 距離ベースのチェック
  enum class Rule {
    EQUAL_TO,
    NOT_EQUAL_TO,
    LESS_THAN,
    GREATER_THAN,
    LESS_THAN_OR_EQUAL_TO,
    GREATER_THAN_OR_EQUAL_TO,
  };

  [[nodiscard]] auto checkDistance(
    const Point & p, const Point & target, double threshold, Rule rule) const -> bool;

  auto addDistanceChecker(const Point & target, double threshold, Rule rule) -> void;

  [[nodiscard]] auto checkDistanceFromBall(const Point & p, double threshold, Rule rule) const
    -> bool;
  auto addDistanceFromBallChecker(double threshold, Rule rule) -> void;

  [[nodiscard]] auto checkDistanceFromRobot(
    const Point & p, RobotIdentifier id, double threshold, Rule rule) const -> bool;
  auto addDistanceFromRobotChecker(RobotIdentifier id, double threshold, Rule rule) -> void;

  [[nodiscard]] auto checkDistanceFromRobot(
    const Point & p, std::shared_ptr<RobotInfo> robot, double threshold, Rule rule) const -> bool;

  [[nodiscard]] auto checkDistanceFromRobots(
    const Point & p, const RobotList & robots, double threshold, Rule rule) const -> bool;
  auto addDistanceFromRobotsChecker(const RobotList & robots, double threshold, Rule rule) -> void;

  [[nodiscard]] auto checkDistanceFromOurRobots(const Point & p, double threshold, Rule rule) const
    -> bool;
  auto addDistanceFromOurRobotsChecker(double threshold, Rule rule) -> void;

  [[nodiscard]] auto checkDistanceFromTheirRobots(const Point & p, double threshold, Rule rule)
    const -> bool;
  auto addDistanceFromTheirRobotsChecker(double threshold, Rule rule) -> void;

  // カスタムチェッカー
  auto addCustomChecker(std::function<bool(const Point &)> checker) -> void;

  // すべてのチェッカーを適用
  auto operator()(const Point & p) const -> bool;

  // 標準的なチェッカーを構築
  static auto buildStandard(std::shared_ptr<WorldModelWrapper> world_model) -> PointChecker;

private:
  WorldModelWrapper * world_model_;
  std::vector<std::function<bool(const Point &)>> checkers_;
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__POINT_CHECKER_HPP_
