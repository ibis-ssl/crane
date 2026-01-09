// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__ALLOCATION_STATE_HPP_
#define CRANE_SESSION_CONTROLLER__ALLOCATION_STATE_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <string>
#include <unordered_map>

namespace crane
{

/**
 * @brief ロボット割当の状態管理クラス
 *
 * フレーム間でのロボット割当状態を追跡し、ヒステリシス制御に必要な情報を提供する。
 */
class AllocationState
{
public:
  AllocationState() = default;

  /**
   * @brief 前フレームで指定のロボットが指定のTacticに割り当てられていたかを確認
   * @param robot_id ロボットID
   * @param tactic_name Tactic名
   * @return 割り当てられていた場合true
   */
  [[nodiscard]] auto wasAssignedTo(uint8_t robot_id, const std::string & tactic_name) const -> bool
  {
    auto it = robot_to_tactic_.find(robot_id);
    return it != robot_to_tactic_.end() && it->second == tactic_name;
  }

  /**
   * @brief 指定のロボットの前フレームのターゲット位置を取得
   * @param robot_id ロボットID
   * @return ターゲット位置（存在しない場合はnullopt）
   */
  [[nodiscard]] auto getPrevTarget(uint8_t robot_id) const -> std::optional<Point>
  {
    auto it = robot_to_position_.find(robot_id);
    if (it != robot_to_position_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  /**
   * @brief 指定のロボットの割当継続フレーム数を取得
   * @param robot_id ロボットID
   * @return 継続フレーム数（未割当の場合は0）
   */
  [[nodiscard]] auto getAssignmentDuration(uint8_t robot_id) const -> int
  {
    auto it = assignment_duration_.find(robot_id);
    return it != assignment_duration_.end() ? it->second : 0;
  }

  /**
   * @brief 割当状態を更新
   * @param robot_id ロボットID
   * @param tactic_name 割り当てられたTactic名
   * @param target_position ターゲット位置
   */
  void updateAssignment(uint8_t robot_id, const std::string & tactic_name, const Point & target_position)
  {
    // Tactic割当を更新
    bool same_tactic = wasAssignedTo(robot_id, tactic_name);
    robot_to_tactic_[robot_id] = tactic_name;
    robot_to_position_[robot_id] = target_position;

    // 継続フレーム数を更新
    if (same_tactic) {
      assignment_duration_[robot_id]++;
    } else {
      assignment_duration_[robot_id] = 1;
    }
  }

  /**
   * @brief 指定のロボットの割当状態をクリア
   * @param robot_id ロボットID
   */
  void clearAssignment(uint8_t robot_id)
  {
    robot_to_tactic_.erase(robot_id);
    robot_to_position_.erase(robot_id);
    assignment_duration_.erase(robot_id);
  }

  /**
   * @brief 全ロボットの割当状態をクリア
   */
  void clearAll()
  {
    robot_to_tactic_.clear();
    robot_to_position_.clear();
    assignment_duration_.clear();
  }

  /**
   * @brief 現在割り当てられているロボットIDの一覧を取得
   * @return ロボットIDのベクター
   */
  [[nodiscard]] auto getAllAssignedRobots() const -> std::vector<uint8_t>
  {
    std::vector<uint8_t> robots;
    robots.reserve(robot_to_tactic_.size());
    for (const auto & [robot_id, _] : robot_to_tactic_) {
      robots.push_back(robot_id);
    }
    return robots;
  }

  /**
   * @brief 指定のTacticに割り当てられているロボットIDの一覧を取得
   * @param tactic_name Tactic名
   * @return ロボットIDのベクター
   */
  [[nodiscard]] auto getRobotsAssignedTo(const std::string & tactic_name) const
    -> std::vector<uint8_t>
  {
    std::vector<uint8_t> robots;
    for (const auto & [robot_id, assigned_tactic] : robot_to_tactic_) {
      if (assigned_tactic == tactic_name) {
        robots.push_back(robot_id);
      }
    }
    return robots;
  }

  /**
   * @brief デバッグ用文字列を生成
   * @return 割当状態の文字列表現
   */
  [[nodiscard]] auto toString() const -> std::string
  {
    std::stringstream ss;
    ss << "AllocationState: ";
    for (const auto & [robot_id, tactic_name] : robot_to_tactic_) {
      ss << "[" << static_cast<int>(robot_id) << "->" << tactic_name << "("
         << getAssignmentDuration(robot_id) << ")] ";
    }
    return ss.str();
  }

private:
  // 前フレームの割当情報
  std::unordered_map<uint8_t, std::string> robot_to_tactic_;  // robot_id -> tactic_name
  std::unordered_map<uint8_t, Point> robot_to_position_;      // robot_id -> target_position

  // 割当継続フレーム数
  std::unordered_map<uint8_t, int> assignment_duration_;  // robot_id -> frames
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__ALLOCATION_STATE_HPP_
