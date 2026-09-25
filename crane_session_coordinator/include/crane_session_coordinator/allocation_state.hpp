// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_COORDINATOR__ALLOCATION_STATE_HPP_
#define CRANE_SESSION_COORDINATOR__ALLOCATION_STATE_HPP_

#include <cstdint>
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
   * @brief 前フレームで指定のロボットが指定のSessionに割り当てられていたかを確認
   * @param robot_id ロボットID
   * @param session_name Session名
   * @return 割り当てられていた場合true
   */
  [[nodiscard]] auto wasAssignedTo(uint8_t robot_id, const std::string & session_name) const -> bool
  {
    auto it = robot_to_session_.find(robot_id);
    return it != robot_to_session_.end() && it->second == session_name;
  }

  /**
   * @brief 割当状態を更新
   * @param robot_id ロボットID
   * @param session_name 割り当てられたSession名
   */
  void updateAssignment(uint8_t robot_id, const std::string & session_name)
  {
    robot_to_session_[robot_id] = session_name;
  }

private:
  // 前フレームの割当情報
  std::unordered_map<uint8_t, std::string> robot_to_session_;  // robot_id -> session_name
};

}  // namespace crane
#endif  // CRANE_SESSION_COORDINATOR__ALLOCATION_STATE_HPP_
