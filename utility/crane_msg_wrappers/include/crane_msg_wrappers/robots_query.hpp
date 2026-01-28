// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROBOTS_QUERY_HPP_
#define CRANE_MSG_WRAPPERS__ROBOTS_QUERY_HPP_

#include <crane_physics/robot_info.hpp>
#include <functional>
#include <memory>
#include <vector>

namespace crane
{

/**
 * @brief ロボットリストをクエリ形式で絞り込むためのFluent Builder APIクラス
 *
 * 使用例:
 * @code
 * // 基本的な使用方法
 * auto robots = team.robotsWhere().available().get();
 *
 * // 複数の条件を組み合わせ
 * auto robots = team.robotsWhere()
 *     .available()
 *     .excludeGoalie()
 *     .excludeId(my_id)
 *     .get();
 *
 * // カスタム条件の追加
 * auto robots = team.robotsWhere()
 *     .available()
 *     .where([&](const auto& robot) {
 *         return robot->getDistance(ball_pos) < 1.0;
 *     })
 *     .get();
 * @endcode
 */
class RobotsQuery
{
public:
  using RobotList = std::vector<RobotInfo::SharedPtr>;
  using Predicate = std::function<bool(const RobotInfo::SharedPtr &)>;

  /**
   * @brief コンストラクタ
   * @param robots 元となるロボットリスト
   * @param goalie_id ゴーリーのID
   */
  explicit RobotsQuery(const RobotList & robots, uint8_t goalie_id);

  // ===== 可用性条件（相互排他） =====

  /**
   * @brief 標準的な可用性判定
   * (available_vision || available_tracker) && available_hardware
   */
  auto available() -> RobotsQuery &;

  /**
   * @brief 厳密な可用性判定
   * available_vision && available_hardware && available_feedback
   */
  auto availableStrict() -> RobotsQuery &;

  /**
   * @brief 緩和された可用性判定
   * available_vision || available_tracker
   */
  auto availableLoose() -> RobotsQuery &;

  // ===== 除外条件 =====

  /**
   * @brief 特定のIDを除外
   * @param id 除外するロボットID
   */
  auto excludeId(uint8_t id) -> RobotsQuery &;

  /**
   * @brief 複数のIDを除外
   * @param ids 除外するロボットIDのリスト
   */
  auto excludeIds(const std::vector<uint8_t> & ids) -> RobotsQuery &;

  /**
   * @brief ゴーリーを除外
   */
  auto excludeGoalie() -> RobotsQuery &;

  // ===== カスタム条件 =====

  /**
   * @brief カスタム述語を追加
   * @param pred ロボットを判定する述語関数
   */
  auto where(Predicate pred) -> RobotsQuery &;

  // ===== 終端操作 =====

  /**
   * @brief フィルタリングされたロボットリストを取得
   * @return 条件を満たすロボットのリスト
   */
  [[nodiscard]] auto get() const -> RobotList;

  /**
   * @brief フィルタリングされたロボットIDのリストを取得
   * @return 条件を満たすロボットのIDリスト
   */
  [[nodiscard]] auto getIds() const -> std::vector<uint8_t>;

  /**
   * @brief フィルタリングされたロボットのranges viewを取得
   * @return 条件を満たすロボットのview
   */
  [[nodiscard]] auto getView() const -> decltype(auto);

  /**
   * @brief フィルタリングされたロボットの数を取得
   * @return 条件を満たすロボットの数
   */
  [[nodiscard]] auto count() const -> size_t;

  /**
   * @brief フィルタリング結果が空かどうかを判定
   * @return 条件を満たすロボットが0の場合true
   */
  [[nodiscard]] auto empty() const -> bool;

private:
  const RobotList & robots_;
  uint8_t goalie_id_;
  std::vector<Predicate> predicates_;
};

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__ROBOTS_QUERY_HPP_
