// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__BALL_PLACEMENT_HISTORY_HPP_
#define CRANE_SESSIONS__BALL_PLACEMENT_HISTORY_HPP_

#include <cstdint>
#include <filesystem>
#include <unordered_map>

namespace crane
{
/// ボールプレイスメント割当の成功/失敗履歴を YAML ファイルに永続化する。
///
/// プロセス再起動を跨いで「どのロボットが何回成功/失敗したか」「最後に
/// 選ばれたのは何順目か」を覚えておき、`BallPlacementSkillSession` の
/// suitability 関数がそれを読んでロボット選定に利用する。
///
/// last_seq はゼロ帯（成功支配で clamp された複数ロボット）でローテーションを
/// 行うためのグローバル順番付けで、completion (SUCCESS/FAILURE) を記録する
/// たびに `next_seq_` を払い出してそのロボットに記録する。
class BallPlacementHistory
{
public:
  struct Entry
  {
    int success = 0;
    int failure = 0;
    std::uint64_t last_seq = 0;
  };

  explicit BallPlacementHistory(const std::filesystem::path & file_path);

  Entry get(std::uint8_t robot_id) const;

  /// 成功を記録し、新しい seq を割り当ててファイルに書き出す。
  bool recordSuccess(std::uint8_t robot_id);

  /// 失敗を記録し、新しい seq を割り当ててファイルに書き出す。
  bool recordFailure(std::uint8_t robot_id);

  std::uint64_t nextSeq() const { return next_seq_; }

private:
  void load();
  bool save() const;

  std::filesystem::path file_path_;
  std::unordered_map<std::uint8_t, Entry> entries_;
  std::uint64_t next_seq_ = 1;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__BALL_PLACEMENT_HISTORY_HPP_
