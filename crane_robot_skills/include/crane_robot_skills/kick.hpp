// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICK_HPP_
#define CRANE_ROBOT_SKILLS__KICK_HPP_

#include <chrono>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <optional>

namespace crane::skills
{
enum class KickState {
  ENTRY_POINT,
  AROUND_BALL_AND_KICK,
  PIVOT_CAPTURE,
  PIVOT_TURN,
};

class Kick : public SkillBaseWithState
{
private:
  static std::string getStateName(int s);

public:
  template <typename... Args>
  explicit Kick(Args &&... args)
  : SkillBaseWithState(
      static_cast<int>(KickState::ENTRY_POINT), &Kick::getStateName, "kick",
      std::forward<Args>(args)...)
  {
    initialize();
  }

  KickState getCurrentState() const
  {
    return static_cast<KickState>(SkillBaseWithState::getCurrentState());
  }

  /**
   * @brief ボールがフィールドから出る位置を取得
   * @param offset 出る位置を内側にずらすオフセット
   * @return ボールが出る位置
   */
  auto getBallExitPointFromField(const double offset = 0.3) -> Point;

private:
  void initialize();

  void kickWithChip();

  void kickStraight();

  /// @brief ターゲット方向のキックベクトルを計算（フォールバック付き）
  Vector2 computeKickVec() const;

  /// @brief キック方向の矢印と角度しきい値の扇を描画
  void drawKickDirectionVisualization(const Point & ball_pos, const Vector2 & kick_vec);

  /// @brief PIVOT_TURN に入った時刻（タイムアウト検出用）
  std::optional<std::chrono::steady_clock::time_point> pivot_turn_entry_time_;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICK_HPP_
