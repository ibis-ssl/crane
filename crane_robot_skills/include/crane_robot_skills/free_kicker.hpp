// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__FREE_KICKER_HPP_
#define CRANE_ROBOT_SKILLS__FREE_KICKER_HPP_

#include <chrono>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace crane::skills
{
enum class FreeKickerState {
  ENTRY_POINT,
  APPROACH,
  ALIGN,
  KICK,
  FINISH,
};

class FreeKicker : public SkillBaseWithState
{
public:
  template <typename... Args>
  explicit FreeKicker(Args &&... args)
  : SkillBaseWithState(
      static_cast<int>(FreeKickerState::ENTRY_POINT), &FreeKicker::getStateName, "free_kicker",
      std::forward<Args>(args)...)
  {
    initialize();
  }

  FreeKickerState getCurrentState() const
  {
    return static_cast<FreeKickerState>(SkillBaseWithState::getCurrentState());
  }

  /// 直近の試行で実際にボールが意図方向へ高速発射されたか。
  /// FINISH 状態到達時点で確定する: ball_speed > FK_KICK_DETECT_VEL かつ
  /// 意図方向との cos が FK_KICK_DIRECTION_COS_THRESHOLD を超えていれば true、
  /// FK_KICK_TIMEOUT_SEC タイムアウトで FINISH した場合は false。
  /// 試行ごと(ENTRY_POINT 自己遷移時) に false にリセットされる。
  bool kickActuallyLaunched() const { return kick_actually_launched_; }

private:
  void initialize();
  static std::string getStateName(int s);

  void resetInternalState();

  // ターゲット選定
  Point selectKickTarget();
  bool tryShoot(Point & out_target);
  std::optional<Point> selectPassTarget();
  double calculatePassScore(
    const Point & target, const std::vector<std::shared_ptr<RobotInfo>> & enemies,
    double best_enemy_slack) const;
  Point computeFallbackTarget();

  Point kick_target_{Point::Zero()};
  Point standoff_{Point::Zero()};
  bool target_locked_ = false;
  bool use_chip_ = false;
  bool last_chose_shoot_ = false;
  std::optional<uint8_t> last_pass_receiver_id_;

  double chip_distance_ = 2.5;

  bool kick_started_ = false;
  std::chrono::steady_clock::time_point kick_started_time_{};

  std::optional<std::chrono::steady_clock::time_point> approach_entry_time_;
  std::optional<std::chrono::steady_clock::time_point> align_entry_time_;

  int align_stable_count_ = 0;

  bool kick_actually_launched_ = false;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__FREE_KICKER_HPP_
