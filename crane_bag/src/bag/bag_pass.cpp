// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_pass.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>

namespace crane::bag
{

namespace
{

// ─── 検出パラメータ ──────────────────────────────────────────────────────────
// vision ノイズと実測レートを考慮した閾値。値の根拠はコメント参照。

/// キック開始とみなすボール速度の立ち上がり閾値 [m/s]
constexpr double kKickDetectSpeed = 1.5;
/// 立ち上がり確認用の次フレーム最低速度 [m/s]（単発ノイズ除去）
constexpr double kKickConfirmSpeed = 1.2;
/// キッカー帰属のためのボール近傍距離 [m]
constexpr double kKickProximity = 0.35;
/// ongoing_kick を探す先読み時間 [s]（検出器の遅延吸収）
constexpr double kOngoingKickLookahead = 0.25;
/// キック直後に接触・停止判定を無効化する時間 [s]（キッカー自身の接触除外）
constexpr double kSettleTime = 0.15;
/// 接触とみなすロボット中心-ボール距離 [m]（ロボット半径0.09+ボール半径0.02+マージン）
constexpr double kContactDist = 0.15;
/// ball_contact スタンプを「接触中」とみなす鮮度 [s]
constexpr double kContactRecentSec = 0.1;
/// キッカー自身の再接触を有効とみなすためにボールがキック点から離れるべき距離 [m]
constexpr double kKickerReleaseDist = 0.5;
/// パス終端（こぼれ）とみなすボール速度 [m/s]
constexpr double kStopSpeed = 0.3;
/// チップキック飛行中とみなすボール高さ [m]（距離ベース接触判定を無効化）
constexpr double kFlyingHeight = 0.15;
/// 追跡打ち切り時間 [s]
constexpr double kMaxTrackTime = 5.0;
/// シュート除外: ゴールマウス角度範囲に加えるマージン [rad]
constexpr double kShotConeMargin = 0.1;
/// ゴールマウス半幅 [m]（bag_events.cpp の detect_goals と同一値）
constexpr double kGoalHalfWidth = 0.5;
/// 場外判定マージン [m]
constexpr double kOutOfFieldMargin = 0.05;

double norm2(double x, double y) { return std::sqrt(x * x + y * y); }

double ball_speed(const WorldModel & wm)
{
  return norm2(wm.ball_info.velocity.x, wm.ball_info.velocity.y);
}

double dist2d(const Point2D & a, double bx, double by) { return norm2(a.x - bx, a.y - by); }

/// 攻撃方向は +x（world_model は自ゴールが -x に正規化されている。detect_goals と同前提）
bool is_shot_direction(const WorldModel & wm, const Point2D & kick_pos)
{
  const double half_length = wm.field_info.x / 2.0;
  if (half_length <= 0.0) {
    return false;  // フィールド情報なし（合成データ等）は除外しない
  }
  if (kick_pos.x >= half_length) {
    return false;  // ゴールラインより先からのキックは角度が定義できない
  }
  const double vx = wm.ball_info.velocity.x;
  const double vy = wm.ball_info.velocity.y;
  if (norm2(vx, vy) < 1e-6) {
    return false;
  }
  const double dir = std::atan2(vy, vx);
  const double angle_high = std::atan2(kGoalHalfWidth - kick_pos.y, half_length - kick_pos.x);
  const double angle_low = std::atan2(-kGoalHalfWidth - kick_pos.y, half_length - kick_pos.x);
  return dir >= angle_low - kShotConeMargin && dir <= angle_high + kShotConeMargin;
}

/// 意図受け手がキック射線の近く（前方0.3m以降・横ずれ0.6m以内）にいるか。
/// ゴール方向へのキックでも、受け手が射線上にいるならパスとみなすために使う。
bool receiver_on_kick_ray(const WorldModel & wm, const Point2D & kick_pos, int32_t receiver_id)
{
  const double vx = wm.ball_info.velocity.x;
  const double vy = wm.ball_info.velocity.y;
  const double vnorm = norm2(vx, vy);
  if (vnorm < 1e-6) {
    return false;
  }
  for (const auto & r : wm.robot_info_ours) {
    if (static_cast<int32_t>(r.id) != receiver_id) {
      continue;
    }
    const double rx = r.pose.x - kick_pos.x;
    const double ry = r.pose.y - kick_pos.y;
    const double along = (rx * vx + ry * vy) / vnorm;
    if (along < 0.3) {
      return false;
    }
    const double perp = std::abs(rx * vy - ry * vx) / vnorm;
    return perp <= 0.6;
  }
  return false;
}

struct ContactHit
{
  int32_t id = -1;
  bool ours = false;
  double dist = std::numeric_limits<double>::max();
};

}  // namespace

std::string to_string(PassOutcome o)
{
  switch (o) {
    case PassOutcome::SUCCESS:
      return "SUCCESS";
    case PassOutcome::WRONG_RECEIVER:
      return "WRONG_RECEIVER";
    case PassOutcome::INTERCEPTED:
      return "INTERCEPTED";
    case PassOutcome::OVERRUN:
      return "OVERRUN";
    case PassOutcome::OUT_OF_PLAY:
      return "OUT_OF_PLAY";
    case PassOutcome::UNRESOLVED:
      return "UNRESOLVED";
  }
  return "UNKNOWN";
}

size_t pass_distance_band(double distance)
{
  if (distance < 1.5) return 0;
  if (distance <= 4.0) return 1;
  return 2;
}

std::vector<PassEvent> detect_pass_events(const BagData & data)
{
  std::vector<PassEvent> events;
  const auto & wms = data.world_models;
  if (wms.size() < 2) {
    return events;
  }

  bool prev_above = ball_speed(wms.front().msg) >= kKickDetectSpeed;

  for (size_t i = 1; i < wms.size(); ++i) {
    const auto & wm = wms[i].msg;
    const double speed = ball_speed(wm);
    const bool above = speed >= kKickDetectSpeed;
    if (!above || prev_above) {
      prev_above = above;
      continue;
    }
    prev_above = above;

    // 単発ノイズ除去: 次フレームでも速度が維持されていること
    if (i + 1 < wms.size() && ball_speed(wms[i + 1].msg) < kKickConfirmSpeed) {
      continue;
    }

    const auto & prev_wm = wms[i - 1].msg;
    const Point2D kick_pos = {prev_wm.ball_info.position.x, prev_wm.ball_info.position.y};

    // ─ キッカー帰属: ongoing_kick を優先、なければ最近傍ロボット ─
    int32_t kicker_id = -1;
    bool attribution_found = false;
    bool enemy_kick = false;
    for (size_t j = i; j < wms.size(); ++j) {
      if (
        wms[j].t(data.info.start_time_ns) - wms[i].t(data.info.start_time_ns) >
        kOngoingKickLookahead) {
        break;
      }
      const auto & ok = wms[j].msg.ongoing_kick;
      if (ok.present) {
        attribution_found = true;
        enemy_kick = !ok.is_kicker_friend;
        kicker_id = ok.kicker_id;
        break;
      }
    }
    if (!attribution_found) {
      // フォールバック: キック直前のボールに最も近いロボット
      double our_best = std::numeric_limits<double>::max();
      double their_best = std::numeric_limits<double>::max();
      int32_t our_best_id = -1;
      for (const auto & r : prev_wm.robot_info_ours) {
        const double d = dist2d(kick_pos, r.pose.x, r.pose.y);
        if (d < our_best) {
          our_best = d;
          our_best_id = r.id;
        }
      }
      for (const auto & r : prev_wm.robot_info_theirs) {
        their_best = std::min(their_best, dist2d(kick_pos, r.pose.x, r.pose.y));
      }
      if (our_best <= kKickProximity && our_best <= their_best) {
        kicker_id = our_best_id;
      } else {
        enemy_kick = true;
      }
    }
    if (enemy_kick || kicker_id < 0) {
      continue;
    }

    // ─ パス意図: キック判断時点（直前フレーム）の pass_target_id ─
    int32_t intended = prev_wm.pass_target_id >= 0 ? prev_wm.pass_target_id : wm.pass_target_id;
    if (intended < 0 || intended == kicker_id) {
      continue;  // パス意図なし（シュート・クリア等）
    }
    const int32_t reserved = prev_wm.recommended_pass_receiver_id;

    // ─ シュート除外: キック方向がゴールマウスを向いている ─
    // ただし意図受け手が射線上にいる場合はパスとして扱う
    if (is_shot_direction(wm, kick_pos) && !receiver_on_kick_ray(wm, kick_pos, intended)) {
      continue;
    }

    // ─ パス試行として追跡 ─
    PassEvent ev;
    ev.timestamp_ns = wms[i].timestamp_ns;
    ev.t = wms[i].t(data.info.start_time_ns);
    ev.kicker_id = kicker_id;
    ev.intended_receiver_id = intended;
    ev.reserved_receiver_id = reserved;
    ev.kick_pos = kick_pos;

    const int64_t kick_header_ns = wm.header_stamp_ns;
    bool ball_left_kicker = false;
    size_t resolve_frame = wms.size() - 1;

    for (size_t j = i; j < wms.size(); ++j) {
      const auto & cur = wms[j].msg;
      const double dt = (wms[j].timestamp_ns - wms[i].timestamp_ns) / 1e9;
      const double cur_speed = ball_speed(cur);
      const double bx = cur.ball_info.position.x;
      const double by = cur.ball_info.position.y;

      if (dt <= 0.3) {
        ev.kick_speed = std::max(ev.kick_speed, cur_speed);
      }
      if (!ball_left_kicker && norm2(bx - kick_pos.x, by - kick_pos.y) > kKickerReleaseDist) {
        ball_left_kicker = true;
      }

      ev.end_pos = {bx, by};
      ev.duration = dt;
      resolve_frame = j;

      if (dt < kSettleTime) {
        continue;
      }

      // 場外
      const double half_x = cur.field_info.x / 2.0;
      const double half_y = cur.field_info.y / 2.0;
      if (
        half_x > 0.0 &&
        (std::abs(bx) > half_x + kOutOfFieldMargin || std::abs(by) > half_y + kOutOfFieldMargin)) {
        ev.outcome = PassOutcome::OUT_OF_PLAY;
        break;
      }

      // 接触判定（チップ飛行中は距離ベース判定を無効化）
      const bool flying = cur.ball_info.position.z > kFlyingHeight;
      ContactHit hit;
      for (const auto & r : cur.robot_info_ours) {
        if (r.id == kicker_id && !ball_left_kicker) {
          continue;  // キッカー自身の接触はボールが離れるまで除外
        }
        const double d = dist2d({r.pose.x, r.pose.y}, bx, by);
        const bool by_stamp =
          kick_header_ns > 0 && r.ball_contact_last_ns > 0 && cur.header_stamp_ns > 0 &&
          r.ball_contact_last_ns >= kick_header_ns + static_cast<int64_t>(kSettleTime * 1e9) &&
          (cur.header_stamp_ns - r.ball_contact_last_ns) <=
            static_cast<int64_t>(kContactRecentSec * 1e9);
        const bool by_dist = !flying && d <= kContactDist;
        if ((by_stamp || by_dist) && d < hit.dist) {
          hit = {static_cast<int32_t>(r.id), true, d};
        }
      }
      if (!flying) {
        for (const auto & r : cur.robot_info_theirs) {
          const double d = dist2d({r.pose.x, r.pose.y}, bx, by);
          if (d <= kContactDist && d < hit.dist) {
            hit = {static_cast<int32_t>(r.id), false, d};
          }
        }
      }
      if (hit.id >= 0) {
        ev.first_toucher_id = hit.id;
        ev.first_toucher_ours = hit.ours;
        if (!hit.ours) {
          ev.outcome = PassOutcome::INTERCEPTED;
        } else if (hit.id == intended) {
          ev.outcome = PassOutcome::SUCCESS;
        } else {
          ev.outcome = PassOutcome::WRONG_RECEIVER;
        }
        break;
      }

      // 停止（こぼれ）
      if (cur_speed < kStopSpeed) {
        ev.outcome = PassOutcome::OVERRUN;
        break;
      }

      // 打ち切り
      if (dt > kMaxTrackTime) {
        ev.outcome = PassOutcome::UNRESOLVED;
        break;
      }
    }

    ev.pass_distance = norm2(ev.end_pos.x - ev.kick_pos.x, ev.end_pos.y - ev.kick_pos.y);
    ev.forward_progress = ev.end_pos.x - ev.kick_pos.x;
    events.push_back(ev);

    // 解決フレームから走査を再開（同一キックの二重検出を防ぐ）
    i = resolve_frame;
    prev_above = ball_speed(wms[resolve_frame].msg) >= kKickDetectSpeed;
  }

  return events;
}

PassSummary summarize_passes(const std::vector<PassEvent> & events)
{
  PassSummary s;
  s.attempts = events.size();
  if (events.empty()) {
    return s;
  }

  double sum_dist = 0, sum_fwd = 0, sum_dur = 0, sum_speed = 0;
  for (const auto & e : events) {
    switch (e.outcome) {
      case PassOutcome::SUCCESS:
        ++s.success;
        break;
      case PassOutcome::WRONG_RECEIVER:
        ++s.wrong_receiver;
        break;
      case PassOutcome::INTERCEPTED:
        ++s.intercepted;
        break;
      case PassOutcome::OVERRUN:
        ++s.overrun;
        break;
      case PassOutcome::OUT_OF_PLAY:
        ++s.out_of_play;
        break;
      case PassOutcome::UNRESOLVED:
        ++s.unresolved;
        break;
    }
    sum_dist += e.pass_distance;
    sum_fwd += e.forward_progress;
    sum_dur += e.duration;
    sum_speed += e.kick_speed;
    const size_t band = pass_distance_band(e.pass_distance);
    ++s.band_attempts[band];
    if (e.outcome == PassOutcome::SUCCESS) {
      ++s.band_success[band];
    }
  }
  const double n = static_cast<double>(events.size());
  s.avg_distance = sum_dist / n;
  s.avg_forward_progress = sum_fwd / n;
  s.avg_duration = sum_dur / n;
  s.avg_kick_speed = sum_speed / n;
  return s;
}

std::string format_pass_summary(const PassSummary & s)
{
  std::ostringstream out;
  char buf[256];
  out << "=== PASS SUMMARY (ours) ===\n";
  std::snprintf(buf, sizeof(buf), "attempts: %zu\n", s.attempts);
  out << buf;
  if (s.attempts == 0) {
    return out.str();
  }
  const double rate = 100.0 * static_cast<double>(s.success) / static_cast<double>(s.attempts);
  const double int_rate =
    100.0 * static_cast<double>(s.intercepted) / static_cast<double>(s.attempts);
  std::snprintf(
    buf, sizeof(buf),
    "success: %zu (%.1f%%)  wrong_receiver: %zu  intercepted: %zu (%.1f%%)  overrun: %zu  "
    "out_of_play: %zu  unresolved: %zu\n",
    s.success, rate, s.wrong_receiver, s.intercepted, int_rate, s.overrun, s.out_of_play,
    s.unresolved);
  out << buf;
  std::snprintf(
    buf, sizeof(buf),
    "avg pass distance: %.2fm / avg forward progress: %+.2fm / avg duration: %.2fs / avg kick "
    "speed: %.2fm/s\n",
    s.avg_distance, s.avg_forward_progress, s.avg_duration, s.avg_kick_speed);
  out << buf;
  out << "by distance band:";
  for (size_t b = 0; b < 3; ++b) {
    std::snprintf(
      buf, sizeof(buf), "  %s: %zu/%zu", kPassDistanceBandLabels[b], s.band_success[b],
      s.band_attempts[b]);
    out << buf;
  }
  out << "\n";
  return out.str();
}

}  // namespace crane::bag
