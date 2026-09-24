// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <array>
#include <string>
#include <vector>

#include "bag_reader.hpp"

namespace crane::bag
{

/// パス試行の結果分類
enum class PassOutcome {
  SUCCESS,         ///< 意図した受け手（キック時点の pass_target_id）が最初に接触
  WRONG_RECEIVER,  ///< 意図と別の味方が最初に接触（機能的成功・意図不一致）
  INTERCEPTED,     ///< 敵が最初に接触
  OVERRUN,         ///< 誰も触れずにボールが停止（届かない・こぼれた）
  OUT_OF_PLAY,     ///< ボールが場外に出た
  UNRESOLVED,      ///< bag終端・追跡打ち切りで結果不明
};

std::string to_string(PassOutcome o);

/// 検出されたパス試行1件。
/// 注意: 「味方キック + キック時点で pass_target_id が有効 + 非シュート方向」を
/// パス試行とみなすヒューリスティック分類であり、少数の偽陽性/偽陰性を含み得る。
struct PassEvent
{
  int64_t timestamp_ns = 0;  ///< キック検出時刻（bagタイムスタンプ）
  double t = 0.0;            ///< bag先頭からの相対秒
  int32_t kicker_id = -1;
  int32_t intended_receiver_id = -1;  ///< キック時点の pass_target_id
  int32_t reserved_receiver_id = -1;  ///< キック時点の recommended_pass_receiver_id
  PassOutcome outcome = PassOutcome::UNRESOLVED;
  int32_t first_toucher_id = -1;  ///< 最初にボールに触れたロボット（-1: なし）
  bool first_toucher_ours = false;
  double kick_speed = 0.0;        ///< キック直後の最大ボール速度 [m/s]
  double pass_distance = 0.0;     ///< キック点→解決点の距離 [m]
  double forward_progress = 0.0;  ///< 攻撃方向(+x)への前進距離 [m]
  double duration = 0.0;          ///< キック→解決までの時間 [s]
  Point2D kick_pos;
  Point2D end_pos;
};

/// 距離帯: [0]=<1.5m, [1]=1.5-4.0m, [2]=>4.0m
inline constexpr std::array<const char *, 3> kPassDistanceBandLabels = {
  "<1.5m", "1.5-4.0m", ">4.0m"};

struct PassSummary
{
  size_t attempts = 0;
  size_t success = 0;
  size_t wrong_receiver = 0;
  size_t intercepted = 0;
  size_t overrun = 0;
  size_t out_of_play = 0;
  size_t unresolved = 0;
  double avg_distance = 0.0;
  double avg_forward_progress = 0.0;
  double avg_duration = 0.0;
  double avg_kick_speed = 0.0;
  std::array<size_t, 3> band_attempts{};
  std::array<size_t, 3> band_success{};
};

/// /world_model 時系列から味方のパス試行と結果を検出する。
/// BagData のみに依存する純粋関数（合成データで単体テスト可能）。
std::vector<PassEvent> detect_pass_events(const BagData & data);

PassSummary summarize_passes(const std::vector<PassEvent> & events);

std::string format_pass_summary(const PassSummary & s);

/// 距離帯インデックス（kPassDistanceBandLabels に対応）
size_t pass_distance_band(double distance);

}  // namespace crane::bag
