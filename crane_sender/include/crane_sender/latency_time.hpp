// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__LATENCY_TIME_HPP_
#define CRANE_SENDER__LATENCY_TIME_HPP_

#include <algorithm>
#include <cstdint>

namespace crane
{
// RobotCommand.latency_ms（float32）を RobotCommandV2::latency_time_ms（uint16）へ変換する。
// 範囲外の float から整数への変換は未定義動作なので、負と NaN は 0、65535 超は 65535 に寄せる
inline uint16_t toLatencyTimeMs(float latency_ms)
{
  if (!(latency_ms > 0.0f)) {
    return 0;
  }
  return static_cast<uint16_t>(std::min(latency_ms, 65535.0f));
}
}  // namespace crane

#endif  // CRANE_SENDER__LATENCY_TIME_HPP_
