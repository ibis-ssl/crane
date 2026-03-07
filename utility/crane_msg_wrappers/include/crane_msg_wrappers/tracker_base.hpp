// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__TRACKER_BASE_HPP_
#define CRANE_MSG_WRAPPERS__TRACKER_BASE_HPP_

#include <chrono>
#include <cstdint>

namespace crane
{

/**
 * @brief トレースID生成と createTrace() の共通実装を提供する CRTP 基底クラス
 * @tparam Derived 派生クラス
 * @tparam TraceMsg トレースメッセージ型（reference_timestamp_ns と trace_id フィールドが必要）
 */
template <typename Derived, typename TraceMsg>
class TrackerBase
{
public:
  static auto createTrace() -> TraceMsg
  {
    TraceMsg trace;
    auto now = std::chrono::system_clock::now();
    trace.reference_timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    trace.trace_id = ++trace_id_counter_;
    return trace;
  }

protected:
  static uint32_t trace_id_counter_;
};

template <typename Derived, typename TraceMsg>
uint32_t TrackerBase<Derived, TraceMsg>::trace_id_counter_ = 0;

}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__TRACKER_BASE_HPP_
