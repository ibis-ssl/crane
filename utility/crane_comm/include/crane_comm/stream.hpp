// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_COMM__STREAM_HPP_
#define CRANE_COMM__STREAM_HPP_

#include <cstdint>
#include <ostream>
#include <type_traits>
#include <vector>

namespace crane
{
template <typename T>
inline auto operator<<(std::ostream & os, const std::vector<T> & vec) -> std::ostream &
{
  os << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    if constexpr (std::is_same_v<T, std::uint8_t>) {
      // uint8_t の場合は int にキャストして数字表示
      os << static_cast<int>(vec[i]);
    } else {
      // それ以外の場合はそのまま出力
      os << vec[i];
    }

    if (i < vec.size() - 1) {
      os << ",";
    }
  }
  os << "]";
  return os;
}
}  // namespace crane

#endif  // CRANE_COMM__STREAM_HPP_
