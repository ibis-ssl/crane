// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#ifndef CRANE_GEOMETRY__TIME_HPP_
#define CRANE_GEOMETRY__TIME_HPP_

#include <chrono>
#include <cmath>

namespace crane
{
template <typename TClock>
double getDiffSec(std::chrono::time_point<TClock> start, std::chrono::time_point<TClock> end)
{
  return std::abs(std::chrono::duration<double>(end - start).count());
}

template <typename TClock>
double getElapsedSec(std::chrono::time_point<TClock> start)
{
  return getDiffSec(start, TClock::now());
}
} // namespace crane

#endif  //CRANE_GEOMETRY__TIME_HPP_
