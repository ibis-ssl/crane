// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__INTERVAL_HPP_
#define CRANE_GEOMETRY__INTERVAL_HPP_

#include <algorithm>
#include <vector>

class Interval
{
private:
  std::vector<float> uppers;
  std::vector<float> lowers;

public:
  Interval() {}

  ~Interval() {}

  void append(float a, float b)
  {
    float upper = std::max(a, b);
    float lower = std::min(a, b);
    uppers.emplace_back(upper);
    lowers.emplace_back(lower);

    std::sort(uppers.begin(), uppers.end());
    std::sort(lowers.begin(), lowers.end());
    for (size_t i = 1; i < uppers.size(); i++) {
      //重なっている
      if (uppers[i - 1] > lowers[i]) {
        uppers[i - 1] = uppers[i];
        lowers.erase(lowers.begin() + i);
        uppers.erase(uppers.begin() + i);
        i--;
      }
    }
  }

  void erase(float a, float b)
  {
    float upper = std::max(a, b);
    float lower = std::min(a, b);
    for (size_t i = 0; i < uppers.size(); i++) {
      //完全消去
      if (uppers[i] < upper && lowers[i] > lower) {
        lowers.erase(lowers.begin() + i);
        uppers.erase(uppers.begin() + i);
        i--;
        continue;
      }
      //中抜き
      if (uppers[i] > upper && lowers[i] < lower) {
        uppers.emplace_back(lower);
        lowers.emplace_back(upper);
        std::sort(uppers.begin(), uppers.end());
        std::sort(lowers.begin(), lowers.end());
      }

      //上限修正
      if (lower < uppers[i] && upper > uppers[i]) {
        uppers[i] = lower;
      }
      //下限修正
      if (lowers[i] < upper && lowers[i] > lower) {
        lowers[i] = upper;
      }
    }
    std::sort(uppers.begin(), uppers.end());
    std::sort(lowers.begin(), lowers.end());
  }

  float getWidth()
  {
    float width = 0.f;
    for (size_t i = 0; i < lowers.size(); i++) {
      width += uppers[i] - lowers[i];
    }
    return width;
  }

  auto getLargestInterval() -> std::pair<float, float>
  {
    float max_width = 0.f;
    float max_lower = 0.f;
    float max_upper = 0.f;
    for (size_t i = 0; i < lowers.size(); i++) {
      float width = uppers[i] - lowers[i];
      if (width > max_width) {
        max_width = width;
        max_lower = lowers[i];
        max_upper = uppers[i];
      }
    }
    return std::make_pair(max_lower, max_upper);
  }
};

#endif  // CRANE_GEOMETRY__INTERVAL_HPP_
