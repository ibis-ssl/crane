// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__INTERVAL_HPP_
#define CRANE_GEOMETRY__INTERVAL_HPP_

#include <algorithm>
#include <utility>
#include <vector>

class Interval
{
private:
  std::vector<double> uppers;
  std::vector<double> lowers;

public:
  Interval() {}

  ~Interval() {}

  auto append(double a, double b) -> void
  {
    double upper = std::max(a, b);
    double lower = std::min(a, b);
    uppers.emplace_back(upper);
    lowers.emplace_back(lower);

    std::ranges::sort(uppers);
    std::ranges::sort(lowers);
    for (size_t i = 1; i < uppers.size(); i++) {
      // 重なっている
      if (uppers[i - 1] > lowers[i]) {
        uppers[i - 1] = uppers[i];
        lowers.erase(lowers.begin() + i);
        uppers.erase(uppers.begin() + i);
        i--;
      }
    }
  }

  auto erase(double a, double b) -> void
  {
    // 消去区間 [L, U]
    double U = std::max(a, b);
    double L = std::min(a, b);

    // 各既存区間 [lo, hi] から [L, U] を引いた結果を再構築する。
    // uppers/lowers をペアとして扱い、対応関係を崩さないようにする。
    std::vector<double> new_lowers;
    std::vector<double> new_uppers;
    new_lowers.reserve(lowers.size() + 1);
    new_uppers.reserve(uppers.size() + 1);

    for (size_t i = 0; i < uppers.size(); i++) {
      double lo = lowers[i];
      double hi = uppers[i];

      // 重なりなし（境界一致を含む）：区間はそのまま残る
      if (U <= lo || L >= hi) {
        new_lowers.emplace_back(lo);
        new_uppers.emplace_back(hi);
        continue;
      }

      // 完全に覆われる：区間を削除（何も追加しない）
      if (L <= lo && U >= hi) {
        continue;
      }

      // 中抜き（2分割）：[lo, L] と [U, hi]
      if (L > lo && U < hi) {
        new_lowers.emplace_back(lo);
        new_uppers.emplace_back(L);
        new_lowers.emplace_back(U);
        new_uppers.emplace_back(hi);
        continue;
      }

      // 下端を縮める：[U, hi]（L <= lo < U < hi）
      if (L <= lo) {
        new_lowers.emplace_back(U);
        new_uppers.emplace_back(hi);
        continue;
      }

      // 上端を縮める：[lo, L]（lo < L < hi <= U）
      new_lowers.emplace_back(lo);
      new_uppers.emplace_back(L);
    }

    lowers = std::move(new_lowers);
    uppers = std::move(new_uppers);
  }

  auto getWidth() const -> double
  {
    double width = 0.f;
    for (size_t i = 0; i < lowers.size(); i++) {
      width += uppers[i] - lowers[i];
    }
    return width;
  }

  auto getLargestInterval() const -> std::pair<double, double>
  {
    double max_width = 0.f;
    double max_lower = 0.f;
    double max_upper = 0.f;
    for (size_t i = 0; i < lowers.size(); i++) {
      double width = uppers[i] - lowers[i];
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
