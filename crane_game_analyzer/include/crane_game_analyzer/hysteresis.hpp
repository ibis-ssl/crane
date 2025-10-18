// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__HYSTERESIS_HPP_
#define CRANE_GAME_ANALYZER__HYSTERESIS_HPP_

class Hysteresis
{
public:
  Hysteresis(double upperThreshold, double lowerThreshold)
  : upperThreshold(upperThreshold), lowerThreshold(lowerThreshold), state(false)
  {
  }

  auto update(double value) -> void
  {
    // 値が上昇しているときは、上限値で値を切り替え
    if (value > upperThreshold) {
      state = true;
    } else if (value < lowerThreshold) {
      // 値が下降しているときは、下限値で値を切り替え
      state = false;
    }
  }

  auto isUpper() const -> bool { return state; }

private:
  double upperThreshold;
  double lowerThreshold;
  bool state;
};

#endif  // CRANE_GAME_ANALYZER__HYSTERESIS_HPP_
