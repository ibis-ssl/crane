// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PHYSICS__ALLOCATION_HYSTERESIS_HPP_
#define CRANE_PHYSICS__ALLOCATION_HYSTERESIS_HPP_

#include <cassert>
#include <functional>

namespace crane
{
/**
 * @brief 2閾値によるヒステリシス制御クラス（ロボット割当用）
 *
 * Sumatraの実装を参考にした、チャタリング防止のためのヒステリシス機構。
 * lower_threshold未満で下位状態、upper_threshold以上で上位状態に遷移。
 * 閾値間では状態を維持してチャタリングを防止する。
 *
 * 注意: ball_info.hppの既存Hysteresisと競合しないよう、AllocationHysteresisに改名
 *
 * 使用例：
 * @code
 * AllocationHysteresis h(0.3, 0.7);  // 下限0.3、上限0.7
 * h.update(0.8);  // 上位状態へ遷移
 * h.update(0.5);  // 閾値間なので状態維持（上位のまま）
 * h.update(0.2);  // 下位状態へ遷移
 * @endcode
 */
class AllocationHysteresis
{
public:
  /**
   * @brief コンストラクタ
   * @param lower_threshold 下位状態への遷移閾値
   * @param upper_threshold 上位状態への遷移閾値
   */
  AllocationHysteresis(double lower_threshold, double upper_threshold)
  : lower_threshold_(lower_threshold), upper_threshold_(upper_threshold), is_upper_state_(false)
  {
    assert(lower_threshold < upper_threshold);
  }

  /**
   * @brief デフォルトコンストラクタ（閾値0.0, 1.0）
   */
  AllocationHysteresis() : AllocationHysteresis(0.0, 1.0) {}

  /**
   * @brief 値を更新してヒステリシス判定
   * @param value 判定する値
   * @return 自身の参照（メソッドチェーン用）
   */
  auto update(double value) -> AllocationHysteresis &
  {
    if (!is_upper_state_ && value >= upper_threshold_) {
      is_upper_state_ = true;
      if (on_upper_callback_) {
        on_upper_callback_();
      }
    }
    if (is_upper_state_ && value < lower_threshold_) {
      is_upper_state_ = false;
      if (on_lower_callback_) {
        on_lower_callback_();
      }
    }
    return *this;
  }

  /**
   * @brief 上位状態かどうかを返す
   * @return 上位状態の場合true
   */
  bool isUpper() const { return is_upper_state_; }

  /**
   * @brief 下位状態かどうかを返す
   * @return 下位状態の場合true
   */
  bool isLower() const { return !is_upper_state_; }

  /**
   * @brief 上位状態への遷移時のコールバックを設定
   * @param cb コールバック関数
   */
  void setOnUpperCallback(std::function<void()> cb) { on_upper_callback_ = std::move(cb); }

  /**
   * @brief 下位状態への遷移時のコールバックを設定
   * @param cb コールバック関数
   */
  void setOnLowerCallback(std::function<void()> cb) { on_lower_callback_ = std::move(cb); }

  /**
   * @brief 状態を強制的にリセット（下位状態へ）
   */
  void reset() { is_upper_state_ = false; }

  /**
   * @brief 閾値を取得
   */
  double getLowerThreshold() const { return lower_threshold_; }
  double getUpperThreshold() const { return upper_threshold_; }

  /**
   * @brief 閾値を設定
   */
  void setThresholds(double lower, double upper)
  {
    assert(lower < upper);
    lower_threshold_ = lower;
    upper_threshold_ = upper;
  }

private:
  double lower_threshold_;
  double upper_threshold_;
  bool is_upper_state_;
  std::function<void()> on_upper_callback_;
  std::function<void()> on_lower_callback_;
};

}  // namespace crane
#endif  // CRANE_PHYSICS__ALLOCATION_HYSTERESIS_HPP_
