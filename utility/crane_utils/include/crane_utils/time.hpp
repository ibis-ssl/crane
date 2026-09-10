// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_UTILS__TIME_HPP_
#define CRANE_UTILS__TIME_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace crane
{
template <typename TClock>
auto getDiffSec(std::chrono::time_point<TClock> start, std::chrono::time_point<TClock> end)
  -> double
{
  return std::abs(std::chrono::duration<double>(end - start).count());
}

template <typename TClock>
auto getElapsedSec(std::chrono::time_point<TClock> start) -> double
{
  return getDiffSec(start, TClock::now());
}

/**
 * @brief 2つの rclcpp::Time の差分（絶対値、秒単位）を取得
 */
inline auto getDiffSec(const rclcpp::Time & start, const rclcpp::Time & end) -> double
{
  return std::abs((end - start).seconds());
}

/**
 * @brief 2つの builtin_interfaces::msg::Time の差分（絶対値、秒単位）を取得
 */
inline auto getDiffSec(
  const builtin_interfaces::msg::Time & start, const builtin_interfaces::msg::Time & end) -> double
{
  return getDiffSec(rclcpp::Time(start), rclcpp::Time(end));
}

/**
 * @brief rclcpp::Time と builtin_interfaces::msg::Time の差分（絶対値、秒単位）を取得
 */
inline auto getDiffSec(const rclcpp::Time & start, const builtin_interfaces::msg::Time & end)
  -> double
{
  return getDiffSec(start, rclcpp::Time(end));
}

inline auto getDiffSec(const builtin_interfaces::msg::Time & start, const rclcpp::Time & end)
  -> double
{
  return getDiffSec(rclcpp::Time(start), end);
}

/**
 * @brief 開始時刻からの経過時間（秒単位: now - start）を取得
 */
inline auto getElapsedSec(const rclcpp::Time & start, const rclcpp::Time & now) -> double
{
  return (now - start).seconds();
}

inline auto getElapsedSec(const builtin_interfaces::msg::Time & start, const rclcpp::Time & now)
  -> double
{
  return getElapsedSec(rclcpp::Time(start), now);
}

/**
 * @brief タイムスタンプが有効値（0秒・0ナノ秒でない）か判定
 */
inline auto isValidTime(const builtin_interfaces::msg::Time & stamp) -> bool
{
  return stamp.sec != 0 || stamp.nanosec != 0;
}

inline auto isValidTime(const rclcpp::Time & stamp) -> bool { return stamp.nanoseconds() > 0; }

/**
 * @brief 経過時間が指定秒数を超過しているか（タイムアウト判定）
 */
inline auto isTimeout(const rclcpp::Time & start, double timeout_sec, const rclcpp::Time & now)
  -> bool
{
  return getElapsedSec(start, now) > timeout_sec;
}

inline auto isTimeout(
  const builtin_interfaces::msg::Time & start, double timeout_sec, const rclcpp::Time & now) -> bool
{
  return isTimeout(rclcpp::Time(start), timeout_sec, now);
}

class ScopedTimer
{
public:
  explicit ScopedTimer(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub) : publisher(pub) {}

  ~ScopedTimer()
  {
    std_msgs::msg::Float32 msg;
    msg.data = getElapsedSec(start);
    publisher->publish(msg);
  }
  auto elapsedSec() const -> double { return getElapsedSec(start); }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start =
    std::chrono::high_resolution_clock::now();

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
};
}  // namespace crane

#endif  // CRANE_UTILS__TIME_HPP_
