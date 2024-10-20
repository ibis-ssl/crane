// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__TIME_HPP_
#define CRANE_BASICS__TIME_HPP_

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

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

class ScopedTimer
{
public:
  explicit ScopedTimer(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub)
  : start(std::chrono::high_resolution_clock::now()), publisher(pub)
  {
  }

  ~ScopedTimer()
  {
    std_msgs::msg::Float32 msg;
    msg.data = getElapsedSec(start);
    publisher->publish(msg);
  }
  double elapsedSec() const { return getElapsedSec(start); }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
};
}  // namespace crane

#endif  // CRANE_BASICS__TIME_HPP_
