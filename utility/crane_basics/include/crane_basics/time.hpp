// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

export module crane_basics:time;

import <chrono>;
import <cmath>;

// These ROS2 headers are kept as #include for now.
// They will likely need to be in a global module fragment.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

export namespace crane
{
export template <typename TClock>
auto getDiffSec(std::chrono::time_point<TClock> start, std::chrono::time_point<TClock> end)
  -> double
{
  return std::abs(std::chrono::duration<double>(end - start).count());
}

export template <typename TClock>
auto getElapsedSec(std::chrono::time_point<TClock> start) -> double
{
  return getDiffSec(start, TClock::now());
}

export class ScopedTimer
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
