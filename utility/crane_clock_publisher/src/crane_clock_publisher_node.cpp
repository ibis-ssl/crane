// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <unordered_map>

class ClockPublisher : public rclcpp::Node
{
public:
  ClockPublisher() : Node("clock_publisher")
  {
    declare_parameter<double>("time_scale", 1.0);
    time_scale = get_parameter("time_scale").as_double();
    start_time = this->get_clock()->now();
    publisher = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    using std::chrono_literals::operator""ms;
    timer = this->create_wall_timer(5ms, [this]() {
      auto message = rosgraph_msgs::msg::Clock();
      rclcpp::Time now = this->get_clock()->now();
      rclcpp::Duration elapsed_time = now - start_time;
      double scaled_seconds = elapsed_time.seconds() * time_scale;
      rclcpp::Duration scaled_duration =
        rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(scaled_seconds * 1e9));
      message.clock.sec = scaled_duration.seconds();
      message.clock.nanosec = scaled_duration.nanoseconds();

      message.clock.sec = scaled_duration.seconds();
      message.clock.nanosec = scaled_duration.nanoseconds();
      publisher->publish(message);
    });
  }

private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Time start_time;
  double time_scale;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClockPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
