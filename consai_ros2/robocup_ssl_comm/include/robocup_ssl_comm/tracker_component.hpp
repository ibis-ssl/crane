// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOCUP_SSL_COMM__TRACKER_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__TRACKER_COMPONENT_HPP_

#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>

#include <boost/asio.hpp>
#include <crane_comm/unicast.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/tracked_frame.hpp>
#include <thread>

#include "visibility_control.h"

namespace asio = boost::asio;

namespace robocup_ssl_comm
{
class Tracker : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit Tracker(const rclcpp::NodeOptions & options);

  ~Tracker();

protected:
  void on_timer();

private:
  robocup_ssl_msgs::msg::TrackedFrame parse_tracked_frame(
    const robocup_ssl::TrackerWrapperPacket & wrapper_packet);

  rclcpp::TimerBase::SharedPtr timer;

  asio::io_context io_context_;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;
  std::unique_ptr<crane::AsyncUdpReceiver> receiver;

  std::mutex latest_mutex_;
  std::optional<robocup_ssl_msgs::msg::TrackedFrame> latest_frame_;

  rclcpp::Publisher<robocup_ssl_msgs::msg::TrackedFrame>::SharedPtr pub_tracked_frame;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__TRACKER_COMPONENT_HPP_
