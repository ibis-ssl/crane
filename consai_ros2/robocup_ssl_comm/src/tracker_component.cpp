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

#include "robocup_ssl_comm/tracker_component.hpp"

#include <chrono>
#include <crane_utils/parameter.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/robocup_ssl_msgs/conversions.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
Tracker::Tracker(const rclcpp::NodeOptions & options) : Node("tracker", options)
{
  const std::string address =
    crane::get_or_declare_parameter(this, "multicast_address", "224.5.23.2");
  const int port = crane::get_or_declare_parameter(this, "multicast_port", 10010);

  receiver = std::make_unique<crane::AsyncUdpReceiver>(asio_ctx_.io_context, address, port);
  receiver->startReceive([this](const std::vector<char> & buf, size_t size) {
    if (size > 0) {
      robocup_ssl::TrackerWrapperPacket wrapper_packet;
      if (
        wrapper_packet.ParseFromArray(buf.data(), static_cast<int>(size)) &&
        wrapper_packet.has_tracked_frame()) {
        auto frame = robocup_ssl_msgs::conversions::Convert(wrapper_packet.tracked_frame());
        std::lock_guard<std::mutex> lock(latest_mutex_);
        latest_frame_ = std::move(frame);
      }
    }
  });

  asio_ctx_.start();

  pub_tracked_frame = create_publisher<robocup_ssl_msgs::msg::TrackedFrame>("tracked_frame", 10);
  timer = rclcpp::create_timer(this, get_clock(), 25ms, std::bind(&Tracker::on_timer, this));
}

Tracker::~Tracker() = default;

void Tracker::on_timer()
{
  std::optional<robocup_ssl_msgs::msg::TrackedFrame> frame;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    frame = std::exchange(latest_frame_, std::nullopt);
  }

  if (frame) {
    pub_tracked_frame->publish(std::move(*frame));
  }
}

}  // namespace robocup_ssl_comm

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::Tracker)
