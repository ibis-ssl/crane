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

#include "robocup_ssl_comm/game_controller_component.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
GameController::GameController(const rclcpp::NodeOptions & options)
: Node("game_controller", options)
{
  declare_parameter("multicast_address", "224.5.23.1");
  declare_parameter("multicast_port", 10003);
  const std::string address = get_parameter("multicast_address").get_value<std::string>();
  const int port = get_parameter("multicast_port").get_value<int>();

  receiver = std::make_unique<crane::AsyncUdpReceiver>(asio_ctx_.io_context, address, port);
  receiver->startReceive([this](const std::vector<char> & buf, size_t size) {
    if (size > 0) {
      robocup_ssl::Referee packet;
      if (packet.ParseFromArray(buf.data(), static_cast<int>(size))) {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        latest_packet_ = std::move(packet);
        packet_received_.store(true, std::memory_order_relaxed);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Refereeパケットのパース失敗 (size=%zu) — protoバージョン不整合の可能性", size);
      }
    }
  });

  asio_ctx_.start();

  pub_referee = create_publisher<robocup_ssl_msgs::msg::Referee>("referee", 10);
  pub_game_event = create_publisher<robocup_ssl_msgs::msg::GameEvent>("game_event", 10);
  timer = rclcpp::create_timer(this, get_clock(), 25ms, std::bind(&GameController::on_timer, this));

  status_timer_ = rclcpp::create_timer(this, get_clock(), 1000ms, [this, address, port]() {
    if (!packet_received_.exchange(false, std::memory_order_relaxed)) {
      RCLCPP_WARN(get_logger(), "Referee受信が直近1秒間ありません (%s:%d)", address.c_str(), port);
    }
  });

  RCLCPP_INFO(
    get_logger(), "game_controller initialized - listening on %s:%d", address.c_str(), port);
}

GameController::~GameController() = default;

void GameController::on_timer()
{
  std::optional<robocup_ssl::Referee> packet;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    packet = std::exchange(latest_packet_, std::nullopt);
  }

  if (!packet) {
    return;
  }

  // Use proto2ros Convert API to convert protobuf message to ROS message
  auto referee_msg = std::make_unique<robocup_ssl_msgs::msg::Referee>();
  robocup_ssl_msgs::conversions::Convert(*packet, referee_msg.get());
  pub_referee->publish(std::move(referee_msg));

  // Process game events - only publish new events
  std::vector<robocup_ssl_msgs::msg::GameEvent> current_events;
  for (const auto & proto_event : packet->game_events()) {
    robocup_ssl_msgs::msg::GameEvent event_msg;
    robocup_ssl_msgs::conversions::Convert(proto_event, &event_msg);
    current_events.push_back(event_msg);
  }

  for (const auto & event : current_events) {
    auto it = std::find(previous_game_events_.begin(), previous_game_events_.end(), event);
    if (it == previous_game_events_.end()) {
      pub_game_event->publish(event);
    }
  }

  previous_game_events_ = std::move(current_events);
}

}  // namespace robocup_ssl_comm

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::GameController)
