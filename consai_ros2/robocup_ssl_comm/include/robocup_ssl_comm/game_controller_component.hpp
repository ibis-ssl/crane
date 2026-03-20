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

#ifndef ROBOCUP_SSL_COMM__GAME_CONTROLLER_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__GAME_CONTROLLER_COMPONENT_HPP_

#include <robocup_ssl_msgs/ssl_gc_referee_message.pb.h>

#include <boost/asio.hpp>
#include <crane_comm/unicast.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/game_event.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <robocup_ssl_msgs/robocup_ssl_msgs/conversions.hpp>
#include <thread>

#include "visibility_control.h"

namespace asio = boost::asio;

namespace robocup_ssl_comm
{
class GameController : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit GameController(const rclcpp::NodeOptions & options);

  ~GameController();

protected:
  void on_timer();

private:
  rclcpp::TimerBase::SharedPtr timer;

  asio::io_context io_context_;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;
  std::unique_ptr<crane::AsyncUdpReceiver> receiver;

  rclcpp::Publisher<robocup_ssl_msgs::msg::Referee>::SharedPtr pub_referee;
  rclcpp::Publisher<robocup_ssl_msgs::msg::GameEvent>::SharedPtr pub_game_event;

  std::mutex latest_mutex_;
  std::optional<robocup_ssl::Referee> latest_packet_;
  std::vector<robocup_ssl_msgs::msg::GameEvent> previous_game_events_;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__GAME_CONTROLLER_COMPONENT_HPP_
