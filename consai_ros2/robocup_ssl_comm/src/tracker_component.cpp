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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
Tracker::Tracker(const rclcpp::NodeOptions & options) : Node("tracker", options)
{
  declare_parameter("multicast_address", "224.5.23.2");
  declare_parameter("multicast_port", 10010);
  receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("multicast_address").get_value<std::string>(),
    get_parameter("multicast_port").get_value<int>());
  pub_tracked_frame = create_publisher<robocup_ssl_msgs::msg::TrackedFrame>("tracked_frame", 10);
  timer = rclcpp::create_timer(this, get_clock(), 25ms, std::bind(&Tracker::on_timer, this));
}

void Tracker::on_timer()
{
  while (receiver->available()) {
    std::vector<char> buf(2048);
    const size_t size = receiver->receive(buf);

    if (size > 0) {
      TrackerWrapperPacket wrapper_packet;
      wrapper_packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if (wrapper_packet.has_tracked_frame()) {
        auto tracked_frame_msg = std::make_unique<robocup_ssl_msgs::msg::TrackedFrame>();
        *tracked_frame_msg = parse_tracked_frame(wrapper_packet);
        pub_tracked_frame->publish(std::move(tracked_frame_msg));
      }
    }
  }
}

robocup_ssl_msgs::msg::TrackedFrame Tracker::parse_tracked_frame(
  const TrackerWrapperPacket & wrapper_packet)
{
  robocup_ssl_msgs::msg::TrackedFrame tracked_frame_msg;
  const auto & tracked_frame = wrapper_packet.tracked_frame();

  tracked_frame_msg.frame_number = tracked_frame.frame_number();
  tracked_frame_msg.timestamp = tracked_frame.timestamp();

  // Parse tracked balls
  for (const auto & ball : tracked_frame.balls()) {
    robocup_ssl_msgs::msg::TrackedBall ball_msg;

    // Position (required)
    ball_msg.pos.x = ball.pos().x();
    ball_msg.pos.y = ball.pos().y();
    ball_msg.pos.z = ball.pos().z();

    // Velocity (optional)
    if (ball.has_vel()) {
      robocup_ssl_msgs::msg::Vector3 velocity;
      velocity.x = ball.vel().x();
      velocity.y = ball.vel().y();
      velocity.z = ball.vel().z();
      ball_msg.vel.push_back(velocity);
    }

    // Visibility (optional)
    if (ball.has_visibility()) {
      ball_msg.visibility.push_back(ball.visibility());
    }

    tracked_frame_msg.balls.push_back(ball_msg);
  }

  // Parse tracked robots
  for (const auto & robot : tracked_frame.robots()) {
    robocup_ssl_msgs::msg::TrackedRobot robot_msg;

    // Robot ID (required)
    robot_msg.robot_id.id = robot.robot_id().id();
    robot_msg.robot_id.team = robot.robot_id().team();

    // Position and orientation (required)
    robot_msg.pos.x = robot.pos().x();
    robot_msg.pos.y = robot.pos().y();
    robot_msg.orientation = robot.orientation();

    // Velocity (optional)
    if (robot.has_vel()) {
      robocup_ssl_msgs::msg::Vector2 velocity;
      velocity.x = robot.vel().x();
      velocity.y = robot.vel().y();
      robot_msg.vel.push_back(velocity);
    }

    // Angular velocity (optional)
    if (robot.has_vel_angular()) {
      robot_msg.vel_angular.push_back(robot.vel_angular());
    }

    // Visibility (optional)
    if (robot.has_visibility()) {
      robot_msg.visibility.push_back(robot.visibility());
    }

    tracked_frame_msg.robots.push_back(robot_msg);
  }

  // Parse kicked ball (optional)
  if (tracked_frame.has_kicked_ball()) {
    robocup_ssl_msgs::msg::KickedBall kicked_ball_msg;
    const auto & kicked_ball = tracked_frame.kicked_ball();

    // Position (required)
    kicked_ball_msg.pos.x = kicked_ball.pos().x();
    kicked_ball_msg.pos.y = kicked_ball.pos().y();

    // Initial velocity (required)
    kicked_ball_msg.vel.x = kicked_ball.vel().x();
    kicked_ball_msg.vel.y = kicked_ball.vel().y();
    kicked_ball_msg.vel.z = kicked_ball.vel().z();

    // Start timestamp (required)
    kicked_ball_msg.start_timestamp = kicked_ball.start_timestamp();

    // Stop timestamp (optional)
    if (kicked_ball.has_stop_timestamp()) {
      kicked_ball_msg.stop_timestamp.push_back(kicked_ball.stop_timestamp());
    }

    // Stop position (optional)
    if (kicked_ball.has_stop_pos()) {
      robocup_ssl_msgs::msg::Vector2 stop_pos;
      stop_pos.x = kicked_ball.stop_pos().x();
      stop_pos.y = kicked_ball.stop_pos().y();
      kicked_ball_msg.stop_pos.push_back(stop_pos);
    }

    // Robot ID that kicked the ball (optional)
    if (kicked_ball.has_robot_id()) {
      robocup_ssl_msgs::msg::RobotId robot_id;
      robot_id.id = kicked_ball.robot_id().id();
      robot_id.team = kicked_ball.robot_id().team();
      kicked_ball_msg.robot_id.push_back(robot_id);
    }

    tracked_frame_msg.kicked_ball.push_back(kicked_ball_msg);
  }

  // Parse capabilities
  for (const auto & capability : tracked_frame.capabilities()) {
    tracked_frame_msg.capabilities.push_back(static_cast<uint32_t>(capability));
  }

  return tracked_frame_msg;
}

}  // namespace robocup_ssl_comm