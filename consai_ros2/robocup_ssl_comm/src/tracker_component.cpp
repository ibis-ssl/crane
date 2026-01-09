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
  receiver = std::make_unique<crane::MulticastReceiver>(
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
      robocup_ssl::TrackerWrapperPacket wrapper_packet;
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
  const robocup_ssl::TrackerWrapperPacket & wrapper_packet)
{
  robocup_ssl_msgs::msg::TrackedFrame tracked_frame_msg;
  const auto & tracked_frame = wrapper_packet.tracked_frame();

  tracked_frame_msg.frame_number = tracked_frame.frame_number();
  tracked_frame_msg.timestamp = tracked_frame.timestamp();

  for (const auto & ball : tracked_frame.balls()) {
    robocup_ssl_msgs::msg::TrackedBall ball_msg;

    // Position (required)
    ball_msg.pos.x = ball.pos().x();
    ball_msg.pos.y = ball.pos().y();
    ball_msg.pos.z = ball.pos().z();
    ball_msg.has_field |= ball_msg.POS_FIELD_SET;

    // Velocity (optional)
    if (ball.has_vel()) {
      ball_msg.vel.x = ball.vel().x();
      ball_msg.vel.y = ball.vel().y();
      ball_msg.vel.z = ball.vel().z();
      ball_msg.has_field |= ball_msg.VEL_FIELD_SET;
    }

    // Visibility (optional)
    if (ball.has_visibility()) {
      ball_msg.visibility = ball.visibility();
      ball_msg.has_field |= ball_msg.VISIBILITY_FIELD_SET;
    }

    tracked_frame_msg.balls.push_back(ball_msg);
  }

  for (const auto & robot : tracked_frame.robots()) {
    robocup_ssl_msgs::msg::TrackedRobot robot_msg;

    // Robot ID (required)
    robot_msg.robot_id.id = robot.robot_id().id();
    robot_msg.robot_id.team.value = static_cast<int32_t>(robot.robot_id().team());
    robot_msg.robot_id.has_field |=
      robot_msg.robot_id.ID_FIELD_SET | robot_msg.robot_id.TEAM_FIELD_SET;
    robot_msg.has_field |= robot_msg.ROBOT_ID_FIELD_SET;

    // Position and orientation (required)
    robot_msg.pos.x = robot.pos().x();
    robot_msg.pos.y = robot.pos().y();
    robot_msg.orientation = robot.orientation();
    robot_msg.has_field |= robot_msg.POS_FIELD_SET | robot_msg.ORIENTATION_FIELD_SET;

    // Velocity (optional)
    if (robot.has_vel()) {
      robot_msg.vel.x = robot.vel().x();
      robot_msg.vel.y = robot.vel().y();
      robot_msg.has_field |= robot_msg.VEL_FIELD_SET;
    }

    // Angular velocity (optional)
    if (robot.has_vel_angular()) {
      robot_msg.vel_angular = robot.vel_angular();
      robot_msg.has_field |= robot_msg.VEL_ANGULAR_FIELD_SET;
    }

    // Visibility (optional)
    if (robot.has_visibility()) {
      robot_msg.visibility = robot.visibility();
      robot_msg.has_field |= robot_msg.VISIBILITY_FIELD_SET;
    }

    tracked_frame_msg.robots.push_back(robot_msg);
  }

  if (tracked_frame.has_kicked_ball()) {
    robocup_ssl_msgs::msg::KickedBall kicked_ball_msg;
    const auto & kicked_ball = tracked_frame.kicked_ball();

    // Position (required)
    kicked_ball_msg.pos.x = kicked_ball.pos().x();
    kicked_ball_msg.pos.y = kicked_ball.pos().y();
    kicked_ball_msg.has_field |= kicked_ball_msg.POS_FIELD_SET;

    // Initial velocity (required)
    kicked_ball_msg.vel.x = kicked_ball.vel().x();
    kicked_ball_msg.vel.y = kicked_ball.vel().y();
    kicked_ball_msg.vel.z = kicked_ball.vel().z();
    kicked_ball_msg.has_field |= kicked_ball_msg.VEL_FIELD_SET;

    // Start timestamp (required)
    kicked_ball_msg.start_timestamp = kicked_ball.start_timestamp();
    kicked_ball_msg.has_field |= kicked_ball_msg.START_TIMESTAMP_FIELD_SET;

    // Stop timestamp (optional)
    if (kicked_ball.has_stop_timestamp()) {
      kicked_ball_msg.stop_timestamp = kicked_ball.stop_timestamp();
      kicked_ball_msg.has_field |= kicked_ball_msg.STOP_TIMESTAMP_FIELD_SET;
    }

    // Stop position (optional)
    if (kicked_ball.has_stop_pos()) {
      kicked_ball_msg.stop_pos.x = kicked_ball.stop_pos().x();
      kicked_ball_msg.stop_pos.y = kicked_ball.stop_pos().y();
      kicked_ball_msg.has_field |= kicked_ball_msg.STOP_POS_FIELD_SET;
    }

    // Robot ID that kicked the ball (optional)
    if (kicked_ball.has_robot_id()) {
      kicked_ball_msg.robot_id.id = kicked_ball.robot_id().id();
      kicked_ball_msg.robot_id.team.value = static_cast<int32_t>(kicked_ball.robot_id().team());
      kicked_ball_msg.robot_id.has_field |=
        kicked_ball_msg.robot_id.ID_FIELD_SET | kicked_ball_msg.robot_id.TEAM_FIELD_SET;
      kicked_ball_msg.has_field |= kicked_ball_msg.ROBOT_ID_FIELD_SET;
    }

    tracked_frame_msg.kicked_ball = kicked_ball_msg;
    tracked_frame_msg.has_field |= tracked_frame_msg.KICKED_BALL_FIELD_SET;
  }

  for (const auto & capability : tracked_frame.capabilities()) {
    robocup_ssl_msgs::msg::Capability capability_msg;
    capability_msg.value = static_cast<int32_t>(capability);
    tracked_frame_msg.capabilities.push_back(capability_msg);
  }

  // Set frame metadata
  tracked_frame_msg.has_field |=
    tracked_frame_msg.FRAME_NUMBER_FIELD_SET | tracked_frame_msg.TIMESTAMP_FIELD_SET;

  return tracked_frame_msg;
}

}  // namespace robocup_ssl_comm
