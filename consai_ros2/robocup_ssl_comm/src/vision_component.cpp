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

#include "robocup_ssl_comm/vision_component.hpp"

#include <algorithm>
#include <chrono>
#include <crane_utils/parameter.hpp>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

namespace robocup_ssl_comm
{
namespace
{
constexpr double kMinConfidence = 0.5;

template <typename Detections>
void append_confident(const Detections & detections, Detections & merged)
{
  std::copy_if(
    detections.begin(), detections.end(), std::back_inserter(merged),
    [](const auto & detection) { return detection.confidence > kMinConfidence; });
}

robocup_ssl_msgs::msg::SSLDetectionRobot toRobotMsg(const robocup_ssl::SSL_DetectionRobot & robot)
{
  robocup_ssl_msgs::msg::SSLDetectionRobot robot_msg;
  robot_msg.confidence = robot.confidence();
  if (robot.has_robot_id()) {
    robot_msg.robot_id = robot.robot_id();
  } else {
    robot_msg.robot_id = 100;  // invalid value
  }
  robot_msg.x = robot.x() / 1000.0;
  robot_msg.y = robot.y() / 1000.0;
  if (robot.has_orientation()) {
    robot_msg.orientation = robot.orientation();
  } else {
    robot_msg.orientation = 0.0;  // invalid value
  }
  robot_msg.pixel_x = robot.pixel_x();
  robot_msg.pixel_y = robot.pixel_y();
  if (robot.has_height()) {
    robot_msg.height = robot.height() / 1000.0;
  } else {
    robot_msg.height = 0.0;  // invalid value
  }
  return robot_msg;
}
}  // namespace

Vision::Vision(const rclcpp::NodeOptions & options) : Node("vision", options)
{
  const std::string multicast_address =
    crane::get_or_declare_parameter(this, "multicast_address", "224.5.23.2");
  const int multicast_port = crane::get_or_declare_parameter(this, "multicast_port", 10020);
  publish_interval_ms_ =
    std::chrono::milliseconds(crane::get_or_declare_parameter(this, "publish_interval_ms", 25));
  max_camera_age_ms_ =
    std::chrono::milliseconds(crane::get_or_declare_parameter(this, "max_camera_age_ms", 100));

  receiver = std::make_unique<crane::AsyncUdpReceiver>(
    asio_ctx_.io_context, multicast_address, multicast_port);
  receiver->startReceive([this](const std::vector<char> & buf, size_t size) {
    if (size > 0) {
      robocup_ssl::SSL_WrapperPacket wrapper_packet;
      if (!wrapper_packet.ParseFromArray(buf.data(), static_cast<int>(size))) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Visionパケットのパース失敗 (size=%zu) — protoバージョン不整合の可能性", size);
        return;
      }
      if (wrapper_packet.has_detection()) {
        auto detection_frame_msg = parse_detection_frame(wrapper_packet);
        uint32_t camera_id = detection_frame_msg.camera_id;
        std::lock_guard<std::mutex> lock(frames_mutex_);
        update_camera_frame(camera_id, detection_frame_msg);
      }
    }
  });

  asio_ctx_.start();

  pub_detection_frame =
    create_publisher<robocup_ssl_msgs::msg::SSLDetectionFrame>("detection_frame", 10);

  timer = rclcpp::create_timer(
    this, get_clock(), publish_interval_ms_, std::bind(&Vision::on_timer, this));

  RCLCPP_INFO(
    get_logger(), "Vision component initialized - listening on %s:%d", multicast_address.c_str(),
    multicast_port);
}

void Vision::on_timer()
{
  robocup_ssl_msgs::msg::SSLDetectionFrame merged_frame;
  {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    merged_frame = merge_camera_frames();
  }
  if (
    merged_frame.camera_id != 0 || !merged_frame.balls.empty() ||
    !merged_frame.robots_yellow.empty() || !merged_frame.robots_blue.empty()) {
    pub_detection_frame->publish(
      std::make_unique<robocup_ssl_msgs::msg::SSLDetectionFrame>(std::move(merged_frame)));
  }
}

robocup_ssl_msgs::msg::SSLDetectionFrame Vision::parse_detection_frame(
  const robocup_ssl::SSL_WrapperPacket & wrapper_packet)
{
  robocup_ssl_msgs::msg::SSLDetectionFrame detection_frame_msg;
  const auto & detection_frame = wrapper_packet.detection();

  detection_frame_msg.frame_number = detection_frame.frame_number();
  detection_frame_msg.t_capture = detection_frame.t_capture();
  detection_frame_msg.t_sent = detection_frame.t_sent();
  detection_frame_msg.camera_id = detection_frame.camera_id();

  for (const auto & ball : detection_frame.balls()) {
    robocup_ssl_msgs::msg::SSLDetectionBall ball_msg;
    ball_msg.confidence = ball.confidence();
    if (ball.has_area()) {
      ball_msg.area = ball.area();
    } else {
      ball_msg.area = 100;  // invalid value
    }
    ball_msg.x = ball.x() / 1000.0;
    ball_msg.y = ball.y() / 1000.0;
    if (ball.has_z()) {
      ball_msg.z = ball.z() / 1000.0;
    } else {
      ball_msg.z = 0.0;  // invalid value
    }
    ball_msg.pixel_x = ball.pixel_x();
    ball_msg.pixel_y = ball.pixel_y();

    detection_frame_msg.balls.push_back(ball_msg);
  }

  for (const auto & robot : detection_frame.robots_yellow()) {
    detection_frame_msg.robots_yellow.push_back(toRobotMsg(robot));
  }
  for (const auto & robot : detection_frame.robots_blue()) {
    detection_frame_msg.robots_blue.push_back(toRobotMsg(robot));
  }

  return detection_frame_msg;
}

robocup_ssl_msgs::msg::SSLDetectionFrame Vision::merge_camera_frames()
{
  robocup_ssl_msgs::msg::SSLDetectionFrame merged_frame;

  uint32_t latest_camera_id = 0;
  double latest_t_capture = 0.0;
  double latest_t_sent = 0.0;
  uint32_t latest_frame_number = 0;

  for (const auto & [camera_id, frame] : camera_frames_) {
    if (!is_camera_frame_valid(camera_id)) {
      RCLCPP_DEBUG(get_logger(), "Camera %u frame is too old, skipping", camera_id);
      continue;
    }

    if (frame.t_capture > latest_t_capture) {
      latest_camera_id = camera_id;
      latest_t_capture = frame.t_capture;
      latest_t_sent = frame.t_sent;
      latest_frame_number = frame.frame_number;
    }

    append_confident(frame.balls, merged_frame.balls);
    append_confident(frame.robots_yellow, merged_frame.robots_yellow);
    append_confident(frame.robots_blue, merged_frame.robots_blue);
  }

  merged_frame.camera_id = latest_camera_id;
  merged_frame.t_capture = latest_t_capture;
  merged_frame.t_sent = latest_t_sent;
  merged_frame.frame_number = latest_frame_number;

  RCLCPP_DEBUG(
    get_logger(),
    "Merged frame: %zu balls, %zu yellow robots, %zu blue robots from %zu active cameras",
    merged_frame.balls.size(), merged_frame.robots_yellow.size(), merged_frame.robots_blue.size(),
    camera_frames_.size());

  return merged_frame;
}

void Vision::update_camera_frame(
  uint32_t camera_id, const robocup_ssl_msgs::msg::SSLDetectionFrame & frame)
{
  camera_frames_[camera_id] = frame;
  camera_timestamps_[camera_id] = std::chrono::steady_clock::now();

  RCLCPP_DEBUG(get_logger(), "Updated camera %u frame data", camera_id);
}

bool Vision::is_camera_frame_valid(uint32_t camera_id) const
{
  auto it = camera_timestamps_.find(camera_id);
  if (it == camera_timestamps_.end()) {
    return false;
  }

  auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - it->second);
  return age <= max_camera_age_ms_;
}

}  // namespace robocup_ssl_comm
