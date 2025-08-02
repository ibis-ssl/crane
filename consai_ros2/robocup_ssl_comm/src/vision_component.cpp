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

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
Vision::Vision(const rclcpp::NodeOptions & options) : Node("vision", options)
{
  declare_parameter("multicast_address", "224.5.23.2");
  declare_parameter("multicast_port", 10020);
  declare_parameter("publish_interval_ms", 25);
  declare_parameter("max_camera_age_ms", 100);

  publish_interval_ms_ =
    std::chrono::milliseconds(get_parameter("publish_interval_ms").get_value<int>());

  receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("multicast_address").get_value<std::string>(),
    get_parameter("multicast_port").get_value<int>());

  pub_detection_frame =
    create_publisher<robocup_ssl_msgs::msg::DetectionFrame>("detection_frame", 10);

  timer = rclcpp::create_timer(
    this, get_clock(), publish_interval_ms_, std::bind(&Vision::on_timer, this));

  RCLCPP_INFO(
    get_logger(), "Vision component initialized - listening on %s:%d",
    get_parameter("multicast_address").get_value<std::string>().c_str(),
    get_parameter("multicast_port").get_value<int>());
}

void Vision::on_timer()
{
  // SSL-Visionパケットを受信し、カメラ別フレームを更新
  while (receiver->available()) {
    std::vector<char> buf(2048);
    const size_t size = receiver->receive(buf);

    if (size > 0) {
      SSL_WrapperPacket wrapper_packet;
      wrapper_packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if (wrapper_packet.has_detection()) {
        auto detection_frame_msg = parse_detection_frame(wrapper_packet);
        uint32_t camera_id = detection_frame_msg.camera_id;

        // カメラ別フレームを更新
        update_camera_frame(camera_id, detection_frame_msg);

        RCLCPP_DEBUG(
          get_logger(),
          "Received detection frame from camera %u with %zu balls, %zu yellow robots, %zu blue "
          "robots",
          camera_id, detection_frame_msg.balls.size(), detection_frame_msg.robots_yellow.size(),
          detection_frame_msg.robots_blue.size());
      }
    }
  }

  // 全カメラのデータをマージして統合フレームをパブリッシュ
  auto merged_frame = merge_camera_frames();
  if (
    merged_frame.camera_id != 0 || !merged_frame.balls.empty() ||
    !merged_frame.robots_yellow.empty() || !merged_frame.robots_blue.empty()) {
    auto merged_frame_msg = std::make_unique<robocup_ssl_msgs::msg::DetectionFrame>();
    *merged_frame_msg = std::move(merged_frame);
    pub_detection_frame->publish(std::move(merged_frame_msg));
  }
}

robocup_ssl_msgs::msg::DetectionFrame Vision::parse_detection_frame(
  const SSL_WrapperPacket & wrapper_packet)
{
  robocup_ssl_msgs::msg::DetectionFrame detection_frame_msg;
  const auto & detection_frame = wrapper_packet.detection();

  detection_frame_msg.frame_number = detection_frame.frame_number();
  detection_frame_msg.t_capture = detection_frame.t_capture();
  detection_frame_msg.t_sent = detection_frame.t_sent();
  detection_frame_msg.camera_id = detection_frame.camera_id();

  // Parse balls
  for (const auto & ball : detection_frame.balls()) {
    robocup_ssl_msgs::msg::DetectionBall ball_msg;
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

  // Parse yellow robots
  for (const auto & robot : detection_frame.robots_yellow()) {
    robocup_ssl_msgs::msg::DetectionRobot robot_msg;
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

    detection_frame_msg.robots_yellow.push_back(robot_msg);
  }

  // Parse blue robots
  for (const auto & robot : detection_frame.robots_blue()) {
    robocup_ssl_msgs::msg::DetectionRobot robot_msg;
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

    detection_frame_msg.robots_blue.push_back(robot_msg);
  }

  return detection_frame_msg;
}

robocup_ssl_msgs::msg::DetectionFrame Vision::merge_camera_frames()
{
  robocup_ssl_msgs::msg::DetectionFrame merged_frame;

  // 統合フレームのメタデータを設定（最新のカメラデータから取得）
  uint32_t latest_camera_id = 0;
  double latest_t_capture = 0.0;
  double latest_t_sent = 0.0;
  uint32_t latest_frame_number = 0;

  auto max_age_ms = std::chrono::milliseconds(get_parameter("max_camera_age_ms").get_value<int>());

  // 有効な全カメラのデータを統合
  for (const auto & [camera_id, frame] : camera_frames_) {
    if (!is_camera_frame_valid(camera_id, max_age_ms)) {
      RCLCPP_DEBUG(get_logger(), "Camera %u frame is too old, skipping", camera_id);
      continue;
    }

    // 最新のフレーム情報を記録
    if (frame.t_capture > latest_t_capture) {
      latest_camera_id = camera_id;
      latest_t_capture = frame.t_capture;
      latest_t_sent = frame.t_sent;
      latest_frame_number = frame.frame_number;
    }

    // ボールデータをマージ（confidenceでフィルタリング）
    for (const auto & ball : frame.balls) {
      if (ball.confidence > 0.5) {  // confidenceの閾値
        merged_frame.balls.push_back(ball);
      }
    }

    // 黄色ロボットデータをマージ
    for (const auto & robot : frame.robots_yellow) {
      if (robot.confidence > 0.5) {  // confidenceの閾値
        merged_frame.robots_yellow.push_back(robot);
      }
    }

    // 青色ロボットデータをマージ
    for (const auto & robot : frame.robots_blue) {
      if (robot.confidence > 0.5) {  // confidenceの閾値
        merged_frame.robots_blue.push_back(robot);
      }
    }
  }

  // 統合フレームのメタデータを設定
  merged_frame.camera_id =
    latest_camera_id;  // 最新のカメラID（統合フレームであることを示すため0にすることも可能）
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
  uint32_t camera_id, const robocup_ssl_msgs::msg::DetectionFrame & frame)
{
  camera_frames_[camera_id] = frame;
  camera_timestamps_[camera_id] = std::chrono::steady_clock::now();

  RCLCPP_DEBUG(get_logger(), "Updated camera %u frame data", camera_id);
}

bool Vision::is_camera_frame_valid(uint32_t camera_id, std::chrono::milliseconds max_age_ms)
{
  auto it = camera_timestamps_.find(camera_id);
  if (it == camera_timestamps_.end()) {
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second);

  return age <= max_age_ms;
}

}  // namespace robocup_ssl_comm
