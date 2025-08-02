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

#ifndef ROBOCUP_SSL_COMM__VISION_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__VISION_COMPONENT_HPP_

#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <chrono>
#include <crane_comm/multicast.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/detection_frame.hpp>

#include "visibility_control.h"

namespace robocup_ssl_comm
{
class Vision : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit Vision(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  robocup_ssl_msgs::msg::DetectionFrame parse_detection_frame(
    const SSL_WrapperPacket & wrapper_packet);
  
  robocup_ssl_msgs::msg::DetectionFrame merge_camera_frames();

  void update_camera_frame(uint32_t camera_id, const robocup_ssl_msgs::msg::DetectionFrame & frame);

  bool is_camera_frame_valid(uint32_t camera_id, std::chrono::milliseconds max_age_ms = std::chrono::milliseconds(100));

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<multicast::MulticastReceiver> receiver;

  rclcpp::Publisher<robocup_ssl_msgs::msg::DetectionFrame>::SharedPtr pub_detection_frame;

  // カメラ別の最新フレームデータを保存
  std::map<uint32_t, robocup_ssl_msgs::msg::DetectionFrame> camera_frames_;
  
  // カメラ別のタイムスタンプを保存（フレームの有効性確認用）
  std::map<uint32_t, std::chrono::steady_clock::time_point> camera_timestamps_;

  // 統合フレームの生成頻度（ミリ秒）
  std::chrono::milliseconds publish_interval_ms_;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__VISION_COMPONENT_HPP_