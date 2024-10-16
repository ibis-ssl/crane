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

#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/detection_frame.hpp>
#include <robocup_ssl_msgs/msg/geometry_data.hpp>

#include "multicast.hpp"
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
  void publish_detection(const SSL_DetectionFrame & detection_frame);

  void publish_geometry(const SSL_GeometryData & geometry_data);

  void set_geometry_field_size(
    robocup_ssl_msgs::msg::GeometryFieldSize & msg_field, const SSL_GeometryFieldSize & data_field);

  robocup_ssl_msgs::msg::GeometryCameraCalibration parse_calib(
    const SSL_GeometryCameraCalibration & data_calib);

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<multicast::MulticastReceiver> receiver;

  rclcpp::Publisher<robocup_ssl_msgs::msg::DetectionFrame>::SharedPtr pub_detection;

  rclcpp::Publisher<robocup_ssl_msgs::msg::GeometryData>::SharedPtr pub_geometry;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__VISION_COMPONENT_HPP_
