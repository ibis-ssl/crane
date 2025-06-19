// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/tracker_data_processor.hpp"

#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>

#include <crane_msgs/msg/robot_info.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <string>
#include <vector>

namespace crane
{
TrackerDataProcessor::TrackerDataProcessor(rclcpp::Node & node) : node_(node)
{
  node_.declare_parameter("tracker_address", "224.5.23.2");
  node_.declare_parameter("tracker_port", 11010);
  tracker_receiver_ = std::make_unique<multicast::MulticastReceiver>(
    node_.get_parameter("tracker_address").get_value<std::string>(),
    node_.get_parameter("tracker_port").get_value<int>());

  area_mask_.min_corner() << -20., -10.;
  area_mask_.max_corner() << 20., 10.;

  for (int i = 0; i < 20; i++) {
    crane_msgs::msg::RobotInfo info;
    info.vision_detected = false;
    info.feedback_detected = false;
    info.detected = false;
    info.id = i;
    robot_info_[0].emplace_back(info);
    robot_info_[1].emplace_back(info);
  }
}

auto TrackerDataProcessor::processTrackerPackets() -> void
{
  while (tracker_receiver_->available()) {
    has_tracker_updated_ = true;
    std::vector<char> buf(2048);
    const size_t size = tracker_receiver_->receive(buf);

    if (size > 0) {
      TrackerWrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if (packet.has_tracked_frame()) {
        trackerCallback(packet.tracked_frame());
      }
    }
  }
}

auto TrackerDataProcessor::trackerCallback(const TrackedFrame & tracked_frame) -> void
{
  rclcpp::Time current_time = node_.now();
  for (auto & robot : robot_info_[0]) {
    robot.vision_detected = false;
  }
  for (auto & robot : robot_info_[1]) {
    robot.vision_detected = false;
  }

  for (const auto & robot : tracked_frame.robots()) {
    int team_index = (robot.robot_id().team() == robocup_ssl_msgs::msg::RobotId::TEAM_COLOR_YELLOW)
                       ? static_cast<int>(Color::YELLOW)
                       : static_cast<int>(Color::BLUE);

    auto & each_robot_info = robot_info_[team_index].at(robot.robot_id().id());
    if (robot.has_visibility()) {
      each_robot_info.vision_detected = (robot.visibility() > 0.2);
    } else {
      each_robot_info.vision_detected = false;
    }

    auto last_frame_stamp = each_robot_info.last_tracker_detection_stamp;

    each_robot_info.pose.x = robot.pos().x();
    each_robot_info.pose.y = robot.pos().y();
    each_robot_info.pose.theta = robot.orientation();
    each_robot_info.last_tracker_detection_stamp = current_time;
    if (robot.has_vel()) {
      auto previous_velocity = each_robot_info.velocity;
      each_robot_info.velocity.x = robot.vel().x();
      each_robot_info.velocity.y = robot.vel().y();
      each_robot_info.velocity_norm =
        std::hypot(each_robot_info.velocity.x, each_robot_info.velocity.y);

      if (double dt = (current_time - last_frame_stamp).seconds(); dt > 0) {
        each_robot_info.acceleration.x = (each_robot_info.velocity.x - previous_velocity.x) / dt;
        each_robot_info.acceleration.y = (each_robot_info.velocity.y - previous_velocity.y) / dt;

        each_robot_info.acceleration_norm =
          std::hypot(each_robot_info.acceleration.x, each_robot_info.acceleration.y);
      }
    } else {
      each_robot_info.acceleration.x = 0.0;
      each_robot_info.acceleration.y = 0.0;
      each_robot_info.acceleration_norm = 0.0;
    }
    if (robot.has_vel_angular()) {
      each_robot_info.velocity.theta = robot.vel_angular();
    } else {
    }
  }

  if (not tracked_frame.balls().empty()) {
    auto ball = [&]() {
      for (const auto & tracked_ball : tracked_frame.balls()) {
        Eigen::Vector3d position{tracked_ball.pos().x(), tracked_ball.pos().y(), 1.0};
        Eigen::Vector3d transformed_pos = transform_matrix_ * position;
        if (isInBox(area_mask_, {transformed_pos.x(), transformed_pos.y()})) {
          return tracked_ball;
        }
      }
      return *tracked_frame.balls().begin();
    }();

    ball_info_.position.x = ball.pos().x();
    ball_info_.position.y = ball.pos().y();
    ball_info_.position.z = ball.pos().z();

    if (ball.has_vel()) {
      ball_info_.velocity.x = ball.vel().x();
      ball_info_.velocity.y = ball.vel().y();
      ball_info_.velocity.z = ball.vel().z();
      ball_info_.velocity_norm = std::hypot(ball_info_.velocity.x, ball_info_.velocity.y);
    }

    double ball_height = ball.pos().z();
    double ball_speed = std::hypot(ball_info_.velocity.x, ball_info_.velocity.y);

    if (ball_height > 0.05) {
      ball_info_.state = 2;
    } else if (ball_speed > 0.1) {
      ball_info_.state = 1;
    } else {
      ball_info_.state = 0;
    }

    ball_info_.deceleration = 0.5f;
    ball_info_.gravity = -9.81f;
    ball_info_.air_resistance = 0.0f;
  }
}
}  // namespace crane
