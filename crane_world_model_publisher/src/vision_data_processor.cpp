// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/vision_data_processor.hpp"

#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <string>
#include <vector>

namespace crane
{
VisionDataProcessor::VisionDataProcessor(rclcpp::Node & node) : node_(node)
{
  node_.declare_parameter("vision_address", "224.5.23.2");
  node_.declare_parameter("vision_port", 10020);
  vision_receiver_ = std::make_unique<multicast::MulticastReceiver>(
    node_.get_parameter("vision_address").get_value<std::string>(),
    node_.get_parameter("vision_port").get_value<int>());

  ball_tracker_manager_ = std::make_unique<BallTrackerManager>();
  last_prediction_time_ = rclcpp::Clock().now();

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

auto VisionDataProcessor::processVisionPackets() -> void
{
  auto current_time = rclcpp::Clock().now();
  double dt = (current_time - last_prediction_time_).seconds();
  
  if (dt > 0.0) {
    ball_tracker_manager_->predict(dt);
    ball_tracker_manager_->removeOldTrackers();
    last_prediction_time_ = current_time;
  }

  while (vision_receiver_->available()) {
    has_vision_updated_ = true;
    std::vector<char> buf(2048);
    const size_t size = vision_receiver_->receive(buf);

    if (size > 0) {
      SSL_WrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));
      if (packet.has_geometry()) {
        visionGeometryCallback(packet.geometry());
      }
      if (packet.has_detection()) {
        visionDetectionCallback(packet.detection());
      }
    }
  }
}

auto VisionDataProcessor::visionGeometryCallback(const SSL_GeometryData & geometry_data) -> void
{
  field_height_ = geometry_data.field().field_width() / 1000.;
  field_width_ = geometry_data.field().field_length() / 1000.;

  goal_height_ = geometry_data.field().goal_depth() / 1000.;
  goal_width_ = geometry_data.field().goal_width() / 1000.;

  if (geometry_data.field().has_penalty_area_depth()) {
    penalty_area_height_ = geometry_data.field().penalty_area_depth() / 1000.;
  } else {
    penalty_area_height_ = goal_width_;
  }

  if (geometry_data.field().has_penalty_area_width()) {
    penalty_area_width_ = geometry_data.field().penalty_area_width() / 1000.;
  } else {
    penalty_area_width_ = goal_width_ * 2.;
  }

  if (geometry_vis_handler_) {
    geometry_vis_handler_(geometry_data, false);
  }
}

auto VisionDataProcessor::visionDetectionCallback(const SSL_DetectionFrame & detection_frame)
  -> void
{
  int balls_size = detection_frame.balls().size();
  auto now = node_.now();
  
  if (balls_size > 0) {
    last_ball_detect_time_ = now;
    
    Eigen::Vector3d ball_position;
    ball_position(0) = detection_frame.balls().at(0).x() * 0.001;
    ball_position(1) = detection_frame.balls().at(0).y() * 0.001;
    ball_position(2) = detection_frame.balls().at(0).has_z() ? 
                       detection_frame.balls().at(0).z() * 0.001 : 0.0;
    
    // 状態推定はBallTrackerManagerに委譲、データの受け渡しのみ
    ball_info_ = ball_tracker_manager_->processVisionDetection(ball_position, now);
    
    ball_info_.vision.stamp = now;
    ball_info_.vision.pos.x = ball_position(0);
    ball_info_.vision.pos.y = ball_position(1);
    ball_info_.vision.pos.z = ball_position(2);
  } else {
    if (
      now.get_clock_type() == last_ball_detect_time_.get_clock_type() &&
      (now - last_ball_detect_time_).seconds() > 0.1) {
      ball_info_.detected = false;
    } else {
      auto best_tracker = ball_tracker_manager_->getBestTracker();
      if (best_tracker) {
        ball_info_ = best_tracker->getState();
      }
    }
  }

  for (const auto & robot : detection_frame.robots_yellow()) {
    if (robot.has_robot_id()) {
      auto & each_robot_info = robot_info_[static_cast<int>(Color::YELLOW)].at(robot.robot_id());
      each_robot_info.vision.pose.x = robot.x() * 0.001;
      each_robot_info.vision.pose.y = robot.y() * 0.001;
      each_robot_info.vision.pose.theta = robot.orientation();
      each_robot_info.vision.stamp = now;
    }
  }

  for (const auto & robot : detection_frame.robots_blue()) {
    if (robot.has_robot_id()) {
      auto & each_robot_info = robot_info_[static_cast<int>(Color::BLUE)].at(robot.robot_id());
      each_robot_info.vision.pose.x = robot.x() * 0.001;
      each_robot_info.vision.pose.y = robot.y() * 0.001;
      each_robot_info.vision.pose.theta = robot.orientation();
      each_robot_info.vision.stamp = now;
    }
  }
}
}  // namespace crane