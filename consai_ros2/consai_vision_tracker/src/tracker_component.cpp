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

#include "consai_vision_tracker/tracker_component.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace consai_vision_tracker
{
using RobotId = robocup_ssl_msgs::msg::RobotId;
using std::placeholders::_1;

Tracker::Tracker(const rclcpp::NodeOptions & options) : Node("tracker", options)
{
  using std::chrono::operator""s;
  const auto UPDATE_RATE = 0.01s;

  ball_tracker = std::make_shared<BallTracker>(UPDATE_RATE.count());
  for (int i = 0; i < 16; i++) {
    blue_robot_tracker.push_back(
      std::make_shared<RobotTracker>(RobotId::TEAM_COLOR_BLUE, i, UPDATE_RATE.count()));
    yellow_robot_tracker.push_back(
      std::make_shared<RobotTracker>(RobotId::TEAM_COLOR_YELLOW, i, UPDATE_RATE.count()));
  }

  timer = rclcpp::create_timer(this, get_clock(), UPDATE_RATE, std::bind(&Tracker::on_timer, this));

  pub_tracked = create_publisher<TrackedFrame>("detection_tracked", 10);

  vis_data_handler_ = std::make_shared<VisualizationDataHandler>(*this);

  declare_parameter("invert", false);

  if (get_parameter("invert").get_value<bool>()) {
    sub_detection = create_subscription<DetectionFrame>(
      "detection", 10, std::bind(&Tracker::callback_detection_invert, this, _1));
  } else {
    sub_detection = create_subscription<DetectionFrame>(
      "detection", 10, std::bind(&Tracker::callback_detection, this, _1));
  }

  sub_geometry_ = create_subscription<GeometryData>(
    "geometry", 10, std::bind(&Tracker::callback_geometry, this, _1));
}

void Tracker::on_timer()
{
  auto tracked_msg = std::make_unique<TrackedFrame>();

  tracked_msg->balls.push_back(ball_tracker->update());

  for (auto && tracker : blue_robot_tracker) {
    tracked_msg->robots.push_back(tracker->update());
  }

  for (auto && tracker : yellow_robot_tracker) {
    tracked_msg->robots.push_back(tracker->update());
  }

  tracked_msg = vis_data_handler_->publish_vis_tracked(std::move(tracked_msg));

  pub_tracked->publish(std::move(tracked_msg));
}

void Tracker::callback_detection(const DetectionFrame::SharedPtr msg)
{
  for (const auto & ball : msg->balls) {
    ball_tracker->push_back_observation(ball);
  }

  for (const auto & blue_robot : msg->robots_blue) {
    if (blue_robot.robot_id.size() > 0) {
      blue_robot_tracker[blue_robot.robot_id[0]]->push_back_observation(blue_robot);
    }
  }

  for (const auto & yellow_robot : msg->robots_yellow) {
    if (yellow_robot.robot_id.size() > 0) {
      yellow_robot_tracker[yellow_robot.robot_id[0]]->push_back_observation(yellow_robot);
    }
  }

  vis_data_handler_->publish_vis_detection(msg);
}

void Tracker::callback_detection_invert(const DetectionFrame::SharedPtr msg)
{
  // detectionトピックの情報を上下左右反転するcallback関数

  for (auto && ball : msg->balls) {
    invert_ball(ball);
    ball_tracker->push_back_observation(ball);
  }

  for (auto && blue_robot : msg->robots_blue) {
    if (blue_robot.robot_id.size() > 0) {
      invert_robot(blue_robot);
      blue_robot_tracker[blue_robot.robot_id[0]]->push_back_observation(blue_robot);
    }
  }

  for (auto && yellow_robot : msg->robots_yellow) {
    if (yellow_robot.robot_id.size() > 0) {
      invert_robot(yellow_robot);
      yellow_robot_tracker[yellow_robot.robot_id[0]]->push_back_observation(yellow_robot);
    }
  }

  vis_data_handler_->publish_vis_detection(msg);
}

void Tracker::callback_geometry(const GeometryData::SharedPtr msg)
{
  vis_data_handler_->publish_vis_geometry(msg);
}

void Tracker::invert_ball(DetectionBall & ball)
{
  // detection ballを上下左右反転する
  ball.x = -ball.x;
  ball.y = -ball.y;
  ball.pixel_x = -ball.pixel_x;
  ball.pixel_y = -ball.pixel_y;
}

void Tracker::invert_robot(DetectionRobot & robot)
{
  // detection robotを上下左右反転する
  robot.x = -robot.x;
  robot.y = -robot.y;
  robot.pixel_x = -robot.pixel_x;
  robot.pixel_y = -robot.pixel_y;
  if (robot.orientation.size() > 0) {
    robot.orientation[0] -= std::copysign(M_PI, robot.orientation[0]);
  }
}

}  // namespace consai_vision_tracker

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(consai_vision_tracker::Tracker)
