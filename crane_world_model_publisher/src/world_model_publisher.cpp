// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "crane_world_model_publisher/world_model_publisher.hpp"

namespace crane
{

WorldModelPublisherComponent::WorldModelPublisherComponent(const rclcpp::NodeOptions & options) : rclcpp::Node("world_model_publisher", options)
{
  sub_vision_ = this->create_subscription<robocup_ssl_msgs::msg::TrackedFrame>(
    "/detection_tracked", 1,
    std::bind(
      &WorldModelPublisherComponent::visionDetectionsCallback, this, std::placeholders::_1));
  sub_geometry_ = this->create_subscription<robocup_ssl_msgs::msg::GeometryData>(
    "/geometry", 1,
    std::bind(&WorldModelPublisherComponent::visionGeometryCallback, this, std::placeholders::_1));

  pub_world_model_ = create_publisher<crane_msgs::msg::WorldModel>("/world_model", 1);

  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(
    10ms, std::bind(&WorldModelPublisherComponent::publishWorldModel, this));
  max_id = 16;
  robot_info_[static_cast<int>(Color::BLUE)].resize(max_id);
  robot_info_[static_cast<int>(Color::YELLOW)].resize(max_id);

  // TODO(HansRobo) : input our_color & their_color from param
  our_color = Color::BLUE;
  their_color = Color::YELLOW;
}
void WorldModelPublisherComponent::visionDetectionsCallback(
  const robocup_ssl_msgs::msg::TrackedFrame::SharedPtr msg)
{
  robot_info_[0].clear();
  robot_info_[1].clear();
  for (auto & robot : msg->robots) {
    crane_msgs::msg::RobotInfo robot_info;
    if (!robot.visibility.empty()) {
      robot_info.detected = (robot.visibility.front() > 0.5);
    } else {
      robot_info.detected = false;
    }

    robot_info.robot_id = robot.robot_id.id;
    robot_info.pose.x = robot.pos.x;
    robot_info.pose.y = robot.pos.y;
    robot_info.pose.theta = robot.orientation;
    if (!robot.vel.empty()) {
      robot_info.velocity.x = robot.vel.front().x;
      robot_info.velocity.y = robot.vel.front().y;
    } else {
      // calc from diff
    }
    if (!robot.vel_angular.empty()) {
      robot_info.velocity.theta = robot.vel_angular.front();
    } else {
      // calc from diff
    }

    int team_index =
      (robot.robot_id.team_color == robocup_ssl_msgs::msg::RobotId::TEAM_COLOR_YELLOW)
        ? static_cast<int>(Color::YELLOW)
        : static_cast<int>(Color::BLUE);
    robot_info_[team_index].push_back(robot_info);
  }

  if (!msg->balls.empty()) {
    auto ball_msg = msg->balls.front();
    ball_info_.pose.x = ball_msg.pos.x;
    ball_info_.pose.y = ball_msg.pos.y;

    ball_info_.velocity.x = ball_msg.pos.x;
    ball_info_.velocity.y = ball_msg.pos.y;

    ball_info_.detected = true;
    ball_info_.detection_time = msg->timestamp;
    ball_info_.disappeared = false;
  } else {
    ball_info_.detected = false;
  }
}

void WorldModelPublisherComponent::visionGeometryCallback(
  const robocup_ssl_msgs::msg::GeometryData::SharedPtr msg)
{
  field_h_ = msg->field.field_width/1000.;
  field_w_ = msg->field.field_length/1000.;

  // msg->goal_width
  // msg->goal_depth
  // msg->boundary_width
  // msg->field_lines
  // msg->field_arcs
}

void WorldModelPublisherComponent::publishWorldModel()
{
  crane_msgs::msg::WorldModel wm;
  wm.ball_info = ball_info_;

  for (auto robot : robot_info_[static_cast<uint8_t>(our_color)]) {
    crane_msgs::msg::RobotInfoOurs info;
    info.id = robot.robot_id;
    info.disappeared = robot.disappeared;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    wm.robot_info_ours.emplace_back(info);
  }
  for (auto robot : robot_info_[static_cast<uint8_t>(their_color)]) {
    crane_msgs::msg::RobotInfoTheirs info;
    info.id = robot.robot_id;
    info.disappeared = robot.disappeared;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    wm.robot_info_theirs.emplace_back(info);
  }

  wm.field_info.x = field_w_;
  wm.field_info.y = field_h_;
  pub_world_model_->publish(wm);
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
