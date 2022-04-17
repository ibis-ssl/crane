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

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "boost/range/adaptor/indexed.hpp"
#include "crane_msgs/msg/ball_info.hpp"
#include "crane_msgs/msg/robot_info.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"

enum class Color : uint8_t {
  BLUE,
  YELLOW,
};

class WorldModelPublisherComponent : public rclcpp::Node
{
public:
  WorldModelPublisherComponent() : Node("world_model_publisher")
  {
    sub_vision_ = this->create_subscription<robocup_ssl_msgs::msg::TrackedFrame>(
      "detection_tracked", 1,
      std::bind(
        &WorldModelPublisherComponent::visionDetectionsCallback, this, std::placeholders::_1));
    sub_geometry_ = this->create_subscription<robocup_ssl_msgs::msg::GeometryData>(
      "geometry", 1,
      std::bind(
        &WorldModelPublisherComponent::visionGeometryCallback, this, std::placeholders::_1));
    pub_ball_ = this->create_publisher<crane_msgs::msg::BallInfo>("~/ball_info", 1);
    pub_robot_[static_cast<uint8_t>(Color::BLUE)] =
      this->create_publisher<crane_msgs::msg::RobotInfo>("~/robot_info_blue", 1);
    pub_robot_[static_cast<uint8_t>(Color::YELLOW)] =
      this->create_publisher<crane_msgs::msg::RobotInfo>("~/robot_info_yellow", 1);

    pub_world_model_ = create_publisher<crane_msgs::msg::WorldModel>("~/world_model", 1);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
      10ms, std::bind(&WorldModelPublisherComponent::publishWorldModel, this));
    max_id = 7 + 1;
    robot_info_[static_cast<int>(Color::BLUE)].resize(max_id);
    robot_info_[static_cast<int>(Color::YELLOW)].resize(max_id);

    enable_publish_ball = true;
    enable_publish_robot[static_cast<uint8_t>(Color::BLUE)] = true;
    enable_publish_robot[static_cast<uint8_t>(Color::YELLOW)] = true;

    // TODO(HansRobo) : input our_color & their_color from param
    our_color = Color::BLUE;
    their_color = Color::YELLOW;
  }

  void visionDetectionsCallback(const robocup_ssl_msgs::msg::TrackedFrame::SharedPtr msg)
  {
    robot_info_[0].clear();
    robot_info_[1].clear();
    for (auto & robot : msg->robots) {
      crane_msgs::msg::RobotInfo robot_info;
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

  void visionGeometryCallback(const robocup_ssl_msgs::msg::GeometryData::SharedPtr msg)
  {
    field_h_ = msg->field.field_width;
    field_w_ = msg->field.field_length;

    // msg->goal_width
    // msg->goal_depth
    // msg->boundary_width
    // msg->field_lines
    // msg->field_arcs
  }

private:
  void publishWorldModel()
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

  bool enable_publish_ball;
  bool enable_publish_robot[2];
  Color our_color;
  Color their_color;
  uint8_t max_id;
  static constexpr float DISAPPEARED_TIME_THRESH_ = 3.0f;
  float field_w_, field_h_;

  crane_msgs::msg::BallInfo ball_info_;
  std::vector<crane_msgs::msg::RobotInfo> robot_info_[2];
  rclcpp::Subscription<robocup_ssl_msgs::msg::TrackedFrame>::SharedPtr sub_vision_;
  rclcpp::Subscription<robocup_ssl_msgs::msg::GeometryData>::SharedPtr sub_geometry_;
  rclcpp::Publisher<crane_msgs::msg::BallInfo>::SharedPtr pub_ball_;
  rclcpp::Publisher<crane_msgs::msg::RobotInfo>::SharedPtr pub_robot_[2];
  // rclcpp::Publisher<crane_msgs::msg::RobotInfoTheirs>::SharedPtr pub_robot_theirs;
  rclcpp::Publisher<crane_msgs::msg::WorldModel>::SharedPtr pub_world_model_;
  rclcpp::TimerBase::SharedPtr timer_;
};

WorldModelPublisherComponent::SharedPtr node = nullptr;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<WorldModelPublisherComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
