// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_publisher.hpp"

namespace crane
{
WorldModelPublisherComponent::WorldModelPublisherComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("world_model_publisher", options)
{
  sub_vision = this->create_subscription<robocup_ssl_msgs::msg::TrackedFrame>(
    "/detection_tracked", 1,
    std::bind(
      &WorldModelPublisherComponent::visionDetectionsCallback, this, std::placeholders::_1));
  sub_geometry = this->create_subscription<robocup_ssl_msgs::msg::GeometryData>(
    "/geometry", 1,
    std::bind(&WorldModelPublisherComponent::visionGeometryCallback, this, std::placeholders::_1));

  pub_world_model = create_publisher<crane_msgs::msg::WorldModel>("/world_model", 1);

  using namespace std::chrono_literals;
  timer = this->create_wall_timer(
    16ms, std::bind(&WorldModelPublisherComponent::publishWorldModel, this));
  max_id = 16;
  robot_info[static_cast<int>(Color::BLUE)].resize(max_id);
  robot_info[static_cast<int>(Color::YELLOW)].resize(max_id);

  declare_parameter("team_name", "ibis-ssl");
  team_name = get_parameter("team_name").as_string();

  sub_referee = this->create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == team_name) {
        our_color = Color::YELLOW;
        their_color = Color::BLUE;
      } else if (msg.blue.name == team_name) {
        our_color = Color::BLUE;
        their_color = Color::YELLOW;
      }else{
        std::stringstream what;
        what << "Cannot find our team name, " << team_name << " in referee message. ";
        what << "blue team name: " << msg.blue.name << ", yellow team name: " << msg.yellow.name;
        throw std::runtime_error(what.str());
      }
    });

  declare_parameter("initial_team_color", "BLUE");
  auto initial_team_color = get_parameter("initial_team_color").as_string();
  if (initial_team_color == "BLUE") {
    our_color = Color::BLUE;
    their_color = Color::YELLOW;
  } else {
    our_color = Color::YELLOW;
    their_color = Color::BLUE;
  }
}
void WorldModelPublisherComponent::visionDetectionsCallback(
  const robocup_ssl_msgs::msg::TrackedFrame::SharedPtr msg)
{
  robot_info[0].clear();
  robot_info[1].clear();
  for (auto & robot : msg->robots) {
    crane_msgs::msg::RobotInfo each_robot_info;
    if (!robot.visibility.empty()) {
      each_robot_info.detected = (robot.visibility.front() > 0.5);
    } else {
      each_robot_info.detected = false;
    }

    each_robot_info.robot_id = robot.robot_id.id;
    each_robot_info.pose.x = robot.pos.x;
    each_robot_info.pose.y = robot.pos.y;
    each_robot_info.pose.theta = robot.orientation;
    if (!robot.vel.empty()) {
      each_robot_info.velocity.x = robot.vel.front().x;
      each_robot_info.velocity.y = robot.vel.front().y;
    } else {
      // calc from diff
    }
    if (!robot.vel_angular.empty()) {
      each_robot_info.velocity.theta = robot.vel_angular.front();
    } else {
      // calc from diff
    }

    int team_index =
      (robot.robot_id.team_color == robocup_ssl_msgs::msg::RobotId::TEAM_COLOR_YELLOW)
        ? static_cast<int>(Color::YELLOW)
        : static_cast<int>(Color::BLUE);

    robot_info[team_index].push_back(each_robot_info);
  }

  if (!msg->balls.empty()) {
    auto ball_msg = msg->balls.front();
    ball_info.pose.x = ball_msg.pos.x;
    ball_info.pose.y = ball_msg.pos.y;

    ball_info.velocity.x = ball_msg.pos.x;
    ball_info.velocity.y = ball_msg.pos.y;

    ball_info.detected = true;
    ball_info.detection_time = msg->timestamp;
    ball_info.disappeared = false;
  } else {
    ball_info.detected = false;
  }
}

void WorldModelPublisherComponent::visionGeometryCallback(
  const robocup_ssl_msgs::msg::GeometryData::SharedPtr msg)
{
  field_h = msg->field.field_width / 1000.;
  field_w = msg->field.field_length / 1000.;

  goal_h = msg->field.goal_depth / 1000.;
  goal_w = msg->field.goal_width / 1000.;

  defense_area_h = goal_w;
  defense_area_w = 2. * goal_w;

  //  std::cout << "h : " << msg->field.penalty_area_depth.size();
  //  for (auto h : msg->field.penalty_area_depth) {
  //    std::cout << static_cast<double>(h) << ", ";
  //  }
  //  std::cout << std::endl;
  //  std::cout << "w : " << msg->field.penalty_area_width.size();
  //  for (auto w : msg->field.penalty_area_width) {
  //    std::cout << static_cast<double>(w) << ", ";
  //  }
  //  std::cout << std::endl;
  //  penalty_area_h_ = msg->field.penalty_area_depth;
  //  penalty_area_w_ = msg->field.penalty_area_width;

  // msg->boundary_width
  // msg->field_lines
  // msg->field_arcs
}

void WorldModelPublisherComponent::publishWorldModel()
{
  crane_msgs::msg::WorldModel wm;

  wm.is_yellow = (our_color == Color::YELLOW);
  wm.ball_info = ball_info;

  for (auto robot : robot_info[static_cast<uint8_t>(our_color)]) {
    crane_msgs::msg::RobotInfoOurs info;
    info.id = robot.robot_id;
    info.disappeared = robot.disappeared;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    wm.robot_info_ours.emplace_back(info);
  }
  for (auto robot : robot_info[static_cast<uint8_t>(their_color)]) {
    crane_msgs::msg::RobotInfoTheirs info;
    info.id = robot.robot_id;
    info.disappeared = robot.disappeared;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    wm.robot_info_theirs.emplace_back(info);
  }

  wm.field_info.x = field_w;
  wm.field_info.y = field_h;

  wm.defense_area.x = defense_area_h;
  wm.defense_area.y = defense_area_w;

  wm.goal.x = goal_h;
  wm.goal.y = goal_w;

  pub_world_model->publish(wm);
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
