// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_publisher.hpp"

namespace crane
{
WorldModelPublisherComponent::WorldModelPublisherComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("world_model_publisher", options)
{
  sub_vision = create_subscription<robocup_ssl_msgs::msg::TrackedFrame>(
    "/detection_tracked", 1,
    [this](const robocup_ssl_msgs::msg::TrackedFrame::SharedPtr msg) -> void {
      visionDetectionsCallback(msg);
    });

  sub_geometry = create_subscription<robocup_ssl_msgs::msg::GeometryData>(
    "/geometry", 1,
    [this](const robocup_ssl_msgs::msg::GeometryData::SharedPtr msg) {
      visionGeometryCallback(msg);
    });

  pub_world_model =
    create_publisher<crane_msgs::msg::WorldModel>("/world_model", 1);

  using std::chrono_literals;

  timer = this->create_wall_timer(16ms, [this]() {
    if (has_vision_updated && has_geometry_updated) {
      publishWorldModel();
    }
  });

  declare_parameter("team_name", "ibis-ssl");
  team_name = get_parameter("team_name").as_string();

  sub_referee = this->create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == team_name) {
        our_color = Color::YELLOW;
        their_color = Color::BLUE;
        our_goalie_id = msg.yellow.goalkeeper;
        their_goalie_id = msg.blue.goalkeeper;
      } else if (msg.blue.name == team_name) {
        our_color = Color::BLUE;
        their_color = Color::YELLOW;
        our_goalie_id = msg.blue.goalkeeper;
        their_goalie_id = msg.yellow.goalkeeper;
      } else {
        std::stringstream what;
        what << "Cannot find our team name, " << team_name
             << " in referee message. ";
        what << "blue team name: " << msg.blue.name
             << ", yellow team name: " << msg.yellow.name;
        throw std::runtime_error(what.str());
      }

      if (not msg.designated_position.empty()) {
        ball_placement_target_x = msg.designated_position.front().x / 1000.;
        ball_placement_target_y = msg.designated_position.front().y / 1000.;
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
  // TODO(HansRobo): 全部クリアしていたら複数カメラのときにうまく更新できないはず
  robot_info[0].clear();
  robot_info[1].clear();
  for (auto & robot : msg->robots) {
    crane_msgs::msg::RobotInfo each_robot_info;
    if (not robot.visibility.empty()) {
      each_robot_info.detected = (robot.visibility.front() > 0.8);
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

    int team_index = (robot.robot_id.team_color ==
                      robocup_ssl_msgs::msg::RobotId::TEAM_COLOR_YELLOW)
                       ? static_cast<int>(Color::YELLOW)
                       : static_cast<int>(Color::BLUE);

    robot_info[team_index].push_back(each_robot_info);
  }

  if (!msg->balls.empty()) {
    auto ball_msg = msg->balls.front();
    ball_info.pose.x = ball_msg.pos.x;
    ball_info.pose.y = ball_msg.pos.y;

    if (not ball_msg.vel.empty()) {
      ball_info.velocity.x = ball_msg.vel.front().x;
      ball_info.velocity.y = ball_msg.vel.front().y;
    }

    ball_info.detected = true;
    ball_info.detection_time = msg->timestamp;
    ball_info.disappeared = false;
  } else {
    ball_info.detected = false;
  }

  has_vision_updated = true;
}

void WorldModelPublisherComponent::visionGeometryCallback(
  const robocup_ssl_msgs::msg::GeometryData::SharedPtr msg)
{
  field_h = msg->field.field_width / 1000.;
  field_w = msg->field.field_length / 1000.;

  goal_h = msg->field.goal_depth / 1000.;
  goal_w = msg->field.goal_width / 1000.;

  if (not msg->field.penalty_area_depth.empty()) {
    std::cout << msg->field.penalty_area_depth.front() << std::endl;
    defense_area_h = msg->field.penalty_area_depth.front() / 1000.;
  }

  if (not msg->field.penalty_area_width.empty()) {
    defense_area_w = msg->field.penalty_area_width.front() / 1000.;
  }

  // msg->boundary_width
  // msg->field_lines
  // msg->field_arcs

  has_geometry_updated = true;
}

void WorldModelPublisherComponent::publishWorldModel()
{
  crane_msgs::msg::WorldModel wm;

  wm.is_yellow = (our_color == Color::YELLOW);
  wm.ball_info = ball_info;

  updateBallContact();

  for (auto robot : robot_info[static_cast<uint8_t>(our_color)]) {
    crane_msgs::msg::RobotInfoOurs info;
    info.id = robot.robot_id;
    info.disappeared = !robot.detected;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    info.ball_contact = robot.ball_contact;
    wm.robot_info_ours.emplace_back(info);
  }
  for (auto robot : robot_info[static_cast<uint8_t>(their_color)]) {
    crane_msgs::msg::RobotInfoTheirs info;
    info.id = robot.robot_id;
    info.disappeared = !robot.detected;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    info.ball_contact = robot.ball_contact;
    wm.robot_info_theirs.emplace_back(info);
  }

  wm.field_info.x = field_w;
  wm.field_info.y = field_h;

  wm.defense_area_size.x = defense_area_h;
  wm.defense_area_size.y = defense_area_w;

  wm.goal_size.x = goal_h;
  wm.goal_size.y = goal_w;

  wm.ball_placement_target.x = ball_placement_target_x;
  wm.ball_placement_target.y = ball_placement_target_y;

  wm.our_goalie_id = our_goalie_id;
  wm.their_goalie_id = their_goalie_id;

  pub_world_model->publish(wm);
}

void WorldModelPublisherComponent::updateBallContact()
{
  auto now = rclcpp::Clock().now();
  for (auto & robot : robot_info[static_cast<uint8_t>(our_color)]) {
    auto & contact = robot.ball_contact;
    // TODO(HansRobo): ロボットのドリブラセンサを使った判定を実装する
    contact.current_time = now;
    if (robot.detected) {
      auto ball_dist = std::hypot(
        ball_info.pose.x - robot.pose.x, ball_info.pose.y - robot.pose.y);
      contact.is_vision_source = true;
      if (ball_dist < 0.1) {
        contact.last_contacted_time = contact.current_time;
      }
    }
  }

  for (auto & robot : robot_info[static_cast<uint8_t>(their_color)]) {
    auto & contact = robot.ball_contact;
    contact.current_time = now;
    if (robot.detected) {
      auto ball_dist = std::hypot(
        ball_info.pose.x - robot.pose.x, ball_info.pose.y - robot.pose.y);
      contact.is_vision_source = true;
      if (ball_dist < 0.1) {
        contact.last_contacted_time = contact.current_time;
      }
    }
  }
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
