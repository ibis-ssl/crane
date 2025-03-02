// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_data_provider.hpp"

#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <crane_msgs/msg/robot_info.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <string>
#include <vector>

namespace crane
{
WorldModelDataProvider::WorldModelDataProvider(rclcpp::Node & node) : node(node)
{
  using std::chrono_literals::operator""ms;
  node.declare_parameter("tracker_address", "224.5.23.2");
  node.declare_parameter("tracker_port", 11010);
  tracker_receiver = std::make_unique<multicast::MulticastReceiver>(
    node.get_parameter("tracker_address").get_value<std::string>(),
    node.get_parameter("tracker_port").get_value<int>());
  node.declare_parameter("vision_address", "224.5.23.2");
  node.declare_parameter("vision_port", 10020);
  vision_receiver = std::make_unique<multicast::MulticastReceiver>(
    node.get_parameter("vision_address").get_value<std::string>(),
    node.get_parameter("vision_port").get_value<int>());

  udp_timer = node.create_wall_timer(10ms, std::bind(&WorldModelDataProvider::on_udp_timer, this));

  for (int i = 0; i < 20; i++) {
    crane_msgs::msg::RobotInfo info;
    info.detected = false;
    info.id = i;
    robot_info[0].emplace_back(info);
    robot_info[1].emplace_back(info);
  }

  node.create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1,
    [this](const crane_msgs::msg::PlaySituation::SharedPtr msg) { latest_play_situation = *msg; });

  node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1, [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) {
      robot_feedback = *msg;
      auto now = rclcpp::Clock().now();
      for (auto & robot : robot_info[0]) {
        auto & contact = robot.ball_contact;
        contact.current_time = now;
        if (auto feedback = std::find_if(
              robot_feedback.feedback.begin(), robot_feedback.feedback.end(),
              [&](const crane_msgs::msg::RobotFeedback & f) { return f.robot_id == robot.id; });
            feedback != robot_feedback.feedback.end()) {
          contact.is_vision_source = false;
          if (feedback->ball_sensor) {
            contact.last_contacted_time = now;
          }
          ball_sensor_detected[robot.id] = feedback->ball_sensor;
        }
      }
    });

  node.create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/blue", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (our_color == Color::BLUE) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          ball_sensor_detected[status.robot_id] = status.infrared;
          auto & contact =
            robot_info[static_cast<uint8_t>(our_color)][status.robot_id].ball_contact;
          contact.current_time = now;
          contact.is_vision_source = false;
          if (status.infrared) {
            contact.last_contacted_time = now;
          }
        }
      }
    });

  node.create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/yellow", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (our_color == Color::YELLOW) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          ball_sensor_detected[status.robot_id] = status.infrared;
          auto & contact =
            robot_info[static_cast<uint8_t>(our_color)][status.robot_id].ball_contact;
          contact.current_time = now;
          contact.is_vision_source = false;
          if (status.infrared) {
            contact.last_contacted_time = now;
          }
        }
      }
    });

  node.declare_parameter("team_name", "ibis-ssl");
  team_name = node.get_parameter("team_name").as_string();

  node.declare_parameter("initial_team_color", "BLUE");
  auto initial_team_color = node.get_parameter("initial_team_color").as_string();
  if (initial_team_color == "BLUE") {
    our_color = Color::BLUE;
    their_color = Color::YELLOW;
  } else {
    our_color = Color::YELLOW;
    their_color = Color::BLUE;
  }

  node.declare_parameter("is_emplace_positive_side", true);
  is_emplace_positive_side = node.get_parameter("is_emplace_positive_side").get_value<bool>();

  node.create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == team_name) {
        // YELLOW
        our_color = Color::YELLOW;
        their_color = Color::BLUE;
        our_goalie_id = msg.yellow.goalkeeper;
        their_goalie_id = msg.blue.goalkeeper;
        if (not msg.yellow.max_allowed_bots.empty()) {
          our_max_allowed_bots = msg.yellow.max_allowed_bots[0];
        }
        if (not msg.blue.max_allowed_bots.empty()) {
          their_max_allowed_bots = msg.blue.max_allowed_bots[0];
        }
        if (not msg.blue_team_on_positive_half.empty()) {
          on_positive_half = not msg.blue_team_on_positive_half[0];
        }
      } else if (msg.blue.name == team_name) {
        // BLUE
        our_color = Color::BLUE;
        their_color = Color::YELLOW;
        our_goalie_id = msg.blue.goalkeeper;
        their_goalie_id = msg.yellow.goalkeeper;
        if (not msg.blue.max_allowed_bots.empty()) {
          our_max_allowed_bots = msg.blue.max_allowed_bots[0];
        }
        if (not msg.yellow.max_allowed_bots.empty()) {
          their_max_allowed_bots = msg.yellow.max_allowed_bots[0];
        }
        if (not msg.blue_team_on_positive_half.empty()) {
          on_positive_half = msg.blue_team_on_positive_half[0];
        }
      } else {
        std::stringstream what;
        what << "Cannot find our team name, " << std::string(team_name) << " in referee message. ";
        what << "blue team name: " << std::string(msg.blue.name)
             << ", yellow team name: " << std::string(msg.yellow.name);
        //        throw std::runtime_error(what.str());
      }

      if (not msg.designated_position.empty()) {
        ball_placement_target_x = msg.designated_position.front().x / 1000.;
        ball_placement_target_y = msg.designated_position.front().y / 1000.;
      }
    });
}

void WorldModelDataProvider::on_udp_timer()
{
  while (tracker_receiver->available()) {
    has_tracker_updated = true;
    std::vector<char> buf(2048);
    const size_t size = tracker_receiver->receive(buf);

    if (size > 0) {
      TrackerWrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if (packet.has_tracked_frame()) {
        trackerCallback(packet.tracked_frame());
      }
    }
  }

  while (vision_receiver->available()) {
    has_vision_updated = true;
    std::vector<char> buf(2048);
    const size_t size = vision_receiver->receive(buf);

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

void WorldModelDataProvider::trackerCallback(const TrackedFrame & tracked_frame)
{
  rclcpp::Time current_time = node.now();
  for (auto & robot : robot_info[0]) {
    robot.detected = false;
  }
  for (auto & robot : robot_info[1]) {
    robot.detected = false;
  }

  for (const auto & robot : tracked_frame.robots()) {
    int team_index = (robot.robot_id().team() == robocup_ssl_msgs::msg::RobotId::TEAM_COLOR_YELLOW)
                       ? static_cast<int>(Color::YELLOW)
                       : static_cast<int>(Color::BLUE);

    auto & each_robot_info = robot_info[team_index].at(robot.robot_id().id());
    if (robot.has_visibility()) {
      each_robot_info.detected = (robot.visibility() > 0.5);
    } else {
      each_robot_info.detected = false;
    }

    auto last_frame_stamp = each_robot_info.last_tracker_detection_stamp;
    //    each_robot_info.robot_id = robot.robot_id.id;
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

      // 加速度の計算
      if (double dt = (current_time - last_frame_stamp).seconds(); dt > 0) {
        each_robot_info.acceleration.x = (each_robot_info.velocity.x - previous_velocity.x) / dt;
        each_robot_info.acceleration.y = (each_robot_info.velocity.y - previous_velocity.y) / dt;

        each_robot_info.acceleration_norm =
          std::hypot(each_robot_info.acceleration.x, each_robot_info.acceleration.y);
        std::cout << "dt: " << dt << ", acc: " << each_robot_info.acceleration_norm << std::endl;
      }
    } else {
      // 速度情報がない場合、加速度を0に設定
      each_robot_info.acceleration.x = 0.0;
      each_robot_info.acceleration.y = 0.0;
      each_robot_info.acceleration_norm = 0.0;
    }
    if (robot.has_vel_angular()) {
      each_robot_info.velocity.theta = robot.vel_angular();
    } else {
      // calc from diff
    }
  }

  if (not tracked_frame.balls().empty()) {
    auto ball = tracked_frame.balls().begin();
    ball_info.pose.x = ball->pos().x();
    ball_info.pose.y = ball->pos().y();

    if (ball->has_vel()) {
      ball_info.velocity.x = ball->vel().x();
      ball_info.velocity.y = ball->vel().y();
      ball_info.velocity_norm = std::hypot(ball_info.velocity.x, ball_info.velocity.y);
    }

    ball_info.detected = true;
    ball_info.detection_time = tracked_frame.timestamp();
    ball_info.disappeared = false;
  } else {
    ball_info.detected = false;

    // ball disappeared 判定
    double elapsed_time_since_last_detected = (node.now() - last_ball_detect_time).seconds();
    // 0.5secビジョンから見えていなければ見失った
    if (0.5 < elapsed_time_since_last_detected) {
      ball_info.disappeared = true;
    }
  }
}

void WorldModelDataProvider::visionGeometryCallback(const SSL_GeometryData & geometry_data)
{
  field_h = geometry_data.field().field_width() / 1000.;
  field_w = geometry_data.field().field_length() / 1000.;

  goal_h = geometry_data.field().goal_depth() / 1000.;
  goal_w = geometry_data.field().goal_width() / 1000.;

  if (geometry_data.field().has_penalty_area_depth()) {
    penalty_area_h = geometry_data.field().penalty_area_depth() / 1000.;
  } else {
    penalty_area_h = goal_w;
  }

  if (geometry_data.field().has_penalty_area_width()) {
    penalty_area_w = geometry_data.field().penalty_area_width() / 1000.;
  } else {
    penalty_area_w = goal_w * 2.;
  }

  // msg.boundary_width
  // msg.field_lines
  // msg.field_arcs
}

void WorldModelDataProvider::visionDetectionCallback(const SSL_DetectionFrame & detection_frame)
{
  int balls_size = detection_frame.balls().size();
  if (0 > balls_size) {
    last_ball_detect_time = node.now();
  }

  for (const auto & robot : detection_frame.robots_yellow()) {
    if (robot.has_robot_id()) {
      auto & each_robot_info = robot_info[static_cast<int>(Color::YELLOW)].at(robot.robot_id());
      //      each_robot_info.last_vision_detection_stamp = detection_frame.t_capture();
    }
  }

  for (const auto & robot : detection_frame.robots_blue()) {
    if (robot.has_robot_id()) {
      auto & each_robot_info = robot_info[static_cast<int>(Color::BLUE)].at(robot.robot_id());
      //      each_robot_info.last_vision_detection_stamp = detection_frame.t_capture();
    }
  }
}

crane_msgs::msg::WorldModel WorldModelDataProvider::getMsg()
{
  crane_msgs::msg::WorldModel msg;
  msg.is_yellow = (our_color == Color::YELLOW);
  msg.on_positive_half = on_positive_half;
  msg.is_emplace_positive_side = is_emplace_positive_side;
  msg.our_max_allowed_bots = our_max_allowed_bots;
  msg.their_max_allowed_bots = their_max_allowed_bots;

  msg.ball_info = ball_info;

  for (const auto & robot : robot_info[static_cast<uint8_t>(our_color)]) {
    msg.robot_info_ours.emplace_back(robot);
  }
  for (const auto & robot : robot_info[static_cast<uint8_t>(their_color)]) {
    msg.robot_info_theirs.emplace_back(robot);
  }

  msg.field_info.x = field_w;
  msg.field_info.y = field_h;

  msg.penalty_area_size.x = penalty_area_h;
  msg.penalty_area_size.y = penalty_area_w;

  msg.goal_size.x = goal_h;
  msg.goal_size.y = goal_w;

  msg.our_goalie_id = our_goalie_id;
  msg.their_goalie_id = their_goalie_id;

  msg.play_situation = latest_play_situation;

  msg.header.stamp = rclcpp::Clock().now();
  return msg;
}
}  // namespace crane
