// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/time.hpp>
#include <crane_world_model_publisher/world_model_publisher.hpp>
#include <deque>

namespace crane
{
WorldModelPublisherComponent::WorldModelPublisherComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("world_model_publisher", options)
{
  using std::chrono_literals::operator""ms;
  declare_parameter("tracker_address", "224.5.23.2");
  declare_parameter("tracker_port", 11010);
  tracker_receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("tracker_address").get_value<std::string>(),
    get_parameter("tracker_port").get_value<int>());
  declare_parameter("vision_address", "224.5.23.2");
  declare_parameter("vision_port", 10006);
  geometry_receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("vision_address").get_value<std::string>(),
    get_parameter("vision_port").get_value<int>());
  udp_timer = rclcpp::create_timer(
    this, get_clock(), 10ms, std::bind(&WorldModelPublisherComponent::on_udp_timer, this));

  pub_process_time = create_publisher<std_msgs::msg::Float32>("~/process_time", 10);
  for (int i = 0; i < 20; i++) {
    crane_msgs::msg::RobotInfo info;
    info.detected = false;
    info.robot_id = i;
    robot_info[0].emplace_back(info);
    robot_info[1].emplace_back(info);
  }

  //  sub_vision = create_subscription<robocup_ssl_msgs::msg::TrackedFrame>(
  //    "/detection_tracked", 1,
  //    [this](const robocup_ssl_msgs::msg::TrackedFrame::SharedPtr msg) -> void {
  //      visionDetectionsCallback(msg);
  //    });
  //
  //  sub_geometry = create_subscription<robocup_ssl_msgs::msg::GeometryData>(
  //    "/geometry", 1, [this](const robocup_ssl_msgs::msg::GeometryData::SharedPtr msg) {
  //      visionGeometryCallback(msg);
  //    });

  sub_play_situation = create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1,
    [this](const crane_msgs::msg::PlaySituation::SharedPtr msg) { latest_play_situation = *msg; });

  sub_robot_feedback = create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1, [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) {
      robot_feedback = *msg;
      auto now = rclcpp::Clock().now();
      for (auto & robot : robot_info[static_cast<uint8_t>(our_color)]) {
        auto & contact = robot.ball_contact;
        contact.current_time = now;
        if (auto feedback = std::find_if(
              robot_feedback.feedback.begin(), robot_feedback.feedback.end(),
              [&](const crane_msgs::msg::RobotFeedback & f) {
                return f.robot_id == robot.robot_id;
              });
            feedback != robot_feedback.feedback.end()) {
          contact.is_vision_source = false;
          if (feedback->ball_sensor) {
            contact.last_contacted_time = now;
          }
          ball_detected[robot.robot_id] = feedback->ball_sensor;
        }
      }
    });

  sub_robots_status_blue = create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/blue", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (our_color == Color::BLUE) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          ball_detected[status.robot_id] = status.infrared;
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

  sub_robots_status_yellow = create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/yellow", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (our_color == Color::YELLOW) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          ball_detected[status.robot_id] = status.infrared;
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

  pub_world_model = create_publisher<crane_msgs::msg::WorldModel>("/world_model", 1);

  using std::chrono::operator""ms;
  timer = rclcpp::create_timer(this, get_clock(), 16ms, [this]() {
    if (has_vision_updated && has_geometry_updated) {
      publishWorldModel();
    }
  });

  declare_parameter("team_name", "ibis-ssl");
  team_name = get_parameter("team_name").as_string();

  declare_parameter("initial_team_color", "BLUE");
  auto initial_team_color = get_parameter("initial_team_color").as_string();
  if (initial_team_color == "BLUE") {
    our_color = Color::BLUE;
    their_color = Color::YELLOW;
  } else {
    our_color = Color::YELLOW;
    their_color = Color::BLUE;
  }

  sub_referee = this->create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == team_name) {
        // YELLOW
        our_color = Color::YELLOW;
        their_color = Color::BLUE;
        our_goalie_id = msg.yellow.goalkeeper;
        their_goalie_id = msg.blue.goalkeeper;
        if (not msg.blue_team_on_positive_half.empty()) {
          on_positive_half = not msg.blue_team_on_positive_half[0];
        }
      } else if (msg.blue.name == team_name) {
        // BLUE
        our_color = Color::BLUE;
        their_color = Color::YELLOW;
        our_goalie_id = msg.blue.goalkeeper;
        their_goalie_id = msg.yellow.goalkeeper;
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

void WorldModelPublisherComponent::on_udp_timer()
{
  while (tracker_receiver->available()) {
    std::vector<char> buf(2048);
    const size_t size = tracker_receiver->receive(buf);

    if (size > 0) {
      TrackerWrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if (packet.has_tracked_frame()) {
        visionDetectionsCallback(packet.tracked_frame());
      }
    }
  }

  while (geometry_receiver->available()) {
    std::vector<char> buf(2048);
    const size_t size = geometry_receiver->receive(buf);

    if (size > 0) {
      SSL_WrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));
      if (packet.has_geometry()) {
        visionGeometryCallback(packet.geometry());
      }
    }
  }
}

void WorldModelPublisherComponent::visionDetectionsCallback(const TrackedFrame & tracked_frame)
{
  ScopedTimer process_timer(pub_process_time);
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
    if (not robot.has_visibility()) {
      each_robot_info.detected = (robot.visibility() > 0.5);
    } else {
      each_robot_info.detected = false;
    }

    //    each_robot_info.robot_id = robot.robot_id.id;
    each_robot_info.pose.x = robot.pos().x();
    each_robot_info.pose.y = robot.pos().y();
    each_robot_info.pose.theta = robot.orientation();
    // each_robot_info.detection_stamp = robot.stamp;
    if (not robot.has_vel()) {
      each_robot_info.velocity.x = robot.vel().x();
      each_robot_info.velocity.y = robot.vel().y();
    } else {
      // calc from diff
    }
    if (not robot.has_vel_angular()) {
      each_robot_info.velocity.theta = robot.vel_angular();
    } else {
      // calc from diff
    }
  }

  if (not tracked_frame.balls().empty()) {
    auto ball = tracked_frame.balls().begin();
    ball_info.pose.x = ball->pos().x();
    ball_info.pose.y = ball->pos().y();

    if (not ball->has_vel()) {
      ball_info.velocity.x = ball->vel().x();
      ball_info.velocity.y = ball->vel().y();
    }

    ball_info.detected = true;
    ball_info.detection_time = tracked_frame.timestamp();
    ball_info.disappeared = false;
  } else {
    ball_info.detected = false;
  }

  has_vision_updated = true;
}

void WorldModelPublisherComponent::visionGeometryCallback(const SSL_GeometryData & geometry_data)
{
  field_h = geometry_data.field().field_width() / 1000.;
  field_w = geometry_data.field().field_length() / 1000.;

  goal_h = geometry_data.field().goal_depth() / 1000.;
  goal_w = geometry_data.field().goal_width() / 1000.;

  if (not geometry_data.field().has_penalty_area_depth()) {
    penalty_area_h = geometry_data.field().penalty_area_depth() / 1000.;
  }

  if (not geometry_data.field().has_penalty_area_width()) {
    penalty_area_w = geometry_data.field().penalty_area_width() / 1000.;
  }

  // msg.boundary_width
  // msg.field_lines
  // msg.field_arcs

  has_geometry_updated = true;
}

void WorldModelPublisherComponent::publishWorldModel()
{
  crane_msgs::msg::WorldModel wm;

  wm.is_yellow = (our_color == Color::YELLOW);
  wm.on_positive_half = on_positive_half;
  wm.ball_info = ball_info;

  updateBallContact();

  wm.ball_info.state_changed = false;
  if (ball_event_detected) {
    switch (last_ball_event) {
      case BallEvent::NONE:
        if (is_our_ball && not is_their_ball) {
          last_ball_event = BallEvent::OUR_BALL;
          wm.ball_info.state_changed = true;
        } else if (is_their_ball && not is_our_ball) {
          last_ball_event = BallEvent::THEIR_BALL;
          wm.ball_info.state_changed = true;
        }
        break;
      case BallEvent::OUR_BALL:
        if (is_their_ball && not is_our_ball) {
          last_ball_event = BallEvent::THEIR_BALL;
          wm.ball_info.state_changed = true;
        } else if (is_our_ball == is_their_ball) {
          last_ball_event = BallEvent::NONE;
          wm.ball_info.state_changed = true;
        }
        break;
      case BallEvent::THEIR_BALL:
        if (is_our_ball && not is_their_ball) {
          last_ball_event = BallEvent::OUR_BALL;
          wm.ball_info.state_changed = true;
        } else if (is_their_ball == is_our_ball) {
          last_ball_event = BallEvent::NONE;
          wm.ball_info.state_changed = true;
        }
        break;
    }
  }

  switch (last_ball_event) {
    case BallEvent::OUR_BALL:
      wm.ball_info.is_our_ball = true;
      wm.ball_info.is_their_ball = false;
      break;
    case BallEvent::THEIR_BALL:
      wm.ball_info.is_our_ball = false;
      wm.ball_info.is_their_ball = true;
      break;
    case BallEvent::NONE:
      wm.ball_info.is_our_ball = false;
      wm.ball_info.is_their_ball = false;
      break;
    default:
      break;
  }

  wm.ball_info.event_detected = ball_event_detected;

  for (const auto & robot : robot_info[static_cast<uint8_t>(our_color)]) {
    crane_msgs::msg::RobotInfoOurs info;
    info.id = robot.robot_id;
    info.disappeared = !robot.detected;
    info.detection_stamp = robot.detection_stamp;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    info.ball_contact = robot.ball_contact;
    wm.robot_info_ours.emplace_back(info);
  }
  for (const auto & robot : robot_info[static_cast<uint8_t>(their_color)]) {
    crane_msgs::msg::RobotInfoTheirs info;
    info.id = robot.robot_id;
    info.disappeared = !robot.detected;
    info.detection_stamp = robot.detection_stamp;
    info.pose = robot.pose;
    info.velocity = robot.velocity;
    info.ball_contact = robot.ball_contact;
    wm.robot_info_theirs.emplace_back(info);
  }

  wm.field_info.x = field_w;
  wm.field_info.y = field_h;

  wm.penalty_area_size.x = penalty_area_h;
  wm.penalty_area_size.y = penalty_area_w;

  wm.goal_size.x = goal_h;
  wm.goal_size.y = goal_w;

  wm.our_goalie_id = our_goalie_id;
  wm.their_goalie_id = their_goalie_id;

  wm.play_situation = latest_play_situation;

  wm.header.stamp = rclcpp::Clock().now();

  pub_world_model->publish(wm);
}

void WorldModelPublisherComponent::updateBallContact()
{
  auto now = rclcpp::Clock().now();
  static std::deque<crane_msgs::msg::BallInfo> ball_info_history;

  ball_info_history.emplace_back(ball_info);
  if (ball_info_history.size() > 10) {
    ball_info_history.pop_front();
  }

  //  bool pre_is_our_ball = std::exchange(is_our_ball, false);
  is_their_ball = false;
  ball_event_detected = false;

  if (ball_info_history.size() > 2) {
    const auto & latest = ball_info_history.at(ball_info_history.size() - 1);
    const auto & pre = ball_info_history.at(ball_info_history.size() - 2);
    double pre_vel = std::hypot(pre.velocity.x, pre.velocity.y);
    double vel_diff =
      std::hypot(latest.velocity.x - pre.velocity.x, latest.velocity.y - pre.velocity.y);

    int count = vel_diff / (pre_vel + 0.1) * 100;
    if (count > 50) {
      ball_event_detected = true;
      std::cout << "イベント発生: " << count << std::endl;

      crane_msgs::msg::RobotInfo nearest_friend;
      crane_msgs::msg::RobotInfo nearest_enemy;
      double nearest_friend_dist = 1000.0;
      for (const auto & robot : robot_info[static_cast<uint8_t>(our_color)]) {
        if (robot.detected) {
          double dist = std::hypot(latest.pose.x - robot.pose.x, latest.pose.y - robot.pose.y);
          if (dist < nearest_friend_dist) {
            nearest_friend = robot;
            nearest_friend_dist = dist;
          }
        }
      }

      double nearest_enemy_dist = 1000.0;
      for (const auto & robot : robot_info[static_cast<uint8_t>(their_color)]) {
        if (robot.detected) {
          double dist = std::hypot(latest.pose.x - robot.pose.x, latest.pose.y - robot.pose.y);
          if (dist < nearest_enemy_dist) {
            nearest_enemy = robot;
            nearest_enemy_dist = dist;
          }
        }
      }
      {
        double ball_angle =
          std::atan2(latest.pose.y - nearest_friend.pose.y, latest.pose.x - nearest_friend.pose.x);
        double angle_diff = std::abs(getAngleDiff(nearest_friend.pose.theta, ball_angle));
        if (nearest_friend_dist < 0.3 && angle_diff < 0.4) {
          std::cout << "味方ボール接触" << std::endl;
          is_our_ball = true;
        }
      }
      {
        double ball_angle =
          std::atan2(latest.pose.y - nearest_enemy.pose.y, latest.pose.x - nearest_enemy.pose.x);
        double angle_diff = std::abs(getAngleDiff(nearest_enemy.pose.theta, ball_angle));
        if (nearest_enemy_dist < 0.3 && angle_diff < 0.4) {
          std::cout << "敵ボール接触" << std::endl;
          is_their_ball = true;
        }
      }
    }
  }

  // ローカルセンサーの情報でボール情報を更新
  for (std::size_t i = 0; i < robot_info[static_cast<uint8_t>(our_color)].size(); i++) {
    // ボールがロボットに近いときのみ接触とみなす（誤作動防止）
    double ball_distance = std::hypot(
      ball_info.pose.x - robot_info[static_cast<uint8_t>(our_color)][i].pose.x,
      ball_info.pose.y - robot_info[static_cast<uint8_t>(our_color)][i].pose.y);
    if (
      ball_detected[i] && not robot_info[static_cast<uint8_t>(our_color)][i].disappeared &&
      ball_distance < 0.3) {
      robot_info[static_cast<uint8_t>(our_color)][i].ball_contact.is_vision_source = false;
      robot_info[static_cast<uint8_t>(our_color)][i].ball_contact.current_time = now;
      robot_info[static_cast<uint8_t>(our_color)][i].ball_contact.last_contacted_time = now;
      if (not is_our_ball) {
        std::cout << "敵ボール接触" << std::endl;
        is_our_ball = true;
        ball_event_detected = true;
      }
    }
  }
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
