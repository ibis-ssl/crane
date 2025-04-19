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
auto createTransformMatrix(bool enable, bool is_positive_side, double field_width)
  -> Eigen::Matrix3d
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();  // 単位行列で初期化
  if (enable) {
    // 半面コートの中心点の座標を計算
    double half_court_center_x = is_positive_side ? field_width / 4.0 : -field_width / 4.0;

    // 1. 半面コートの中心を原点に移動
    Eigen::Matrix3d translate_to_origin = Eigen::Matrix3d::Identity();
    translate_to_origin(0, 2) = -half_court_center_x;

    // 2. スケーリング
    Eigen::Matrix3d scale_matrix = Eigen::Matrix3d::Identity();
    scale_matrix(0, 0) = 1.0;
    scale_matrix(1, 1) = 1.0;

    // 3. 回転 (90度)
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    rotation_matrix(0, 0) = 0.0;
    rotation_matrix(0, 1) = -1.0;
    rotation_matrix(1, 0) = 1.0;
    rotation_matrix(1, 1) = 0.0;

    // 4. 原点を中心に戻す
    Eigen::Matrix3d translate_back = Eigen::Matrix3d::Identity();
    translate_back(0, 2) = 0.0;  // 新しい座標系の原点に配置

    // 全ての変換を合成 (右から左へ適用)
    matrix = translate_back * rotation_matrix * scale_matrix * translate_to_origin;
  }
  return matrix;
}

WorldModelDataProvider::WorldModelDataProvider(rclcpp::Node & node)
: node(node), vis_data_handler(node)
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
    data.robot_info[0].emplace_back(info);
    data.robot_info[1].emplace_back(info);
  }

  // /play_situationのトピック統計はsession_controllerで取得
  sub_play_situation = node.create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1,
    [this](const crane_msgs::msg::PlaySituation msg) { latest_play_situation = msg; });

  sub_robot_feedback = node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1, [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) {
      robot_feedback = *msg;
      auto now = rclcpp::Clock().now();
      for (auto & robot : data.robot_info[0]) {
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
          // 範囲内参照で実行時エラー
          // data.ball_sensor_detected[robot.id] = feedback->ball_sensor;
          auto & robot_info = data.robot_info[static_cast<uint8_t>(game_data.our_color)][robot.id];
          robot_info.ball_sensor = feedback->ball_sensor;
          robot_info.last_ball_sensor_stamp = now;
        }
      }
    });

  sub_robots_status_blue = node.create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/blue", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (game_data.our_color == Color::BLUE) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          // data.ball_sensor_detected[status.robot_id] = status.infrared;
          auto & robot =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id];
          robot.ball_sensor = status.infrared;
          robot.last_ball_sensor_stamp = now;
          auto & contact =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id]
              .ball_contact;
          contact.current_time = now;
          contact.is_vision_source = false;
          if (status.infrared) {
            contact.last_contacted_time = now;
          }
        }
      }
    });

  sub_robots_status_yellow = node.create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/yellow", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (game_data.our_color == Color::YELLOW) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          // data.ball_sensor_detected[status.robot_id] = status.infrared;
          auto & robot =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id];
          robot.ball_sensor = status.infrared;
          robot.last_ball_sensor_stamp = now;

          auto & contact =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id]
              .ball_contact;
          contact.current_time = now;
          contact.is_vision_source = false;
          if (status.infrared) {
            contact.last_contacted_time = now;
          }
        }
      }
    });

  node.declare_parameter("team_name", "ibis-ssl");
  game_data.team_name = node.get_parameter("team_name").as_string();

  node.declare_parameter("initial_team_color", "BLUE");
  auto initial_team_color = node.get_parameter("initial_team_color").as_string();
  if (initial_team_color == "BLUE") {
    game_data.our_color = Color::BLUE;
    game_data.their_color = Color::YELLOW;
  } else {
    game_data.our_color = Color::YELLOW;
    game_data.their_color = Color::BLUE;
  }

  node.declare_parameter("is_emplace_positive_side", true);
  is_emplace_positive_side = node.get_parameter("is_emplace_positive_side").get_value<bool>();

  sub_referee = node.create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == game_data.team_name) {
        // YELLOW
        game_data.our_color = Color::YELLOW;
        game_data.their_color = Color::BLUE;
        game_data.our_goalie_id = msg.yellow.goalkeeper;
        game_data.their_goalie_id = msg.blue.goalkeeper;
        if (not msg.yellow.max_allowed_bots.empty()) {
          game_data.our_max_allowed_bots = msg.yellow.max_allowed_bots[0];
        }
        if (not msg.blue.max_allowed_bots.empty()) {
          game_data.their_max_allowed_bots = msg.blue.max_allowed_bots[0];
        }
        if (not msg.blue_team_on_positive_half.empty()) {
          on_positive_half = not msg.blue_team_on_positive_half[0];
        }
      } else if (msg.blue.name == game_data.team_name) {
        // BLUE
        game_data.our_color = Color::BLUE;
        game_data.their_color = Color::YELLOW;
        game_data.our_goalie_id = msg.blue.goalkeeper;
        game_data.their_goalie_id = msg.yellow.goalkeeper;
        if (not msg.blue.max_allowed_bots.empty()) {
          game_data.our_max_allowed_bots = msg.blue.max_allowed_bots[0];
        }
        if (not msg.yellow.max_allowed_bots.empty()) {
          game_data.their_max_allowed_bots = msg.yellow.max_allowed_bots[0];
        }
        if (not msg.blue_team_on_positive_half.empty()) {
          on_positive_half = msg.blue_team_on_positive_half[0];
        }
      } else {
        std::stringstream what;
        what << "Cannot find our team name, " << std::string(game_data.team_name)
             << " in referee message. ";
        what << "blue team name: " << std::string(msg.blue.name)
             << ", yellow team name: " << std::string(msg.yellow.name);
        //        throw std::runtime_error(what.str());
      }

      if (not msg.designated_position.empty()) {
        data.ball_placement_target_x = msg.designated_position.front().x / 1000.;
        data.ball_placement_target_y = msg.designated_position.front().y / 1000.;
      }
      vis_data_handler.publish_vis_referee(msg, game_data.field_w, game_data.field_h);

      transform_matrix = createTransformMatrix(
        half_court_practice_mode, half_court_is_positive_side, game_data.field_w);
    });
}

auto WorldModelDataProvider::on_udp_timer() -> void
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

auto WorldModelDataProvider::trackerCallback(const TrackedFrame & tracked_frame) -> void
{
  rclcpp::Time current_time = node.now();
  for (auto & robot : data.robot_info[0]) {
    robot.detected = false;
  }
  for (auto & robot : data.robot_info[1]) {
    robot.detected = false;
  }

  for (const auto & robot : tracked_frame.robots()) {
    int team_index = (robot.robot_id().team() == robocup_ssl_msgs::msg::RobotId::TEAM_COLOR_YELLOW)
                       ? static_cast<int>(Color::YELLOW)
                       : static_cast<int>(Color::BLUE);

    auto & each_robot_info = data.robot_info[team_index].at(robot.robot_id().id());
    if (robot.has_visibility()) {
      each_robot_info.detected = (robot.visibility() > 0.5);
    } else {
      each_robot_info.detected = false;
    }

    auto last_frame_stamp = each_robot_info.last_tracker_detection_stamp;

    // トラッカーコールバックではアフィン変換は適用せず、そのまま値を設定
    // 後でgetMsgで一括変換するようにする
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

    // トラッカーコールバックではアフィン変換は適用せず、そのまま値を設定
    // 後でgetMsgで一括変換するようにする
    data.ball_info.pose.x = ball->pos().x();
    data.ball_info.pose.y = ball->pos().y();

    if (ball->has_vel()) {
      data.ball_info.velocity.x = ball->vel().x();
      data.ball_info.velocity.y = ball->vel().y();
      data.ball_info.velocity_norm =
        std::hypot(data.ball_info.velocity.x, data.ball_info.velocity.y);
    }

    // data.ball_info.detected = true;
    data.ball_info.detection_time = tracked_frame.timestamp();
    data.ball_info.disappeared = false;
  } else {
    // data.ball_info.detected = false;

    // ball disappeared 判定
    double elapsed_time_since_last_detected = (node.now() - last_ball_detect_time).seconds();
    // 0.5secビジョンから見えていなければ見失った
    if (0.5 < elapsed_time_since_last_detected) {
      data.ball_info.disappeared = true;
    }
  }
}

auto WorldModelDataProvider::visionGeometryCallback(const SSL_GeometryData & geometry_data) -> void
{
  game_data.field_h = geometry_data.field().field_width() / 1000.;
  game_data.field_w = geometry_data.field().field_length() / 1000.;

  game_data.goal_h = geometry_data.field().goal_depth() / 1000.;
  game_data.goal_w = geometry_data.field().goal_width() / 1000.;

  if (geometry_data.field().has_penalty_area_depth()) {
    game_data.penalty_area_h = geometry_data.field().penalty_area_depth() / 1000.;
  } else {
    game_data.penalty_area_h = game_data.goal_w;
  }

  if (geometry_data.field().has_penalty_area_width()) {
    game_data.penalty_area_w = geometry_data.field().penalty_area_width() / 1000.;
  } else {
    game_data.penalty_area_w = game_data.goal_w * 2.;
  }

  vis_data_handler.publish_vis_geometry(geometry_data);
}

auto WorldModelDataProvider::visionDetectionCallback(const SSL_DetectionFrame & detection_frame)
  -> void
{
  int balls_size = detection_frame.balls().size();
  auto now = node.now();
  if (balls_size > 0) {
    last_ball_detect_time = now;
    data.ball_info.detected = true;
  } else {
    // 10ms以上更新がなければ見失った
    if (
      now.get_clock_type() == last_ball_detect_time.get_clock_type() &&
      (now - last_ball_detect_time).seconds() > 0.01) {
      data.ball_info.detected = false;
    }
  }

  for (const auto & robot : detection_frame.robots_yellow()) {
    if (robot.has_robot_id()) {
      auto & each_robot_info =
        data.robot_info[static_cast<int>(Color::YELLOW)].at(robot.robot_id());
      //      each_robot_info.last_vision_detection_stamp = detection_frame.t_capture();
    }
  }

  for (const auto & robot : detection_frame.robots_blue()) {
    if (robot.has_robot_id()) {
      auto & each_robot_info = data.robot_info[static_cast<int>(Color::BLUE)].at(robot.robot_id());
      //      each_robot_info.last_vision_detection_stamp = detection_frame.t_capture();
    }
  }
}

// アフィン変換行列を設定するメソッド
auto WorldModelDataProvider::setTransformInfo(bool enable, bool is_positive_side) -> void
{
  half_court_practice_mode = enable;
  half_court_is_positive_side = is_positive_side;

  transform_matrix =
    createTransformMatrix(half_court_practice_mode, half_court_is_positive_side, game_data.field_w);
}

// 座標変換を適用する関数
auto WorldModelDataProvider::applyTransformation(crane_msgs::msg::WorldModel & msg) -> void
{
  if (transform_matrix.isIdentity()) {
    return;  // 変換不要（単位行列の場合）
  }

  // フィールドサイズの変換はgetMsg内で行われるので、ここでは行わない

  // ボールの座標変換
  if (msg.ball_info.detected) {
    // 変換前の座標
    Eigen::Vector3d ball_pos(msg.ball_info.pose.x, msg.ball_info.pose.y, 1.0);
    Eigen::Vector3d ball_vel(msg.ball_info.velocity.x, msg.ball_info.velocity.y, 0.0);

    // 変換行列を適用
    Eigen::Vector3d transformed_pos = transform_matrix * ball_pos;

    // 速度は回転・スケーリングのみ適用（平行移動なし）
    Eigen::Matrix2d scale_matrix;
    scale_matrix << transform_matrix(0, 0), transform_matrix(0, 1), transform_matrix(1, 0),
      transform_matrix(1, 1);
    Eigen::Vector2d transformed_vel = scale_matrix * Eigen::Vector2d(ball_vel.x(), ball_vel.y());

    // 変換後の値を設定
    msg.ball_info.pose.x = transformed_pos.x();
    msg.ball_info.pose.y = transformed_pos.y();
    msg.ball_info.velocity.x = transformed_vel.x();
    msg.ball_info.velocity.y = transformed_vel.y();
    msg.ball_info.velocity_norm = transformed_vel.norm();
  }

  // 自チームロボットの座標変換
  for (auto & robot : msg.robot_info_ours) {
    if (robot.detected) {
      // 変換前の座標
      Eigen::Vector3d robot_pos(robot.pose.x, robot.pose.y, 1.0);
      Eigen::Vector3d robot_vel(robot.velocity.x, robot.velocity.y, 0.0);

      // 変換行列を適用
      Eigen::Vector3d transformed_pos = transform_matrix * robot_pos;

      // 速度は回転・スケーリングのみ適用（平行移動なし）
      Eigen::Matrix2d scale_matrix;
      scale_matrix << transform_matrix(0, 0), transform_matrix(0, 1), transform_matrix(1, 0),
        transform_matrix(1, 1);
      Eigen::Vector2d transformed_vel =
        scale_matrix * Eigen::Vector2d(robot_vel.x(), robot_vel.y());

      // 変換後の値を設定
      robot.pose.x = transformed_pos.x();
      robot.pose.y = transformed_pos.y();
      robot.velocity.x = transformed_vel.x();
      robot.velocity.y = transformed_vel.y();
      robot.velocity_norm = transformed_vel.norm();
      robot.pose.theta += M_PI_2;
    }
  }

  // 相手チームロボットの座標変換
  for (auto & robot : msg.robot_info_theirs) {
    if (robot.detected) {
      // 変換前の座標
      Eigen::Vector3d robot_pos(robot.pose.x, robot.pose.y, 1.0);
      Eigen::Vector3d robot_vel(robot.velocity.x, robot.velocity.y, 0.0);

      // 変換行列を適用
      Eigen::Vector3d transformed_pos = transform_matrix * robot_pos;

      // 速度は回転・スケーリングのみ適用（平行移動なし）
      Eigen::Matrix2d scale_matrix;
      scale_matrix << transform_matrix(0, 0), transform_matrix(0, 1), transform_matrix(1, 0),
        transform_matrix(1, 1);
      Eigen::Vector2d transformed_vel =
        scale_matrix * Eigen::Vector2d(robot_vel.x(), robot_vel.y());

      // 変換後の値を設定
      robot.pose.x = transformed_pos.x();
      robot.pose.y = transformed_pos.y();
      robot.velocity.x = transformed_vel.x();
      robot.velocity.y = transformed_vel.y();
      robot.velocity_norm = transformed_vel.norm();
    }
  }

  // ボール配置ターゲットの変換
  if (
    msg.play_situation.command.value == crane_msgs::msg::PlaySituation::OUR_BALL_PLACEMENT ||
    msg.play_situation.command.value == crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT) {
    // placement_positionフィールドが存在する場合
    if (
      msg.play_situation.placement_position.x != 0.0 ||
      msg.play_situation.placement_position.y != 0.0) {
      Eigen::Vector3d target_pos(
        msg.play_situation.placement_position.x, msg.play_situation.placement_position.y, 1.0);
      Eigen::Vector3d transformed_target = transform_matrix * target_pos;
      msg.play_situation.placement_position.x = transformed_target.x();
      msg.play_situation.placement_position.y = transformed_target.y();
    }
  }
}

crane_msgs::msg::WorldModel WorldModelDataProvider::getMsg()
{
  crane_msgs::msg::WorldModel msg;
  msg.is_yellow = (game_data.our_color == Color::YELLOW);
  msg.on_positive_half = on_positive_half;
  msg.is_emplace_positive_side = is_emplace_positive_side;
  msg.our_max_allowed_bots = game_data.our_max_allowed_bots;
  msg.their_max_allowed_bots = game_data.their_max_allowed_bots;

  msg.ball_info = data.ball_info;

  for (const auto & robot : data.robot_info[static_cast<uint8_t>(game_data.our_color)]) {
    msg.robot_info_ours.emplace_back(robot);
  }
  for (const auto & robot : data.robot_info[static_cast<uint8_t>(game_data.their_color)]) {
    msg.robot_info_theirs.emplace_back(robot);
  }

  // 変換行列がIdentityでないときは変換を適用
  if (not transform_matrix.isIdentity()) {
    // フィールドサイズの変換
    crane_msgs::msg::FieldSize field_info;
    // 半分のコートを90度回転して使っている
    field_info.x = game_data.field_h;
    field_info.y = game_data.field_w * 0.5;
    msg.field_info = field_info;

    // 順当に半分サイズ
    crane_msgs::msg::FieldSize penalty_area_size;
    penalty_area_size.x = game_data.penalty_area_h * 0.5;
    penalty_area_size.y = game_data.penalty_area_w * 0.5;
    msg.penalty_area_size = penalty_area_size;

    // 順当に半分サイズ
    crane_msgs::msg::FieldSize goal_size;
    goal_size.x = game_data.goal_h * 0.5;
    goal_size.y = game_data.goal_w * 0.5;
    msg.goal_size = goal_size;

    // 座標変換を適用
    applyTransformation(msg);
  } else {
    // 通常モード - 変換なし
    crane_msgs::msg::FieldSize field_info;
    field_info.x = game_data.field_w;
    field_info.y = game_data.field_h;
    msg.field_info = field_info;

    crane_msgs::msg::FieldSize penalty_area_size;
    penalty_area_size.x = game_data.penalty_area_h;
    penalty_area_size.y = game_data.penalty_area_w;
    msg.penalty_area_size = penalty_area_size;

    crane_msgs::msg::FieldSize goal_size;
    goal_size.x = game_data.goal_h;
    goal_size.y = game_data.goal_w;
    msg.goal_size = goal_size;
  }

  msg.our_goalie_id = game_data.our_goalie_id;
  msg.their_goalie_id = game_data.their_goalie_id;

  msg.play_situation = latest_play_situation;

  msg.header.stamp = rclcpp::Clock().now();
  return msg;
}
}  // namespace crane
