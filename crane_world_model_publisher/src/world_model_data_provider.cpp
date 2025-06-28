// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_data_provider.hpp"

#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
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

    // 2. 回転 (90度)
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    rotation_matrix(0, 0) = 0.0;
    rotation_matrix(0, 1) = -1.0;
    rotation_matrix(1, 0) = 1.0;
    rotation_matrix(1, 1) = 0.0;

    // 3. 原点を中心に戻す
    Eigen::Matrix3d translate_back = Eigen::Matrix3d::Identity();
    translate_back(0, 2) = 0.0;  // 新しい座標系の原点に配置

    // 全ての変換を合成 (右から左へ適用)
    matrix = translate_back * rotation_matrix * translate_to_origin;
  }
  return matrix;
}

WorldModelDataProvider::WorldModelDataProvider(rclcpp::Node & node)
: node(node), vis_data_handler(node)
{
  using std::chrono_literals::operator""ms;

  vision_processor_ = std::make_unique<VisionDataProcessor>(node);
  tracker_processor_ = std::make_unique<TrackerDataProcessor>(node);

  area_mask.min_corner() << -20., -10.;
  area_mask.max_corner() << 20., 10.;

  tracker_processor_->setAreaMask(area_mask);

  vision_processor_->setVisualizationHandler(
    [this](const SSL_GeometryData & geometry_data, bool half_court_mode) {
      vis_data_handler.flushGeometryVisualization(geometry_data, half_court_mode);
    });

  vision_processor_->setGeometryUpdateHandler([this]() {
    // Update geometry data whenever vision geometry is received
    updateGeometryIfNeeded();
  });

  udp_timer = node.create_wall_timer(10ms, std::bind(&WorldModelDataProvider::on_udp_timer, this));

  for (int i = 0; i < 20; i++) {
    crane_msgs::msg::RobotInfo info;
    info.vision_detected = false;
    info.feedback_detected = false;
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
          robot_info.feedback_detected = true;
          robot_info.last_feedback_detection_stamp = now;
          if (not robot_info.vision_detected) {
            try {
              // odom_speedはグローバル座標系
              robot_info.velocity.x = feedback->odom_speed[0];
              robot_info.velocity.y = feedback->odom_speed[1];
              robot_info.velocity_norm = std::hypot(robot_info.velocity.x, robot_info.velocity.y);
              // feedbackは100Hz
              // robot_info.pose.x += robot_info.velocity.x * 0.01;
              // robot_info.pose.y += robot_info.velocity.y * 0.01;
              robot_info.pose.x = feedback->odom[0];
              robot_info.pose.y = feedback->odom[1];
              // yaw_angleはdeg
              using boost::math::constants::degree;
              robot_info.pose.theta = feedback->yaw_angle * degree<double>();
            } catch (...) {
              std::cout << "feedback->odom_speed has noe element" << std::endl;
            }
          }
        } else {
          try {
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][robot.id].feedback_detected =
              false;
          } catch (...) {
            std::cout << "aaaaaaaaaa element" << std::endl;
          }
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
      vis_data_handler.flushRefereeVisualization(msg, game_data.field_w, game_data.field_h);
      CraneVisualizerBuffer::publish();
    });
}

auto WorldModelDataProvider::on_udp_timer() -> void
{
  tracker_processor_->processTrackerPackets();
  vision_processor_->processVisionPackets();

  // Check if geometry needs to be updated after processing vision packets
  if (!geometry_initialized) {
    updateGeometryIfNeeded();
  }
}

// アフィン変換行列を設定するメソッド
auto WorldModelDataProvider::setTransformInfo(bool enable, bool is_positive_side) -> void
{
  half_court_practice_mode = enable;
  half_court_is_positive_side = is_positive_side;

  // Try to update geometry immediately if vision data is already available
  updateGeometryIfNeeded();
}

auto WorldModelDataProvider::updateGeometryIfNeeded() -> void
{
  // Check if vision processor has valid geometry data (non-zero field dimensions)
  double field_w = vision_processor_->getFieldWidth();
  double field_h = vision_processor_->getFieldHeight();

  if (field_w <= 0.0 || field_h <= 0.0) {
    // Vision geometry not yet available, skip update
    return;
  }

  // Check if geometry has actually changed to avoid unnecessary updates
  bool geometry_changed = !geometry_initialized || std::abs(game_data.field_w - field_w) > 1e-6 ||
                          std::abs(game_data.field_h - field_h) > 1e-6;

  // Update geometry data from vision processor
  game_data.field_w = field_w;
  game_data.field_h = field_h;
  game_data.goal_w = vision_processor_->getGoalWidth();
  game_data.goal_h = vision_processor_->getGoalHeight();
  game_data.penalty_area_w = vision_processor_->getPenaltyAreaWidth();
  game_data.penalty_area_h = vision_processor_->getPenaltyAreaHeight();

  // Log geometry update for debugging
  if (geometry_changed) {
    RCLCPP_INFO(
      node.get_logger(),
      "Field geometry %s: field=%.3fx%.3f, goal=%.3fx%.3f, penalty_area=%.3fx%.3f",
      geometry_initialized ? "updated" : "initialized", game_data.field_w, game_data.field_h,
      game_data.goal_w, game_data.goal_h, game_data.penalty_area_w, game_data.penalty_area_h);
  }

  transform_matrix =
    createTransformMatrix(half_court_practice_mode, half_court_is_positive_side, game_data.field_w);

  constexpr double OFFSET = 0.3;
  if (half_court_practice_mode) {
    area_mask.min_corner() << -0.5 * game_data.field_h - OFFSET, -0.25 * game_data.field_w - OFFSET;
    area_mask.max_corner() << 0.5 * game_data.field_h + OFFSET, 0.25 * game_data.field_w + OFFSET;
  } else {
    area_mask.min_corner() << -0.5 * game_data.field_w - OFFSET, -0.5 * game_data.field_h - OFFSET;
    area_mask.max_corner() << 0.5 * game_data.field_w + OFFSET, 0.5 * game_data.field_h + OFFSET;
  }

  tracker_processor_->setAreaMask(area_mask);
  tracker_processor_->setTransformMatrix(transform_matrix);

  geometry_initialized = true;
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
    Eigen::Vector3d ball_pos(msg.ball_info.position.x, msg.ball_info.position.y, 1.0);
    Eigen::Vector3d ball_vel(msg.ball_info.velocity.x, msg.ball_info.velocity.y, 0.0);

    // 変換行列を適用
    Eigen::Vector3d transformed_pos = transform_matrix * ball_pos;

    // 速度は回転・スケーリングのみ適用（平行移動なし）
    Eigen::Matrix2d scale_matrix;
    scale_matrix << transform_matrix(0, 0), transform_matrix(0, 1), transform_matrix(1, 0),
      transform_matrix(1, 1);
    Eigen::Vector2d transformed_vel = scale_matrix * Eigen::Vector2d(ball_vel.x(), ball_vel.y());

    // 変換後の値を設定
    msg.ball_info.position.x = transformed_pos.x();
    msg.ball_info.position.y = transformed_pos.y();
    msg.ball_info.velocity.x = transformed_vel.x();
    msg.ball_info.velocity.y = transformed_vel.y();
    msg.ball_info.velocity_norm = transformed_vel.norm();
    // Z座標と速度は変換しない（2D変換のため）
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

  // マスクでロボットをフィルタリング(マスク外を削除)
  ranges::actions::remove_if(msg.robot_info_ours, [&](const auto & robot) {
    return not isInBox(area_mask, Point{robot.pose.x, robot.pose.y});
  });
  ranges::actions::remove_if(msg.robot_info_theirs, [&](const auto & robot) {
    return not isInBox(area_mask, Point{robot.pose.x, robot.pose.y});
  });

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

  // Get ball info from tracker processor (primary) or vision processor (fallback)
  if (tracker_processor_->hasTrackerUpdated()) {
    msg.ball_info = tracker_processor_->getBallInfo();
  } else {
    msg.ball_info = vision_processor_->getBallInfo();
  }

  // Merge robot info from tracker and vision processors
  auto tracker_robots_0 = tracker_processor_->getRobotInfo(0);
  auto tracker_robots_1 = tracker_processor_->getRobotInfo(1);
  auto vision_robots_0 = vision_processor_->getRobotInfo(0);
  auto vision_robots_1 = vision_processor_->getRobotInfo(1);

  // Update data.robot_info with feedback data that was processed in constructor subscriptions
  for (size_t i = 0; i < tracker_robots_0.size() && i < data.robot_info[0].size(); ++i) {
    auto merged_robot = tracker_robots_0[i];
    merged_robot.feedback_detected = data.robot_info[0][i].feedback_detected;
    merged_robot.ball_sensor = data.robot_info[0][i].ball_sensor;
    merged_robot.last_ball_sensor_stamp = data.robot_info[0][i].last_ball_sensor_stamp;
    merged_robot.last_feedback_detection_stamp =
      data.robot_info[0][i].last_feedback_detection_stamp;
    merged_robot.detected = merged_robot.vision_detected or merged_robot.feedback_detected;

    if (static_cast<uint8_t>(game_data.our_color) == 0) {
      if (ranges::contains(robot_ids_mask, merged_robot.id)) {
        msg.robot_info_theirs.emplace_back(merged_robot);
      } else {
        msg.robot_info_ours.emplace_back(merged_robot);
      }
    } else {
      msg.robot_info_theirs.emplace_back(merged_robot);
    }
  }

  for (size_t i = 0; i < tracker_robots_1.size() && i < data.robot_info[1].size(); ++i) {
    auto merged_robot = tracker_robots_1[i];
    merged_robot.feedback_detected = data.robot_info[1][i].feedback_detected;
    merged_robot.ball_sensor = data.robot_info[1][i].ball_sensor;
    merged_robot.last_ball_sensor_stamp = data.robot_info[1][i].last_ball_sensor_stamp;
    merged_robot.last_feedback_detection_stamp =
      data.robot_info[1][i].last_feedback_detection_stamp;
    merged_robot.detected = merged_robot.vision_detected or merged_robot.feedback_detected;

    if (static_cast<uint8_t>(game_data.our_color) == 1) {
      if (ranges::contains(robot_ids_mask, merged_robot.id)) {
        msg.robot_info_theirs.emplace_back(merged_robot);
      } else {
        msg.robot_info_ours.emplace_back(merged_robot);
      }
    } else {
      msg.robot_info_theirs.emplace_back(merged_robot);
    }
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

    // Validation warning for invalid geometry (half-court mode)
    if (game_data.field_w <= 0.0 || game_data.field_h <= 0.0) {
      static rclcpp::Time last_warning_time = rclcpp::Clock(RCL_ROS_TIME).now();
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();
      // Warn every 5 seconds to avoid spam
      if ((now - last_warning_time).seconds() > 5.0) {
        RCLCPP_WARN(
          node.get_logger(),
          "Invalid field geometry in WorldModel (half-court mode): field=%.3fx%.3f, "
          "goal=%.3fx%.3f, penalty_area=%.3fx%.3f",
          game_data.field_w, game_data.field_h, game_data.goal_w, game_data.goal_h,
          game_data.penalty_area_w, game_data.penalty_area_h);
        last_warning_time = now;
      }
    }

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

    // Validation warning for invalid geometry
    if (game_data.field_w <= 0.0 || game_data.field_h <= 0.0) {
      static rclcpp::Time last_warning_time = rclcpp::Clock(RCL_ROS_TIME).now();
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();
      // Warn every 5 seconds to avoid spam
      if ((now - last_warning_time).seconds() > 5.0) {
        RCLCPP_WARN(
          node.get_logger(),
          "Invalid field geometry in WorldModel: field=%.3fx%.3f, goal=%.3fx%.3f, "
          "penalty_area=%.3fx%.3f",
          game_data.field_w, game_data.field_h, game_data.goal_w, game_data.goal_h,
          game_data.penalty_area_w, game_data.penalty_area_h);
        last_warning_time = now;
      }
    }
  }

  msg.our_goalie_id = game_data.our_goalie_id;
  msg.their_goalie_id = game_data.their_goalie_id;

  msg.play_situation = latest_play_situation;

  // Vision遅延情報をDelayCheckpointに追加
  if (
    vision_processor_->getLastVisionTCapture() > 0.0 &&
    vision_processor_->getLastVisionTSent() > 0.0) {
    auto now = rclcpp::Clock().now();
    std::string vision_delay_info = DelayMonitorWrapper::formatVisionDelayInfo(
      vision_processor_->getLastVisionTCapture(), vision_processor_->getLastVisionTSent(), now);

    DelayMonitorWrapper::addDelayCheckpoint(
      msg.delay_checkpoints, "vision_timestamps", vision_delay_info);
  }

  msg.header.stamp = rclcpp::Clock().now();
  return msg;
}
}  // namespace crane
