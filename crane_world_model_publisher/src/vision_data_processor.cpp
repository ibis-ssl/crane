// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/vision_data_processor.hpp"

#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <crane_geometry/geometry_operations.hpp>
#include <string>
#include <vector>
#include <map>
#include <cmath>

using VisionColor = crane::VisionDataProcessor::Color;

namespace crane
{
VisionDataProcessor::VisionDataProcessor(rclcpp::Node & node)
: node_(node)
{
  node_.declare_parameter("vision_address", "224.5.23.2");
  node_.declare_parameter("vision_port", 10020);

  // 新しいアーキテクチャによるコンポーネント初期化
  vision_receiver_ = std::make_shared<VisionPacketReceiver>(node_);
  vision_converter_ = std::make_shared<VisionDataConverter>(node_);

  // Vision receiver initialization
  std::string vision_address = node_.get_parameter("vision_address").get_value<std::string>();
  int vision_port = node_.get_parameter("vision_port").get_value<int>();
  vision_receiver_->start(vision_address, vision_port);

  // コールバック設定
  setupVisionCallbacks();

  last_prediction_time_ = node_.get_clock()->now();

  for (int i = 0; i < 20; i++) {
    crane_msgs::msg::RobotInfo info;
    info.vision_detected = false;
    info.feedback_detected = false;
    info.internal_tracker_detected = false;
    info.detected = false;
    info.id = i;
    robot_info_[0].emplace_back(info);
    robot_info_[1].emplace_back(info);
  }
}

auto VisionDataProcessor::processVisionPackets() -> void
{
  auto current_time = node_.get_clock()->now();
  double dt = (current_time - last_prediction_time_).seconds();

  if (dt > 0.0) {
    last_prediction_time_ = current_time;
  }

  // 新しいアーキテクチャによるパケット処理
  if (vision_receiver_) {
    vision_receiver_->processIncomingPackets();
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

  // Notify geometry update handler
  if (geometry_update_handler_) {
    geometry_update_handler_();
  }
}

auto VisionDataProcessor::visionDetectionCallback(const SSL_DetectionFrame & detection_frame)
  -> void
{
  // Visionパケットタイムスタンプを保存
  last_t_capture_ = detection_frame.t_capture();
  last_t_sent_ = detection_frame.t_sent();

  int balls_size = detection_frame.balls().size();
  auto now = node_.now();

  // 各フレーム開始時に全ロボットのvision_detectedフラグをリセット
  for (auto & team : robot_info_) {
    for (auto & robot : team) {
      robot.vision_detected = false;
      robot.internal_tracker_detected = false;
    }
  }

  if (balls_size > 0) {
    last_ball_detect_time_ = now;

    Vector3 ball_position;
    ball_position(0) = detection_frame.balls().at(0).x() * 0.001;
    ball_position(1) = detection_frame.balls().at(0).y() * 0.001;
    ball_position(2) =
      detection_frame.balls().at(0).has_z() ? detection_frame.balls().at(0).z() * 0.001 : 0.0;

    // Vision専用実装: ボール情報を直接更新
    ball_info_.vision.stamp = now;
    ball_info_.vision.pos.x = ball_position(0);
    ball_info_.vision.pos.y = ball_position(1);
    ball_info_.vision.pos.z = ball_position(2);
    ball_info_.detected = true;
    ball_info_.position.x = ball_position(0);
    ball_info_.position.y = ball_position(1);
    ball_info_.position.z = ball_position(2);
    
    // 簡易的なボール速度計算
    static std::pair<Vector3, rclcpp::Time> last_ball_data = {Vector3::Zero(), now};
    double dt = (now - last_ball_data.second).seconds();
    if (dt > 0.001) {  // 1ms以上の差分のみ計算
      Vector3 vel = (ball_position - last_ball_data.first) / dt;
      ball_info_.velocity.x = vel(0);
      ball_info_.velocity.y = vel(1);
      ball_info_.velocity.z = vel(2);
    }
    last_ball_data = {ball_position, now};
  } else {
    // ボールが検出されない場合の処理
    if (
      now.get_clock_type() == last_ball_detect_time_.get_clock_type() &&
      (now - last_ball_detect_time_).seconds() > 0.1) {
      ball_info_.detected = false;
      // 速度を0にリセット
      ball_info_.velocity.x = 0.0;
      ball_info_.velocity.y = 0.0;
      ball_info_.velocity.z = 0.0;
    }
  }

  // 黄チームロボット処理
  for (const auto & robot : detection_frame.robots_yellow()) {
    if (robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(robot.robot_id());
      double raw_orientation = robot.orientation();
      // SSL-Vision角度を正規化してCrane座標系に変換
      double transformed_angle = crane::normalizeAngle(raw_orientation);
      Vector3 robot_pose(robot.x() * 0.001, robot.y() * 0.001, transformed_angle);
      
      // デバッグ用：角度変換の確認（10秒間、全ロボット対象）
      static rclcpp::Time debug_start = node_.now();
      if ((node_.now() - debug_start).seconds() < 10.0) {
        RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
          "Robot %d (Yellow): SSL angle=%.3f, normalized=%.3f", 
          robot_id, raw_orientation, transformed_angle);
      }

      // チーム色判定：黄チームが味方かどうか
      
      // デバッグ：チーム色判定の詳細ログ
      static rclcpp::Time last_team_debug = node_.now();
      if ((node_.now() - last_team_debug).seconds() > 3.0) {
        RCLCPP_INFO(node_.get_logger(), 
          "Team Color Debug: our_team_color_=%s, processing YELLOW robot %d",
          (our_team_color_ == VisionColor::YELLOW) ? "YELLOW" : "BLUE", robot_id);
        last_team_debug = node_.now();
      }

      // Vision専用実装: ロボット座標を直接robot_info_に反映
      int team_index =
        (our_team_color_ == VisionColor::YELLOW) ? 0 : 1;  // 黄チームが味方なら0、敵なら1
        
      // デバッグ：格納先team_indexのログ
      RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "YELLOW Robot %d -> team_index=%d (%s)", robot_id, team_index, 
        (team_index == 0) ? "OUR" : "THEIR");
      if (robot_id < robot_info_[team_index].size()) {
        auto & robot = robot_info_[team_index][robot_id];
        robot.vision_detected = true;
        robot.detected = true;
        
        // Vision座標を直接設定（角度正規化）
        robot.pose.x = robot_pose(0);
        robot.pose.y = robot_pose(1);
        robot.pose.theta = crane::normalizeAngle(robot_pose(2));
        
        // 簡易的な速度計算（前フレームとの差分）
        static std::map<std::pair<int, uint8_t>, std::pair<Vector3, rclcpp::Time>> last_poses;
        auto key = std::make_pair(team_index, robot_id);
        
        if (last_poses.count(key)) {
          auto [last_pose, last_time] = last_poses[key];
          double dt = (now - last_time).seconds();
          if (dt > 0.001) {  // 1ms以上の差分のみ計算
            Point current_pos(robot_pose(0), robot_pose(1));
            Point last_pos(last_pose(0), last_pose(1));
            Point vel = (current_pos - last_pos) / dt;
            robot.velocity.x = vel.x();
            robot.velocity.y = vel.y();
            robot.velocity_norm = vel.norm();
            
            // 角速度計算（正規化使用）
            double angle_diff = crane::normalizeAngle(robot_pose(2) - last_pose(2));
            robot.velocity.theta = angle_diff / dt;
          }
        }
        
        last_poses[key] = {robot_pose, now};
        robot.last_tracker_detection_stamp = now;
      }
    }
  }

  // 青チームロボット処理
  for (const auto & robot : detection_frame.robots_blue()) {
    if (robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(robot.robot_id());
      double raw_orientation = robot.orientation();
      // SSL-Vision角度を正規化してCrane座標系に変換
      double transformed_angle = crane::normalizeAngle(raw_orientation);
      Vector3 robot_pose(robot.x() * 0.001, robot.y() * 0.001, transformed_angle);

      // デバッグ用：角度変換の確認（10秒間、全ロボット対象）
      static rclcpp::Time debug_start_blue = node_.now();
      if ((node_.now() - debug_start_blue).seconds() < 10.0) {
        RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
          "Robot %d (Blue): SSL angle=%.3f, normalized=%.3f", 
          robot_id, raw_orientation, transformed_angle);
      }

      // チーム色判定：青チームが味方かどうか
      
      // デバッグ：チーム色判定の詳細ログ
      static rclcpp::Time last_team_debug_blue = node_.now();
      if ((node_.now() - last_team_debug_blue).seconds() > 3.0) {
        RCLCPP_INFO(node_.get_logger(), 
          "Team Color Debug: our_team_color_=%s, processing BLUE robot %d",
          (our_team_color_ == VisionColor::YELLOW) ? "YELLOW" : "BLUE", robot_id);
        last_team_debug_blue = node_.now();
      }

      // Vision専用実装: ロボット座標を直接robot_info_に反映
      int team_index =
        (our_team_color_ == VisionColor::BLUE) ? 0 : 1;  // 青チームが味方なら0、敵なら1
        
      // デバッグ：格納先team_indexのログ
      RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
        "BLUE Robot %d -> team_index=%d (%s)", robot_id, team_index, 
        (team_index == 0) ? "OUR" : "THEIR");
      if (robot_id < robot_info_[team_index].size()) {
        auto & robot = robot_info_[team_index][robot_id];
        robot.vision_detected = true;
        robot.detected = true;
        
        // Vision座標を直接設定（角度正規化）
        robot.pose.x = robot_pose(0);
        robot.pose.y = robot_pose(1);
        robot.pose.theta = crane::normalizeAngle(robot_pose(2));
        
        // 簡易的な速度計算（前フレームとの差分）
        static std::map<std::pair<int, uint8_t>, std::pair<Vector3, rclcpp::Time>> last_poses;
        auto key = std::make_pair(team_index, robot_id);
        
        if (last_poses.count(key)) {
          auto [last_pose, last_time] = last_poses[key];
          double dt = (now - last_time).seconds();
          if (dt > 0.001) {  // 1ms以上の差分のみ計算
            Point current_pos(robot_pose(0), robot_pose(1));
            Point last_pos(last_pose(0), last_pose(1));
            Point vel = (current_pos - last_pos) / dt;
            robot.velocity.x = vel.x();
            robot.velocity.y = vel.y();
            robot.velocity_norm = vel.norm();
            
            // 角速度計算（正規化使用）
            double angle_diff = crane::normalizeAngle(robot_pose(2) - last_pose(2));
            robot.velocity.theta = angle_diff / dt;
          }
        }
        
        last_poses[key] = {robot_pose, now};
        robot.last_tracker_detection_stamp = now;
      }
    }
  }

  // Vision専用実装では不要（直接更新済み）
}

auto VisionDataProcessor::setOurTeamColor(Color color) -> void
{
  if (our_team_color_ != color) {
    RCLCPP_INFO(node_.get_logger(), 
      "setOurTeamColor called: changing from %s to %s",
      (our_team_color_ == Color::YELLOW) ? "YELLOW" : "BLUE",
      (color == Color::YELLOW) ? "YELLOW" : "BLUE");
  }
  our_team_color_ = color;
}

auto VisionDataProcessor::updateFriendlyRobotFeedback(
  uint8_t robot_id, const crane_msgs::msg::RobotFeedback & feedback) -> void
{
  // Vision専用実装: フィードバック情報を直接robot_info_に反映
  if (robot_id < robot_info_[0].size()) {  // 味方チーム（インデックス0）
    auto & robot = robot_info_[0][robot_id];
    robot.feedback_detected = true;
    robot.ball_sensor = feedback.ball_sensor;
    robot.last_ball_sensor_stamp = node_.now();
    robot.last_feedback_detection_stamp = node_.now();
    
    // Visionで検出されていない場合のみodometry情報を使用
    if (!robot.vision_detected) {
      robot.velocity.x = feedback.odom_speed[0];
      robot.velocity.y = feedback.odom_speed[1];
      robot.velocity_norm = std::hypot(robot.velocity.x, robot.velocity.y);
    }
  }
}

auto VisionDataProcessor::updateFriendlyRobotCommand(
  uint8_t robot_id, const Vector2 & cmd_vel, double cmd_omega) -> void
{
  // Vision専用実装: コマンド情報をrobot_info_に記録（必要に応じて）
  if (robot_id < robot_info_[0].size()) {  // 味方チーム（インデックス0）
    // 現在は特に処理不要（将来的に予測などに活用可能）
  }
}

auto VisionDataProcessor::updateRobotInfoWithEKFData() -> void
{
  // Vision専用実装では不要（visionDetectionCallbackで直接更新済み）
  // この関数は互換性のために残されているが、処理は行わない
}

auto VisionDataProcessor::setupVisionCallbacks() -> void
{
  // Vision packet callback setup
  vision_receiver_->setPacketCallback([this](const VisionPacket & packet) {
    has_vision_updated_ = true;
    
    // 直接検出パス：SSL_DetectionFrameを直接処理
    if (packet.packet.has_detection()) {
      visionDetectionCallback(packet.packet.detection());
    }
    
    // Geometry処理はコンバーター経由
    if (packet.packet.has_geometry()) {
      vision_converter_->processVisionPacket(packet);
    }
  });

  // Geometry data callback
  vision_converter_->setGeometryCallback([this](const FieldGeometry & geometry) {
    field_height_ = geometry.field_width;
    field_width_ = geometry.field_height;
    goal_height_ = geometry.goal_depth;
    goal_width_ = geometry.goal_width;
    penalty_area_height_ = geometry.penalty_area_height;
    penalty_area_width_ = geometry.penalty_area_width;

    // 可視化ハンドラーへの橋渡し（SSL_GeometryDataが必要）
    if (geometry_vis_handler_ && geometry.is_valid) {
      // 簡単な変換を実行（完全な実装では適切な変換が必要）
      SSL_GeometryData ssl_geometry;
      geometry_vis_handler_(ssl_geometry, false);
    }

    if (geometry_update_handler_) {
      geometry_update_handler_();
    }
  });

  // Ball detection callback
  vision_converter_->setBallDetectionCallback([this](const crane_msgs::msg::BallInfo & ball_info) {
    auto now = node_.now();
    last_ball_detect_time_ = now;

    Vector3 ball_position(ball_info.position.x, ball_info.position.y, ball_info.position.z);
    
    // Vision専用実装: ボール情報を直接更新
    ball_info_.vision.stamp = now;
    ball_info_.vision.pos.x = ball_position(0);
    ball_info_.vision.pos.y = ball_position(1);
    ball_info_.vision.pos.z = ball_position(2);
    ball_info_.detected = true;
    ball_info_.position.x = ball_position(0);
    ball_info_.position.y = ball_position(1);
    ball_info_.position.z = ball_position(2);
    
    // 簡易的なボール速度計算
    static std::pair<Vector3, rclcpp::Time> last_ball_data_callback = {Vector3::Zero(), now};
    double dt = (now - last_ball_data_callback.second).seconds();
    if (dt > 0.001) {  // 1ms以上の差分のみ計算
      Vector3 vel = (ball_position - last_ball_data_callback.first) / dt;
      ball_info_.velocity.x = vel(0);
      ball_info_.velocity.y = vel(1);
      ball_info_.velocity.z = vel(2);
    }
    last_ball_data_callback = {ball_position, now};
  });

  // Robot detection callback - 無効化（直接検出パスを使用）
  // FIXME: コンバーター経由のロボット検出は直接検出パスと競合するため一時的に無効化
  // vision_converter_->setRobotDetectionCallback(...);
}
}  // namespace crane
