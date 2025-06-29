// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/vision_data_processor.hpp"

#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <string>
#include <vector>

using VisionColor = crane::VisionDataProcessor::Color;

namespace crane
{
VisionDataProcessor::VisionDataProcessor(
  rclcpp::Node & node, std::shared_ptr<TrackerServiceInterface> tracker_service) 
: node_(node), tracker_service_(tracker_service)
{
  node_.declare_parameter("vision_address", "224.5.23.2");
  node_.declare_parameter("vision_port", 10020);
  
  // 新しいアーキテクチャによるコンポーネント初期化
  vision_receiver_ = std::make_shared<VisionPacketReceiver>(node_);
  vision_converter_ = std::make_shared<VisionDataConverter>(node_);
  
  // TrackerServiceの初期化
  if (!tracker_service_) {
    // デフォルトのトラッカーサービスを作成
    auto container = std::make_shared<TrackerManagerContainer>(node_);
    tracker_service_ = std::make_shared<TrackerServiceImplementation>(container);
    owns_tracker_service_ = true;
  }
  
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
    if (tracker_service_) {
      tracker_service_->predict(dt);
      tracker_service_->removeOldTrackers();
    }
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

    // 状態推定はTrackerServiceに委譲、データの受け渡しのみ
    if (tracker_service_) {
      ball_info_ = tracker_service_->processBallDetection(ball_position, now);
    }

    ball_info_.vision.stamp = now;
    ball_info_.vision.pos.x = ball_position(0);
    ball_info_.vision.pos.y = ball_position(1);
    ball_info_.vision.pos.z = ball_position(2);
  } else {
    if (
      now.get_clock_type() == last_ball_detect_time_.get_clock_type() &&
      (now - last_ball_detect_time_).seconds() > 0.1) {
      ball_info_.detected = false;
    } else {
      if (tracker_service_) {
        auto best_tracker = tracker_service_->getBestBallTracker();
        if (best_tracker) {
          ball_info_ = best_tracker->getState();
        }
      }
    }
  }

  // 黄チームロボット処理
  for (const auto & robot : detection_frame.robots_yellow()) {
    if (robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(robot.robot_id());
      double raw_orientation = robot.orientation();
      Vector3 robot_pose(robot.x() * 0.001, robot.y() * 0.001, raw_orientation);

      // チーム色判定：黄チームが味方かどうか
      RobotTrackerType tracker_type = (our_team_color_ == VisionColor::YELLOW)
                                        ? RobotTrackerType::FRIENDLY
                                        : RobotTrackerType::ENEMY;

      // EKFトラッカー処理（重複検出を防ぐため、我々のチームカラーのみ処理）
      if (our_team_color_ == VisionColor::YELLOW && tracker_service_) {
        tracker_service_->processRobotDetection(robot_id, tracker_type, robot_pose, now);
      }

      // チーム色に応じたrobot_info_配列のvision_detectedフラグを設定
      int team_index =
        (our_team_color_ == VisionColor::YELLOW) ? 0 : 1;  // 黄チームが味方なら0、敵なら1
      if (robot_id < robot_info_[team_index].size()) {
        robot_info_[team_index][robot_id].vision_detected = true;
      }
    }
  }

  // 青チームロボット処理
  for (const auto & robot : detection_frame.robots_blue()) {
    if (robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(robot.robot_id());
      double raw_orientation = robot.orientation();
      Vector3 robot_pose(robot.x() * 0.001, robot.y() * 0.001, raw_orientation);

      // チーム色判定：青チームが味方かどうか
      RobotTrackerType tracker_type = (our_team_color_ == VisionColor::BLUE)
                                        ? RobotTrackerType::FRIENDLY
                                        : RobotTrackerType::ENEMY;

      // EKFトラッカー処理（重複検出を防ぐため、我々のチームカラーのみ処理）
      if (our_team_color_ == VisionColor::BLUE && tracker_service_) {
        tracker_service_->processRobotDetection(robot_id, tracker_type, robot_pose, now);
      }

      // チーム色に応じたrobot_info_配列のvision_detectedフラグを設定
      int team_index =
        (our_team_color_ == VisionColor::BLUE) ? 0 : 1;  // 青チームが味方なら0、敵なら1
      if (robot_id < robot_info_[team_index].size()) {
        robot_info_[team_index][robot_id].vision_detected = true;
      }
    }
  }

  // EKFフィルタリング後の状態をrobot_info_配列に統合
  updateRobotInfoWithEKFData();
}

auto VisionDataProcessor::updateFriendlyRobotFeedback(
  uint8_t robot_id, const crane_msgs::msg::RobotFeedback & feedback) -> void
{
  if (tracker_service_) {
    tracker_service_->updateRobotFeedback(robot_id, feedback);
  }
}

auto VisionDataProcessor::updateFriendlyRobotCommand(
  uint8_t robot_id, const Vector2 & cmd_vel, double cmd_omega) -> void
{
  if (tracker_service_) {
    tracker_service_->updateRobotCommand(robot_id, cmd_vel, cmd_omega);
  }
}

auto VisionDataProcessor::updateRobotInfoWithEKFData() -> void
{
  // EKFフィルタリング済みの全ロボット情報を取得
  if (!tracker_service_) {
    return;
  }
  
  auto ekf_robots = tracker_service_->getAllRobotInfo();

  // EKFデータをrobot_info_配列に統合
  for (const auto & ekf_robot : ekf_robots) {
    // チーム色とロボットIDで対応する robot_info を見つける
    int team_index = [&]() {
      if (our_team_color_ == Color::BLUE) {
        // 青チームが味方の場合
        return (ekf_robot.id < 20) ? 0 : 1;  // 0: 青チーム, 1: 黄チーム
      } else {
        // 黄チームが味方の場合
        return (ekf_robot.id < 20) ? 1 : 0;  // 0: 青チーム, 1: 黄チーム
      }
    }();

    if (team_index >= 0 && team_index < 2 && ekf_robot.id < robot_info_[team_index].size()) {
      auto & robot_info = robot_info_[team_index][ekf_robot.id];

      // EKFトラッカーが有効な場合（MIN_TRACKING_CONFIDENCE以上）
      // getAllRobotInfo()は既にconfidence > 0.2でフィルタリング済み
      robot_info.internal_tracker_detected = true;

      // Vision検出されているかチェック
      if (robot_info.vision_detected) {
        // EKFフィルタリング後の状態でrobot_infoを更新
        robot_info.pose.x = ekf_robot.pose.x;
        robot_info.pose.y = ekf_robot.pose.y;
        robot_info.pose.theta = ekf_robot.pose.theta;  // これが重要！EKFフィルタリング後の角度
        robot_info.velocity.x = ekf_robot.velocity.x;
        robot_info.velocity.y = ekf_robot.velocity.y;
        robot_info.velocity.theta = ekf_robot.velocity.theta;
        robot_info.velocity_norm = ekf_robot.velocity_norm;
      }
    }
  }
}

auto VisionDataProcessor::setupVisionCallbacks() -> void
{
  // Vision packet callback setup
  vision_receiver_->setPacketCallback(
    [this](const VisionPacket & packet) {
      has_vision_updated_ = true;
      vision_converter_->processVisionPacket(packet);
    });

  // Geometry data callback
  vision_converter_->setGeometryCallback(
    [this](const FieldGeometry & geometry) {
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
  vision_converter_->setBallDetectionCallback(
    [this](const crane_msgs::msg::BallInfo & ball_info) {
      auto now = node_.now();
      last_ball_detect_time_ = now;
      
      Vector3 ball_position(ball_info.position.x, ball_info.position.y, ball_info.position.z);
      if (tracker_service_) {
        ball_info_ = tracker_service_->processBallDetection(ball_position, now);
      }
      
      ball_info_.vision.stamp = now;
      ball_info_.vision.pos.x = ball_position(0);
      ball_info_.vision.pos.y = ball_position(1);
      ball_info_.vision.pos.z = ball_position(2);
    });

  // Robot detection callback
  vision_converter_->setRobotDetectionCallback(
    [this](int team_index, const std::vector<crane_msgs::msg::RobotInfo> & robot_infos) {
      auto now = node_.now();
      
      // 各フレーム開始時にvision_detectedフラグをリセット
      for (auto & team : robot_info_) {
        for (auto & robot : team) {
          robot.vision_detected = false;
          robot.internal_tracker_detected = false;
        }
      }
      
      for (const auto & robot_info : robot_infos) {
        uint8_t robot_id = robot_info.id;
        
        // チーム色判定と処理
        RobotTrackerType tracker_type = 
          (team_index == static_cast<int>(our_team_color_)) ? 
          RobotTrackerType::FRIENDLY : RobotTrackerType::ENEMY;
        
        Vector3 robot_pose(robot_info.pose.x, robot_info.pose.y, robot_info.pose.theta);
        
        // EKFトラッカー処理（自チームのみ）
        if (team_index == static_cast<int>(our_team_color_) && tracker_service_) {
          tracker_service_->processRobotDetection(robot_id, tracker_type, robot_pose, now);
        }
        
        // robot_info_配列のvision_detectedフラグ設定
        int info_team_index = (our_team_color_ == Color::BLUE) ? 
          (team_index == 0 ? 0 : 1) : (team_index == 0 ? 1 : 0);
        
        if (robot_id < robot_info_[info_team_index].size()) {
          robot_info_[info_team_index][robot_id].vision_detected = true;
        }
      }
      
      // EKFフィルタリング後の状態をrobot_info_配列に統合
      updateRobotInfoWithEKFData();
    });
}
}  // namespace crane
