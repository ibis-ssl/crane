// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_

#include <robocup_ssl_msgs/ssl_vision_detection_tracked.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <Eigen/Dense>  // Add this line
#include <crane_basics/multicast.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_world_model_publisher/visualization_data_handler.hpp>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/robots_status.hpp>
#include <string>
#include <vector>

namespace crane
{
class WorldModelDataProvider
{
public:
  explicit WorldModelDataProvider(rclcpp::Node & node);

  ~WorldModelDataProvider() = default;

  auto on_udp_timer() -> void;

  crane_msgs::msg::WorldModel getMsg();

  [[nodiscard]] auto available() const -> bool { return has_tracker_updated && has_vision_updated; }

  VisualizationDataHandler vis_data_handler;

  auto setTransformInfo(bool enable, bool is_positive_side) -> void;

  auto setRobotIDsMask(const std::vector<uint8_t> & ids) -> void { robot_ids_mask = ids; }

  auto setAreaMask(const Box & area) -> void { area_mask = area; }

private:
  rclcpp::Node & node;

  std::unique_ptr<multicast::MulticastReceiver> tracker_receiver;

  std::unique_ptr<multicast::MulticastReceiver> vision_receiver;

  rclcpp::TimerBase::SharedPtr udp_timer;

  enum class Color { BLUE, YELLOW };

  struct GameData
  {
    std::string team_name;

    Color our_color;

    Color their_color;

    int our_goalie_id;

    int their_goalie_id;

    int our_max_allowed_bots;

    int their_max_allowed_bots;

    double field_w;

    double field_h;

    double goal_w;

    double goal_h;

    double penalty_area_w;

    double penalty_area_h;
  } game_data;

  struct Data
  {
    double ball_placement_target_x;

    double ball_placement_target_y;

    std::vector<crane_msgs::msg::RobotInfo> robot_info[2];

    crane_msgs::msg::BallInfo ball_info;

    std::vector<bool> ball_sensor_detected;
  } data;

  bool on_positive_half;

  bool is_emplace_positive_side;

  // アフィン変換行列
  Eigen::Matrix3d transform_matrix = Eigen::Matrix3d::Identity();

  bool half_court_practice_mode;

  bool half_court_is_positive_side;

  // 座標変換を適用するメソッド
  auto applyTransformation(crane_msgs::msg::WorldModel & msg) -> void;

  bool has_tracker_updated = false;

  bool has_vision_updated = false;

  rclcpp::Time last_ball_detect_time;

  struct BallAnalysis
  {
    bool is_our_ball;

    bool is_their_ball;

    bool ball_event_detected;

    enum class BallEvent { NONE, OUR_BALL, THEIR_BALL };

    BallEvent last_ball_event;
  };

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr sub_play_situation;

  crane_msgs::msg::PlaySituation latest_play_situation;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr sub_robot_feedback;

  crane_msgs::msg::RobotFeedbackArray robot_feedback;

  rclcpp::Subscription<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr sub_robots_status_blue;

  rclcpp::Subscription<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr sub_robots_status_yellow;

  rclcpp::Subscription<robocup_ssl_msgs::msg::Referee>::SharedPtr sub_referee;

  std::vector<uint8_t> robot_ids_mask;

  Box area_mask;

  auto trackerCallback(const TrackedFrame & tracked_frame) -> void;

  auto visionGeometryCallback(const SSL_GeometryData & geometry_data) -> void;

  auto visionDetectionCallback(const SSL_DetectionFrame & detection_frame) -> void;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
