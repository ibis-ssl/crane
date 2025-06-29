// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_

#include <Eigen/Dense>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_world_model_publisher/tracker_data_processor.hpp>
#include <crane_world_model_publisher/vision_data_processor.hpp>
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

  [[nodiscard]] auto available() const -> bool
  {
    return tracker_processor_->hasTrackerUpdated() || vision_processor_->hasVisionUpdated();
  }

  VisualizationDataHandler vis_data_handler;

  auto setRobotIDsMask(const std::vector<uint8_t> & ids) -> void { robot_ids_mask = ids; }

  auto setAreaMask(const Box & area) -> void { area_mask = area; }

  auto updateGeometryIfNeeded() -> void;

private:
  rclcpp::Node & node;

  std::unique_ptr<VisionDataProcessor> vision_processor_;

  std::unique_ptr<TrackerDataProcessor> tracker_processor_;

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

  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_robot_commands;

  std::vector<uint8_t> robot_ids_mask;

  Box area_mask;

  bool geometry_initialized = false;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
