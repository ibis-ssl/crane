// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_

#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>

#include <Eigen/Dense>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/world_model.hpp>
// Simplified architecture - direct VisionStreamProcessor usage
#include <crane_world_model_publisher/vision_stream_processor.hpp>
#include <deque>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
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

  [[nodiscard]] auto available() const -> bool { return vision_processor_->hasVisionUpdated(); }

  auto setRobotIDsMask(const std::vector<uint8_t> & ids) -> void { robot_ids_mask = ids; }

  auto setAreaMask(const Box & area) -> void { area_mask = area; }

  auto setVisualizationCallbacks(
    std::function<void(const SSL_GeometryData &, bool)> geometry_callback,
    std::function<void(const robocup_ssl_msgs::msg::Referee &, double, double)> referee_callback)
    -> void;

  auto updateGeometryIfNeeded() -> void;

private:
  rclcpp::Node & node;

  std::function<void(const SSL_GeometryData &, bool)> geometry_visualization_callback_;
  std::function<void(const robocup_ssl_msgs::msg::Referee &, double, double)>
    referee_visualization_callback_;

  std::unique_ptr<VisionStreamProcessor> vision_processor_;

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

  std::vector<uint8_t> robot_ids_mask;

  Box area_mask;

  bool geometry_initialized = false;

  auto createFieldInfo() -> crane_msgs::msg::FieldSize;
  auto createPenaltyAreaInfo() -> crane_msgs::msg::FieldSize;
  auto createGoalInfo() -> crane_msgs::msg::FieldSize;

  auto mergeRobotInfo(
    const crane_msgs::msg::RobotInfo & vision_robot,
    const crane_msgs::msg::RobotInfo & feedback_robot) -> crane_msgs::msg::RobotInfo;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_DATA_PROVIDER_HPP_
