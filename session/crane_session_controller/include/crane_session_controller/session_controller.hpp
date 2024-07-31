// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_

#include <chrono>
#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <deque>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "visibility_control.h"

namespace crane
{
struct SessionCapacity
{
  std::string session_name;

  int selectable_robot_num;
};

class BallOwnerCalculator
{
public:
  explicit BallOwnerCalculator(const WorldModelWrapper::SharedPtr world_model)
  : world_model(world_model)
  {
  }
  void update()
  {
    if (not ball_owner) {
      ball_owner = world_model
                     ->getNearestRobotWithDistanceFromPoint(
                       world_model->ball.pos, world_model->ours.getAvailableRobots())
                     .first;
    }

    auto our_robots = world_model->ours.getAvailableRobots();
    std::vector<std::pair<std::shared_ptr<RobotInfo>, double>> scores;
    std::transform(
      our_robots.begin(), our_robots.end(), scores.begin(),
      [&](const std::shared_ptr<RobotInfo> & robot) {
        return std::make_pair(robot, calculateBallOwnerScore(robot));
      });
    std::sort(
      scores.begin(), scores.end(),
      [](
        const std::pair<std::shared_ptr<RobotInfo>, double> & a,
        const std::pair<std::shared_ptr<RobotInfo>, double> & b) { return a.second > b.second; });

    // トップがball_ownerでなかったらヒステリシスを考慮しつつ交代させる
  }

  [[nodiscard]] double calculateBallOwnerScore(const std::shared_ptr<RobotInfo> & robot) const
  {
    auto [min_slack, max_slack] =
      world_model->getMinMaxSlackInterceptPointAndSlackTime({robot}, 3.0, 0.1);
    // 3秒後にボールにたどり着けないロボットはスコアゼロ
    double score = max_slack.has_value() ? max_slack->second + 3.0 : 0.;
    return score;
  }

  void setNextOwner(uint8_t robot_id) { next_owner = world_model->getOurRobot(robot_id); }

private:
  std::shared_ptr<RobotInfo> ball_owner = nullptr;
  std::shared_ptr<RobotInfo> next_owner = nullptr;

  const WorldModelWrapper::SharedPtr world_model;
};

class SessionControllerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit SessionControllerComponent(const rclcpp::NodeOptions & options);

  void request(const std::string & situation, std::vector<uint8_t> selectable_robot_ids);

private:
  WorldModelWrapper::SharedPtr world_model;

  ConsaiVisualizerWrapper::SharedPtr visualizer;

  std::deque<crane_msgs::srv::RobotSelect::Request> query_queue;

  //  identifier: situation name,
  //    content: [ list of  [ pair of session name & selectable robot num]]
  std::unordered_map<std::string, std::vector<SessionCapacity>> robot_selection_priority_map;

  //  identifier :  event name, content : situation name
  std::unordered_map<std::string, std::string> event_map;

  rclcpp::Subscription<crane_msgs::msg::GameAnalysis>::SharedPtr game_analysis_sub;

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub;

  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr robot_commands_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timer_process_time_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr callback_process_time_pub;

  std::vector<PlannerBase::SharedPtr> available_planners;

  PlaySituationWrapper play_situation;

  rclcpp::TimerBase::SharedPtr timer;

  bool world_model_ready = false;

  std::shared_ptr<std::unordered_map<uint8_t, RobotRole>> robot_roles;
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
