// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_physics/ball_info.hpp>
#include <crane_physics/robot_info.hpp>
#include <crane_physics/travel_time.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <range/v3/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "delay_monitor_wrapper.hpp"
#include "play_situation_wrapper.hpp"

namespace crane
{
using RobotList = std::vector<std::shared_ptr<RobotInfo>>;

struct TeamInfo
{
  Box penalty_area;

  RobotList robots;

  uint32_t max_allowed_bots;

  uint8_t goalie_id;

  [[nodiscard]] auto getAvailableRobotsView(uint8_t my_id = 255, bool except_goalie = false) const
    -> decltype(auto)
  {
    return robots | ranges::views::filter([&](const auto & robot) {
             if (except_goalie) {
               return robot->available && robot->id != my_id && robot->id != goalie_id;
             } else {
               return robot->available && robot->id != my_id;
             }
           });
  }

  [[nodiscard]] auto getAvailableRobots(uint8_t my_id = 255, bool except_goalie = false) const
    -> RobotList
  {
    return getAvailableRobotsView(my_id, except_goalie) | ranges::to<std::vector>();
  }

  [[nodiscard]] auto getAvailableRobotIds(uint8_t my_id = 255, bool except_goalie = false) const
    -> std::vector<uint8_t>
  {
    return robots | ranges::views::filter([&](const auto & robot) {
             if (except_goalie) {
               return robot->available && robot->id != my_id && robot->id != goalie_id;
             } else {
               return robot->available && robot->id != my_id;
             }
           }) |
           ranges::views::transform([](const auto & robot) { return robot->id; }) |
           ranges::to<std::vector>();
  }
};

struct WorldModelWrapper
{
  using SharedPtr = std::shared_ptr<WorldModelWrapper>;

  using UniquePtr = std::unique_ptr<WorldModelWrapper>;

  explicit WorldModelWrapper(rclcpp::Node & node, bool setup_subscriber = true);

  auto update(const crane_msgs::msg::WorldModel & world_model) -> void;

  auto update(const crane_msgs::msg::GameAnalysis & msg) -> void { latest_msg.game_analysis = msg; }

  auto overwriteBallPos(Point pos, double z = 0.0) -> void
  {
    ball_.pos = pos;
    ball_.pos_z = z;
    // Ball構造体からBallInfoメッセージに同期
    ball_.toMsg(latest_msg.ball_info);
  }

  auto updateBallToMsg() -> void
  {
    // Ball構造体の現在状態をBallInfoメッセージに反映
    ball_.toMsg(latest_msg.ball_info);
  }

  [[nodiscard]] const auto & getMsg() const { return latest_msg; }

  auto & getEditableMsg() { return latest_msg; }

  [[nodiscard]] auto onPositiveHalf() const { return (latest_msg.on_positive_half); }

  [[nodiscard]] auto getOurSideSign() const { return onPositiveHalf() ? 1.0 : -1.0; }

  [[nodiscard]] auto isYellow() const { return (latest_msg.is_yellow); }

  [[nodiscard]] auto hasUpdated() const { return has_updated; }

  [[nodiscard]] auto isEmplacePositiveSide() const { return latest_msg.is_emplace_positive_side; }

  auto addCallback(std::function<void(void)> && callback_func) -> void
  {
    callbacks.emplace_back(callback_func);
  }

  [[nodiscard]] auto getRobot(RobotIdentifier robot) const
  {
    if (robot.is_ours) {
      return ours_.robots.at(robot.id);
    } else {
      return theirs_.robots.at(robot.id);
    }
  }

  [[nodiscard]] auto getOurRobot(uint8_t id) const { return ours_.robots.at(id); }

  [[nodiscard]] auto getTheirRobot(uint8_t id) const { return theirs_.robots.at(id); }

  [[nodiscard]] auto getOurMaxAllowedBots() const { return ours_.max_allowed_bots; }

  [[nodiscard]] auto getTheirMaxAllowedBots() const { return theirs_.max_allowed_bots; }

  [[nodiscard]] auto getDistanceFromRobotToBall(RobotIdentifier id) const -> double
  {
    return getDistanceFromRobot(id, ball_.pos);
  }

  [[nodiscard]] auto getDistanceFromRobotToBall(uint8_t our_id) const -> double
  {
    return getDistanceFromRobot({true, our_id}, ball_.pos);
  }

  [[nodiscard]] auto getSquareDistanceFromRobotToBall(RobotIdentifier id) const -> double
  {
    return getSquareDistanceFromRobot(id, ball_.pos);
  }

  [[nodiscard]] auto getSquareDistanceFromRobotToBall(uint8_t our_id) const -> double
  {
    return getSquareDistanceFromRobot({true, our_id}, ball_.pos);
  }

  [[nodiscard]] auto generateFieldPoints(float grid_size) const;

  [[nodiscard]] auto getDistanceFromRobot(RobotIdentifier id, const Point & point) const -> double
  {
    return (getRobot(id)->pose.pos - point).norm();
  }

  [[nodiscard]] auto getDistanceFromRobot(uint8_t our_id, const Point & point) const -> double
  {
    return (getOurRobot(our_id)->pose.pos - point).norm();
  }

  [[nodiscard]] auto getSquareDistanceFromRobot(RobotIdentifier id, const Point & point) const
    -> double
  {
    return (getRobot(id)->pose.pos - point).squaredNorm();
  }

  [[nodiscard]] auto getSquareDistanceFromRobot(uint8_t our_id, const Point & point) const -> double
  {
    return (getOurRobot(our_id)->pose.pos - point).squaredNorm();
  }

  [[nodiscard]] auto getDistanceFromBall(const Point & point) const -> double
  {
    return (ball_.pos - point).norm();
  }

  [[nodiscard]] auto getSquareDistanceFromBall(const Point & point) const -> double
  {
    return (ball_.pos - point).squaredNorm();
  }

  struct RobotWithDistance
  {
    RobotInfo::SharedPtr robot;
    double distance;
  };
  [[nodiscard]] auto getNearestRobotWithDistanceFromPoint(
    const Point & point, const RobotList & robots) const -> std::optional<RobotWithDistance>;

  [[nodiscard]] auto getNearestRobotWithDistanceFromSegment(
    const Segment & segment, const RobotList & robots) const -> std::optional<RobotWithDistance>;

  [[nodiscard]] auto getFieldMargin() const { return 0.3; }

  [[nodiscard]] auto getDefenseWidth() const
  {
    return ours_.penalty_area.max_corner().y() - ours_.penalty_area.min_corner().y();
  }

  [[nodiscard]] auto getDefenseHeight() const
  {
    return ours_.penalty_area.max_corner().x() - ours_.penalty_area.min_corner().x();
  }

  [[nodiscard]] auto getOurGoalPosts() const -> std::pair<Point, Point>
  {
    double x = getOurGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  [[nodiscard]] auto getTheirGoalPosts() const -> std::pair<Point, Point>
  {
    double x = getTheirGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  [[nodiscard]] auto getOurPenaltyArea() const { return ours_.penalty_area; }

  [[nodiscard]] auto getTheirPenaltyArea() const { return theirs_.penalty_area; }

  [[nodiscard]] auto getOurGoalCenter() const -> Point { return goal_; }

  [[nodiscard]] auto getTheirGoalCenter() const -> Point { return Point(-goal_.x(), goal_.y()); }

  [[nodiscard]] auto getBallPlacementTarget() const -> std::optional<Point>;

  // rule 8.4.3
  [[nodiscard]] auto getBallPlacementArea(double offset = 0.) const -> std::optional<Capsule>;

  [[nodiscard]] auto getOurGoalieId() const { return ours_.goalie_id; }

  [[nodiscard]] auto getTheirGoalieId() const { return theirs_.goalie_id; }

  /**
   *
   * @param from
   * @return {angle, width}
   */
  struct GoalAngleRange
  {
    double center_angle;
    double angle_width;
  };
  [[nodiscard]] auto getLargestGoalAngleRangeFromPoint(
    const Point from, const std::pair<Point, Point> & goal_posts,
    const RobotList & obstacle_robots) const -> GoalAngleRange;

  [[nodiscard]] auto getLargestGoalAngleRangeFromPoint(const Point from) const -> GoalAngleRange
  {
    return getLargestGoalAngleRangeFromPoint(
      from, getTheirGoalPosts(), theirs_.getAvailableRobots());
  }

  [[nodiscard]] auto getLargestOurGoalAngleRangeFromPoint(const Point from) const -> GoalAngleRange
  {
    return getLargestGoalAngleRangeFromPoint(from, getOurGoalPosts(), ours_.getAvailableRobots());
  }

  struct SlackTimeResult
  {
    double slack_time;

    Point intercept_point;

    std::shared_ptr<RobotInfo> robot;
  };

  [[nodiscard]] auto getBallSequence(double t_horizon, double t_step)
    -> std::vector<std::pair<Point, double>>;

  [[nodiscard]] auto getBallSlackTime(
    double time, const RobotList & robots, const double max_acc, const double max_vel)
    -> std::optional<SlackTimeResult>;

  [[nodiscard]] auto getSlackInterceptPointAndSlackTimeArray(
    const RobotList & robots, double t_horizon = 5.0, double t_step = 0.1,
    double slack_time_offset = 0.0, const double max_acc = 4.0, const double max_vel = 4.0,
    double distance_horizon = 100., double velocity_epsilon = 0.2) -> std::vector<SlackTimeResult>;

  [[nodiscard]] auto getMinMaxSlackInterceptPointAndSlackTime(
    const RobotList & robots, double t_horizon = 5.0, double t_step = 0.1,
    double slack_time_offset = 0.0, const double max_acc = 4.0, const double max_vel = 4.0,
    double distance_horizon = 100., double velocity_epsilon = 0.2)
    -> std::pair<std::optional<SlackTimeResult>, std::optional<SlackTimeResult>>;

  // Getter methods for accessing member variables
  [[nodiscard]] auto ours() const -> const TeamInfo & { return ours_; }
  [[nodiscard]] auto theirs() const -> const TeamInfo & { return theirs_; }
  [[nodiscard]] auto ball() const -> const Ball & { return ball_; }
  [[nodiscard]] auto fieldSize() const -> const Point & { return field_size_; }
  [[nodiscard]] auto penaltyAreaSize() const -> const Point & { return penalty_area_size_; }
  [[nodiscard]] auto goalSize() const -> const Point & { return goal_size_; }
  [[nodiscard]] auto goal() const -> const Point & { return goal_; }

private:
  class BallOwnerCalculator
  {
  public:
    explicit BallOwnerCalculator(WorldModelWrapper * world_model) : world_model(world_model) {}

    struct RobotWithScore
    {
      std::shared_ptr<RobotInfo> robot = nullptr;
      double min_slack = 100.;
      double min_slack_pos_distance = 100.;
      double max_slack = -100.;
      double score;
    };

    auto update() -> void;

    auto updateScore(bool our_team, double ball_distance_horizon) -> void;

    [[nodiscard]] auto calculateScore(
      const std::shared_ptr<RobotInfo> & robot, double ball_distance_horizon) const
      -> RobotWithScore;

    [[nodiscard]] auto getOurFrontier() const -> std::optional<RobotWithScore>
    {
      if (sorted_our_robots.empty()) {
        return std::nullopt;
      } else {
        return sorted_our_robots.front();
      }
    }

    [[nodiscard]] auto getTheirFrontier() const -> std::optional<RobotWithScore>
    {
      if (sorted_their_robots.empty()) {
        return std::nullopt;
      } else {
        return sorted_their_robots.front();
      }
    }

  private:
    std::vector<RobotWithScore> sorted_our_robots;

    std::vector<RobotWithScore> sorted_their_robots;

    WorldModelWrapper * world_model;

    std::uint8_t our_frontier = 255;
  } ball_owner_calculator;

  bool ball_owner_calculator_enabled = false;

public:
  auto setBallOwnerCalculatorEnabled(bool enabled = true) -> void
  {
    ball_owner_calculator_enabled = enabled;
  }

  [[nodiscard]] auto getOurFrontier() const -> std::optional<BallOwnerCalculator::RobotWithScore>
  {
    return ball_owner_calculator.getOurFrontier();
  }

  [[nodiscard]] auto getTheirFrontier() const -> std::optional<BallOwnerCalculator::RobotWithScore>
  {
    return ball_owner_calculator.getTheirFrontier();
  }

  auto getPenaltyAreaCorners(double offset_x, double offset_y) const
    -> std::tuple<Point, Point, Point, Point>;
  auto getOurAreaCorners() const -> std::tuple<Point, Point, Point, Point>;

  auto getIntersectionOurPenaltyArea(
    const Segment & target_segment, double offset_x, double offset_y) const -> std::optional<Point>;

  auto getForwardDefenseRatio(const Segment & ball_line) const -> std::optional<double>;

  class PointChecker
  {
  public:
    explicit PointChecker(WorldModelWrapper::SharedPtr & world_model)
    : world_model(world_model.get())
    {
    }

    explicit PointChecker(WorldModelWrapper * world_model) : world_model(world_model) {}

    [[nodiscard]] auto isFieldInside(const Point & p, double offset = 0.) const -> bool;

    auto addFieldInsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back([this, offset](const Point & p) { return isFieldInside(p, offset); });
    }

    auto addFieldOutsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isFieldInside(p, offset); });
    }

    [[nodiscard]] auto isBallPlacementArea(const Point & p, double offset = 0.) const -> bool;

    auto addBallPlacementAreaInsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return isBallPlacementArea(p, offset); });
    }

    auto addBallPlacementAreaOutsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isBallPlacementArea(p, offset); });
    }

    [[nodiscard]] auto isEnemyPenaltyArea(const Point & p, double offset = 0.) const -> bool;

    auto addEnemyPenaltyAreaInsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return isEnemyPenaltyArea(p, offset); });
    }

    auto addEnemyPenaltyAreaOutsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isEnemyPenaltyArea(p, offset); });
    }

    [[nodiscard]] auto isFriendPenaltyArea(const Point & p, double offset = 0.) const -> bool;

    auto addFriendPenaltyAreaInsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return isFriendPenaltyArea(p, offset); });
    }

    auto addFriendPenaltyAreaOutsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isFriendPenaltyArea(p, offset); });
    }

    [[nodiscard]] auto isPenaltyArea(const Point & p, double offset = 0.) const -> bool;

    auto addPenaltyAreaInsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back([this, offset](const Point & p) { return isPenaltyArea(p, offset); });
    }

    auto addPenaltyAreaOutsideChecker(double offset = 0.) -> void
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isPenaltyArea(p, offset); });
    }

    [[nodiscard]] auto isInOurHalf(const Point & p, double offset = 0.) const -> bool
    {
      return p.x() * world_model->getOurSideSign() > offset;
    }

    auto addInOurHalfChecker(double offset = 0.) -> void
    {
      checkers.emplace_back([this, offset](const Point & p) { return isInOurHalf(p, offset); });
    }

    enum class Rule {
      EQUAL_TO,
      NOT_EQUAL_TO,
      LESS_THAN,
      GREATER_THAN,
      LESS_THAN_OR_EQUAL_TO,
      GREATER_THAN_OR_EQUAL_TO,
    };

    [[nodiscard]] auto checkDistance(
      const Point & p, const Point & target, double threshold, const Rule rule) const -> bool
    {
      double distance = (p - target).norm();
      switch (rule) {
        case Rule::EQUAL_TO:
          return distance == threshold;
        case Rule::NOT_EQUAL_TO:
          return distance != threshold;
        case Rule::LESS_THAN:
          return distance < threshold;
        case Rule::GREATER_THAN:
          return distance > threshold;
        case Rule::LESS_THAN_OR_EQUAL_TO:
          return distance <= threshold;
        case Rule::GREATER_THAN_OR_EQUAL_TO:
          return distance >= threshold;
        default:
          return false;
      }
    }

    auto addDistanceChecker(const Point & target, double threshold, const Rule rule) -> void
    {
      checkers.emplace_back([this, target, threshold, rule](const Point & p) {
        return checkDistance(p, target, threshold, rule);
      });
    }

    [[nodiscard]] auto checkDistanceFromBall(
      const Point & p, double threshold, const Rule rule) const -> bool
    {
      return checkDistance(p, world_model->ball_.pos, threshold, rule);
    }

    auto addDistanceFromBallChecker(double threshold, const Rule rule) -> void
    {
      checkers.emplace_back([this, threshold, rule](const Point & p) {
        return checkDistanceFromBall(p, threshold, rule);
      });
    }

    [[nodiscard]] auto checkDistanceFromRobot(
      const Point & p, RobotIdentifier id, double threshold, const Rule rule) const -> bool
    {
      return checkDistance(p, world_model->getRobot(id)->pose.pos, threshold, rule);
    }

    auto addDistanceFromRobotChecker(RobotIdentifier id, double threshold, const Rule rule) -> void
    {
      checkers.emplace_back([this, id, threshold, rule](const Point & p) {
        return checkDistanceFromRobot(p, id, threshold, rule);
      });
    }

    [[nodiscard]] auto checkDistanceFromRobot(
      const Point & p, std::shared_ptr<RobotInfo> robot, double threshold, const Rule rule) const
      -> bool
    {
      return checkDistance(p, robot->pose.pos, threshold, rule);
    }

    [[nodiscard]] auto checkDistanceFromRobots(
      const Point & p, const RobotList & robots, double threshold, const Rule rule) const -> bool
    {
      for (auto robot : robots) {
        if (not checkDistance(p, robot->pose.pos, threshold, rule)) {
          return false;
        }
      }
      return true;
    }

    auto addDistanceFromRobotsChecker(const RobotList & robots, double threshold, const Rule rule)
      -> void
    {
      checkers.emplace_back([this, robots, threshold, rule](const Point & p) {
        return checkDistanceFromRobots(p, robots, threshold, rule);
      });
    }

    [[nodiscard]] auto checkDistanceFromOurRobots(
      const Point & p, double threshold, const Rule rule) const -> bool
    {
      return checkDistanceFromRobots(p, world_model->ours_.getAvailableRobots(), threshold, rule);
    }

    auto addDistanceFromOurRobotsChecker(double threshold, const Rule rule) -> void
    {
      checkers.emplace_back([this, threshold, rule](const Point & p) {
        return checkDistanceFromOurRobots(p, threshold, rule);
      });
    }

    [[nodiscard]] auto checkDistanceFromTheirRobots(
      const Point & p, double threshold, const Rule rule) const -> bool
    {
      return checkDistanceFromRobots(p, world_model->theirs_.getAvailableRobots(), threshold, rule);
    }

    auto addDistanceFromTheirRobotsChecker(double threshold, const Rule rule) -> void
    {
      checkers.emplace_back([this, threshold, rule](const Point & p) {
        return checkDistanceFromTheirRobots(p, threshold, rule);
      });
    }

    auto addCustomChecker(std::function<bool(const Point &)> checker) -> void
    {
      checkers.emplace_back(checker);
    }

    auto operator()(const Point & p) const -> bool
    {
      return std::ranges::all_of(checkers, [p](auto & check) { return check(p); });
    }

    static auto buildStandard(WorldModelWrapper::SharedPtr world_model) -> PointChecker
    {
      PointChecker checker(world_model);
      checker.addFieldInsideChecker();
      checker.addPenaltyAreaOutsideChecker();
      checker.addBallPlacementAreaOutsideChecker();
      return checker;
    }

  private:
    WorldModelWrapper * world_model;

    std::vector<std::function<bool(const Point &)>> checkers;
  } point_checker;

  // ===== 遅延監視関連メソッド =====

  auto addDelayCheckpoint(const std::string & name, const std::string & value = "") -> void
  {
    DelayMonitorWrapper::addDelayCheckpoint(latest_msg.delay_checkpoints, name, value);
  }

  auto clearDelayCheckpoints() -> void
  {
    DelayMonitorWrapper::clearCheckpoints(latest_msg.delay_checkpoints);
  }

  auto calculateDelayMs(const std::string & start_name, const std::string & end_name) -> double
  {
    return DelayMonitorWrapper::calculateDelayMs(
      latest_msg.delay_checkpoints, start_name, end_name);
  }

  auto calculateTotalDelayMs(const std::string & end_name) -> double
  {
    return DelayMonitorWrapper::calculateTotalDelayMs(latest_msg.delay_checkpoints, end_name);
  }

  auto getDelayCheckpointsString() -> std::string
  {
    return DelayMonitorWrapper::checkpointsToString(latest_msg.delay_checkpoints);
  }

  auto mergeDelayCheckpoints(const crane_msgs::msg::DelayCheckpoints & source_checkpoints) -> void
  {
    DelayMonitorWrapper::mergeCheckpoints(latest_msg.delay_checkpoints, source_checkpoints);
  }

  [[nodiscard]] auto getDelayCheckpoints() const -> const crane_msgs::msg::DelayCheckpoints &
  {
    return latest_msg.delay_checkpoints;
  }

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber;

  std::vector<std::function<void(void)>> callbacks;

  crane_msgs::msg::WorldModel latest_msg;

  bool has_updated = false;

  // Private member variables with underscore suffix
  TeamInfo ours_;
  TeamInfo theirs_;
  Point field_size_;
  Point penalty_area_size_;
  Point goal_size_;
  Point goal_;
  Ball ball_;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
