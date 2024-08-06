// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <crane_basics/ball_info.hpp>
#include <crane_basics/ball_model.hpp>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/interval.hpp>
#include <crane_basics/robot_info.hpp>
#include <crane_basics/travel_time.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "play_situation_wrapper.hpp"

namespace crane
{
struct TeamInfo
{
  Box penalty_area;

  std::vector<std::shared_ptr<RobotInfo>> robots;

  [[nodiscard]] auto getAvailableRobots(uint8_t my_id = 255)
    -> std::vector<std::shared_ptr<RobotInfo>>;

  [[nodiscard]] auto getAvailableRobotIds(uint8_t my_id = 255) const -> std::vector<uint8_t>
  {
    std::vector<uint8_t> available_robot_ids;
    for (const auto & robot : robots) {
      if (robot->available && robot->id != my_id) {
        available_robot_ids.emplace_back(robot->id);
      }
    }
    return available_robot_ids;
  }
};

struct WorldModelWrapper
{
  typedef std::shared_ptr<WorldModelWrapper> SharedPtr;

  typedef std::unique_ptr<WorldModelWrapper> UniquePtr;

  explicit WorldModelWrapper(rclcpp::Node & node);

  void update(const crane_msgs::msg::WorldModel & world_model);

  [[nodiscard]] const auto & getMsg() const { return latest_msg; }

  [[nodiscard]] auto onPositiveHalf() const { return (latest_msg.on_positive_half); }

  [[nodiscard]] auto getOurSideSign() const { return onPositiveHalf() ? 1.0 : -1.0; }

  [[nodiscard]] auto isYellow() const { return (latest_msg.is_yellow); }

  [[nodiscard]] auto hasUpdated() const { return has_updated; }

  [[nodiscard]] auto isOurBall() const { return latest_msg.ball_info.is_our_ball; }

  [[nodiscard]] auto isTheirBall() const { return latest_msg.ball_info.is_their_ball; }

  [[nodiscard]] auto isBallPossessionStateChanged() const
  {
    return latest_msg.ball_info.state_changed;
  }

  void addCallback(std::function<void(void)> && callback_func)
  {
    callbacks.emplace_back(callback_func);
  }

  [[nodiscard]] auto getRobot(RobotIdentifier id) const
  {
    if (id.is_ours) {
      return ours.robots.at(id.robot_id);
    } else {
      return theirs.robots.at(id.robot_id);
    }
  }

  [[nodiscard]] auto getOurRobot(uint8_t id) const { return ours.robots.at(id); }

  [[nodiscard]] auto getTheirRobot(uint8_t id) const { return theirs.robots.at(id); }

  [[nodiscard]] auto getDistanceFromRobotToBall(RobotIdentifier id) const -> double
  {
    return getDistanceFromRobot(id, ball.pos);
  }

  [[nodiscard]] auto getDistanceFromRobotToBall(uint8_t our_id) const -> double
  {
    return getDistanceFromRobot({true, our_id}, ball.pos);
  }

  [[nodiscard]] auto getSquareDistanceFromRobotToBall(RobotIdentifier id) const -> double
  {
    return getSquareDistanceFromRobot(id, ball.pos);
  }

  [[nodiscard]] auto getSquareDistanceFromRobotToBall(uint8_t our_id) const -> double
  {
    return getSquareDistanceFromRobot({true, our_id}, ball.pos);
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
    return (ball.pos - point).norm();
  }

  [[nodiscard]] auto getSquareDistanceFromBall(const Point & point) const -> double
  {
    return (ball.pos - point).squaredNorm();
  }

  [[nodiscard]] auto getNearestRobotWithDistanceFromPoint(
    const Point & point, std::vector<std::shared_ptr<RobotInfo>> robots) const
    -> std::pair<std::shared_ptr<RobotInfo>, double>;

  [[nodiscard]] auto getNearestRobotWithDistanceFromSegment(
    const Segment & segment, std::vector<std::shared_ptr<RobotInfo>> robots) const
    -> std::pair<std::shared_ptr<RobotInfo>, double>;

  [[nodiscard]] auto getFieldMargin() const { return 0.3; }

  [[nodiscard]] auto getDefenseWidth() const
  {
    return ours.penalty_area.max_corner().y() - ours.penalty_area.min_corner().y();
  }

  [[nodiscard]] auto getDefenseHeight() const
  {
    return ours.penalty_area.max_corner().x() - ours.penalty_area.min_corner().x();
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

  [[nodiscard]] auto getOurPenaltyArea() const { return ours.penalty_area; }

  [[nodiscard]] auto getTheirPenaltyArea() const { return theirs.penalty_area; }

  [[nodiscard]] auto getOurGoalCenter() const -> Point { return goal; }

  [[nodiscard]] auto getTheirGoalCenter() const -> Point { return Point(-goal.x(), goal.y()); }

  [[nodiscard]] auto getBallPlacementTarget() const -> std::optional<Point>;

  // rule 8.4.3
  [[nodiscard]] auto getBallPlacementArea(double offset = 0.) const -> std::optional<Capsule>;

  [[nodiscard]] auto getOurGoalieId() const { return latest_msg.our_goalie_id; }

  [[nodiscard]] auto getTheirGoalieId() const { return latest_msg.their_goalie_id; }

  /**
   *
   * @param from
   * @return {angle, width}
   */
  [[nodiscard]] auto getLargestGoalAngleRangeFromPoint(Point from) -> std::pair<double, double>;

  [[nodiscard]] auto getLargestOurGoalAngleRangeFromPoint(
    Point from, std::vector<std::shared_ptr<RobotInfo>> robots) -> std::pair<double, double>;

  [[nodiscard]] auto getLargestOurGoalAngleRangeFromPoint(Point from) -> std::pair<double, double>
  {
    return getLargestOurGoalAngleRangeFromPoint(from, ours.getAvailableRobots());
  }

  struct SlackTimeResult
  {
    double slack_time;
    Point intercept_point;
    std::shared_ptr<RobotInfo> robot;
  };

  [[nodiscard]] auto getBallSlackTime(
    double time, const std::vector<std::shared_ptr<RobotInfo>> & robots)
    -> std::optional<SlackTimeResult>;

  [[nodiscard]] auto getMinMaxSlackInterceptPoint(
    std::vector<std::shared_ptr<RobotInfo>> robots, double t_horizon = 5.0, double t_step = 0.1,
    double slack_time_offset = 0.0) -> std::pair<std::optional<Point>, std::optional<Point>>;

  [[nodiscard]] auto getMinMaxSlackInterceptPointAndSlackTime(
    std::vector<std::shared_ptr<RobotInfo>> robots, double t_horizon = 5.0, double t_step = 0.1,
    double slack_time_offset = 0.0)
    -> std::pair<std::optional<std::pair<Point, double>>, std::optional<std::pair<Point, double>>>;

  TeamInfo ours;

  TeamInfo theirs;

  Point field_size, penalty_area_size, goal_size;

  Point goal;

  std::optional<Point> ball_placement_target = std::nullopt;

  Ball ball;

  PlaySituationWrapper play_situation;

private:
  class BallOwnerCalculator
  {
  public:
    explicit BallOwnerCalculator(WorldModelWrapper * world_model) : world_model(world_model) {}

    struct RobotWithScore
    {
      std::shared_ptr<RobotInfo> robot = nullptr;
      double min_slack = 100.;
      double max_slack = -100.;
      double score;
    };

    void update();

    void updateScore(bool our_team);

    [[nodiscard]] RobotWithScore calculateScore(const std::shared_ptr<RobotInfo> & robot) const;

    [[nodiscard]] std::optional<RobotWithScore> getOurFrontier() const
    {
      if (sorted_our_robots.empty()) {
        return std::nullopt;
      } else {
        return sorted_our_robots.front();
      }
    }

    [[nodiscard]] std::optional<RobotWithScore> getTheirFrontier() const
    {
      if (sorted_their_robots.empty()) {
        return std::nullopt;
      } else {
        return sorted_their_robots.front();
      }
    }

    [[nodiscard]] bool isOurBall() const { return is_our_ball; }

    void setBallOwnerTeamChangeCallback(const std::function<void(bool)> & callback)
    {
      ball_owner_team_change_callback = callback;
    }

    void setBallOwnerIDChangeCallback(const std::function<void(std::uint8_t)> & callback)
    {
      ball_owner_id_change_callback = callback;
    }

    bool getIsOurBallOwnerChanged() const { return is_our_ball_owner_changed; }

    bool getIsTheirBallOwnerChanged() const { return is_their_ball_owner_changed; }

    bool getIsBallOwnerTeamChanged() const { return is_ball_owner_team_changed; }

  private:
    std::vector<RobotWithScore> sorted_our_robots;

    std::vector<RobotWithScore> sorted_their_robots;

    WorldModelWrapper * world_model;

    bool is_our_ball = false;

    bool is_our_ball_owner_changed = false;

    bool is_their_ball_owner_changed = false;

    bool is_ball_owner_team_changed = false;

    std::uint8_t our_frontier = 255;

    std::function<void(bool)> ball_owner_team_change_callback = nullptr;

    std::function<void(std::uint8_t)> ball_owner_id_change_callback = nullptr;
  } ball_owner_calculator;

  bool ball_owner_calculator_enabled = false;

public:
  void setBallOwnerCalculatorEnabled(bool enabled = true)
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

  [[nodiscard]] auto isOurBallByBallOwnerCalculator() const
  {
    return ball_owner_calculator.isOurBall();
  }

  [[nodiscard]] auto isOurBallOwnerChanged() const
  {
    return ball_owner_calculator.getIsOurBallOwnerChanged();
  }

  [[nodiscard]] auto isTheirBallOwnerChanged() const
  {
    return ball_owner_calculator.getIsTheirBallOwnerChanged();
  }

  [[nodiscard]] auto isBallOwnerTeamChanged() const
  {
    return ball_owner_calculator.getIsBallOwnerTeamChanged();
  }

  class PointChecker
  {
  public:
    explicit PointChecker(WorldModelWrapper::SharedPtr & world_model)
    : world_model(world_model.get())
    {
    }

    explicit PointChecker(WorldModelWrapper * world_model) : world_model(world_model) {}

    [[nodiscard]] bool isFieldInside(const Point & p, double offset = 0.) const;

    void addFieldInsideChecker(double offset = 0.)
    {
      checkers.emplace_back([this, offset](const Point & p) { return isFieldInside(p, offset); });
    }

    void addFieldOutsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isFieldInside(p, offset); });
    }

    [[nodiscard]] bool isBallPlacementArea(const Point & p, double offset = 0.) const;

    void addBallPlacementAreaInsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return isBallPlacementArea(p, offset); });
    }

    void addBallPlacementAreaOutsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isBallPlacementArea(p, offset); });
    }

    [[nodiscard]] bool isEnemyPenaltyArea(const Point & p, double offset = 0.) const;

    void addEnemyPenaltyAreaInsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return isEnemyPenaltyArea(p, offset); });
    }

    void addEnemyPenaltyAreaOutsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isEnemyPenaltyArea(p, offset); });
    }

    [[nodiscard]] bool isFriendPenaltyArea(const Point & p, double offset = 0.) const;

    void addFriendPenaltyAreaInsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return isFriendPenaltyArea(p, offset); });
    }

    void addFriendPenaltyAreaOutsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isFriendPenaltyArea(p, offset); });
    }

    [[nodiscard]] bool isPenaltyArea(const Point & p, double offset = 0.) const;

    void addPenaltyAreaInsideChecker(double offset = 0.)
    {
      checkers.emplace_back([this, offset](const Point & p) { return isPenaltyArea(p, offset); });
    }

    void addPenaltyAreaOutsideChecker(double offset = 0.)
    {
      checkers.emplace_back(
        [this, offset](const Point & p) { return not isPenaltyArea(p, offset); });
    }

    [[nodiscard]] bool isInOurHalf(const Point & p, double offset = 0.) const
    {
      return p.x() * world_model->getOurSideSign() > offset;
    }

    void addInOurHalfChecker(double offset = 0.)
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

    [[nodiscard]] bool checkDistance(
      const Point & p, const Point & target, double threshold, const Rule rule) const
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

    void addDistanceChecker(const Point & target, double threshold, const Rule rule)
    {
      checkers.emplace_back([this, target, threshold, rule](const Point & p) {
        return checkDistance(p, target, threshold, rule);
      });
    }

    [[nodiscard]] bool checkDistanceFromBall(
      const Point & p, double threshold, const Rule rule) const
    {
      return checkDistance(p, world_model->ball.pos, threshold, rule);
    }

    void addDistanceFromBallChecker(double threshold, const Rule rule)
    {
      checkers.emplace_back([this, threshold, rule](const Point & p) {
        return checkDistanceFromBall(p, threshold, rule);
      });
    }

    [[nodiscard]] bool checkDistanceFromRobot(
      const Point & p, RobotIdentifier id, double threshold, const Rule rule) const
    {
      return checkDistance(p, world_model->getRobot(id)->pose.pos, threshold, rule);
    }

    void addDistanceFromRobotChecker(RobotIdentifier id, double threshold, const Rule rule)
    {
      checkers.emplace_back([this, id, threshold, rule](const Point & p) {
        return checkDistanceFromRobot(p, id, threshold, rule);
      });
    }

    [[nodiscard]] bool checkDistanceFromRobot(
      const Point & p, std::shared_ptr<RobotInfo> robot, double threshold, const Rule rule) const
    {
      return checkDistance(p, robot->pose.pos, threshold, rule);
    }

    [[nodiscard]] bool checkDistanceFromRobots(
      const Point & p, std::vector<std::shared_ptr<RobotInfo>> robots, double threshold,
      const Rule rule) const
    {
      for (auto robot : robots) {
        if (not checkDistance(p, robot->pose.pos, threshold, rule)) {
          return false;
        }
      }
      return true;
    }

    void addDistanceFromRobotsChecker(
      std::vector<std::shared_ptr<RobotInfo>> robots, double threshold, const Rule rule)
    {
      checkers.emplace_back([this, robots, threshold, rule](const Point & p) {
        return checkDistanceFromRobots(p, robots, threshold, rule);
      });
    }

    [[nodiscard]] bool checkDistanceFromOurRobots(
      const Point & p, double threshold, const Rule rule) const
    {
      return checkDistanceFromRobots(p, world_model->ours.getAvailableRobots(), threshold, rule);
    }

    void addDistanceFromOurRobotsChecker(double threshold, const Rule rule)
    {
      checkers.emplace_back([this, threshold, rule](const Point & p) {
        return checkDistanceFromOurRobots(p, threshold, rule);
      });
    }

    [[nodiscard]] bool checkDistanceFromTheirRobots(
      const Point & p, double threshold, const Rule rule) const
    {
      return checkDistanceFromRobots(p, world_model->theirs.getAvailableRobots(), threshold, rule);
    }

    void addDistanceFromTheirRobotsChecker(double threshold, const Rule rule)
    {
      checkers.emplace_back([this, threshold, rule](const Point & p) {
        return checkDistanceFromTheirRobots(p, threshold, rule);
      });
    }

    void addCustomChecker(std::function<bool(const Point &)> checker)
    {
      checkers.emplace_back(checker);
    }

    bool operator()(const Point & p) const
    {
      return std::all_of(checkers.begin(), checkers.end(), [p](auto & check) { return check(p); });
    }

    static PointChecker buildStandard(WorldModelWrapper::SharedPtr world_model)
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

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber;

  std::vector<std::function<void(void)>> callbacks;

  crane_msgs::msg::WorldModel latest_msg;

  bool has_updated = false;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
