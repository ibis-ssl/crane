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
  Box defense_area;

  std::vector<std::shared_ptr<RobotInfo>> robots;

  std::vector<std::shared_ptr<RobotInfo>> getAvailableRobots(uint8_t my_id = 255);

  std::vector<uint8_t> getAvailableRobotIds(uint8_t my_id = 255)
  {
    std::vector<uint8_t> available_robot_ids;
    for (auto robot : robots) {
      if (robot->available && robot->id != my_id) {
        available_robot_ids.emplace_back(robot->id);
      }
    }
    return available_robot_ids;
  }
};

struct WorldModelWrapper : public std::enable_shared_from_this<WorldModelWrapper>
{
  typedef std::shared_ptr<WorldModelWrapper> SharedPtr;

  typedef std::unique_ptr<WorldModelWrapper> UniquePtr;

  explicit WorldModelWrapper(rclcpp::Node & node);

  void update(const crane_msgs::msg::WorldModel & world_model);

  [[nodiscard]] const crane_msgs::msg::WorldModel & getMsg() const { return latest_msg; }

  [[nodiscard]] bool onPositiveHalf() const { return (latest_msg.on_positive_half); }

  [[nodiscard]] double getOurSideSign() const { return onPositiveHalf() ? 1.0 : -1.0; }

  [[nodiscard]] bool isYellow() const { return (latest_msg.is_yellow); }

  [[nodiscard]] bool hasUpdated() const { return has_updated; }

  [[nodiscard]] bool isOurBall() const { return latest_msg.ball_info.is_our_ball; }

  [[nodiscard]] bool isTheirBall() const { return latest_msg.ball_info.is_their_ball; }

  [[nodiscard]] bool isBallPossessionStateChanged() const
  {
    return latest_msg.ball_info.state_changed;
  }

  void addCallback(std::function<void(void)> && callback_func)
  {
    callbacks.emplace_back(callback_func);
  }

  auto getRobot(RobotIdentifier id) const
  {
    if (id.is_ours) {
      return ours.robots.at(id.robot_id);
    } else {
      return theirs.robots.at(id.robot_id);
    }
  }

  auto getOurRobot(uint8_t id) const { return ours.robots.at(id); }

  auto getTheirRobot(uint8_t id) const { return theirs.robots.at(id); }

  auto getDistanceFromRobotToBall(RobotIdentifier id) const -> double
  {
    return getDistanceFromRobot(id, ball.pos);
  }

  auto getDistanceFromRobotToBall(uint8_t our_id) const -> double
  {
    return getDistanceFromRobot({true, our_id}, ball.pos);
  }

  auto getSquareDistanceFromRobotToBall(RobotIdentifier id) const -> double
  {
    return getSquareDistanceFromRobot(id, ball.pos);
  }

  auto getSquareDistanceFromRobotToBall(uint8_t our_id) const -> double
  {
    return getSquareDistanceFromRobot({true, our_id}, ball.pos);
  }

  [[nodiscard]] auto generateFieldPoints(float grid_size) const;

  auto getDistanceFromRobot(RobotIdentifier id, const Point & point) const -> double
  {
    return (getRobot(id)->pose.pos - point).norm();
  }

  auto getDistanceFromRobot(uint8_t our_id, const Point & point) const -> double
  {
    return (getOurRobot(our_id)->pose.pos - point).norm();
  }

  auto getSquareDistanceFromRobot(RobotIdentifier id, const Point & point) const -> double
  {
    return (getRobot(id)->pose.pos - point).squaredNorm();
  }

  auto getSquareDistanceFromRobot(uint8_t our_id, const Point & point) const -> double
  {
    return (getOurRobot(our_id)->pose.pos - point).squaredNorm();
  }

  auto getDistanceFromBall(const Point & point) const -> double
  {
    return (ball.pos - point).norm();
  }

  auto getSquareDistanceFromBall(const Point & point) const -> double
  {
    return (ball.pos - point).squaredNorm();
  }

  auto getNearestRobotsWithDistanceFromPoint(
    const Point & point, const std::vector<std::shared_ptr<RobotInfo>> robots) const
    -> std::pair<std::shared_ptr<RobotInfo>, double>;

  [[nodiscard]] double getFieldMargin() const { return 0.3; }

  [[nodiscard]] double getDefenseWidth() const
  {
    return ours.defense_area.max_corner().y() - ours.defense_area.min_corner().y();
  }

  [[nodiscard]] double getDefenseHeight() const
  {
    return ours.defense_area.max_corner().x() - ours.defense_area.min_corner().x();
  }

  [[nodiscard]] std::pair<Point, Point> getOurGoalPosts() const
  {
    double x = getOurGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  [[nodiscard]] std::pair<Point, Point> getTheirGoalPosts() const
  {
    double x = getTheirGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  [[nodiscard]] Box getOurDefenseArea() const { return ours.defense_area; }

  [[nodiscard]] Box getTheirDefenseArea() const { return theirs.defense_area; }

  [[nodiscard]] Point getOurGoalCenter() const { return goal; }

  [[nodiscard]] Point getTheirGoalCenter() const { return Point(-goal.x(), goal.y()); }

  [[nodiscard]] std::optional<Point> getBallPlacementTarget() const;

  // rule 8.4.3
  [[nodiscard]] std::optional<Capsule> getBallPlacementArea(const double offset = 0.) const;

  [[nodiscard]] uint8_t getOurGoalieId() const { return latest_msg.our_goalie_id; }

  [[nodiscard]] uint8_t getTheirGoalieId() const { return latest_msg.their_goalie_id; }

  /**
   *
   * @param from
   * @return {angle, width}
   */
  auto getLargestGoalAngleRangeFromPoint(Point from) -> std::pair<double, double>;

  auto getLargestOurGoalAngleRangeFromPoint(
    Point from, std::vector<std::shared_ptr<RobotInfo>> robots) -> std::pair<double, double>;

  auto getLargestOurGoalAngleRangeFromPoint(Point from) -> std::pair<double, double>
  {
    return getLargestOurGoalAngleRangeFromPoint(from, ours.getAvailableRobots());
  }

  struct SlackTimeResult
  {
    double slack_time;
    Point intercept_point;
    std::shared_ptr<RobotInfo> robot;
  };

  auto getBallSlackTime(double time, std::vector<std::shared_ptr<RobotInfo>> robots)
    -> std::optional<SlackTimeResult>
  {
    // https://www.youtube.com/live/bizGFvaVUIk?si=mFZqirdbKDZDttIA&t=1452
    auto p_ball = getFutureBallPosition(ball.pos, ball.vel, time);
    if (p_ball) {
      Point intercept_point = p_ball.value() + ball.vel.normalized() * 0.3;
      double min_robot_time = std::numeric_limits<double>::max();
      std::shared_ptr<RobotInfo> best_robot = nullptr;
      for (auto robot : robots) {
        double t_robot = getTravelTimeTrapezoidal(robot, intercept_point);
        if (t_robot < min_robot_time) {
          min_robot_time = t_robot;
          best_robot = robot;
        }
      }
      if (min_robot_time != std::numeric_limits<double>::max()) {
        return std::make_optional<SlackTimeResult>(
          {time - min_robot_time, intercept_point, best_robot});
      } else {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  }

  TeamInfo ours;

  TeamInfo theirs;

  Point field_size, defense_area_size, goal_size;

  Point goal;

  std::optional<Point> ball_placement_target = std::nullopt;

  Ball ball;

  PlaySituationWrapper play_situation;

  class PointChecker
  {
  public:
    explicit PointChecker(const WorldModelWrapper::SharedPtr & world_model)
    : world_model(world_model)
    {
    }

    [[nodiscard]] bool isFieldInside(const Point & p, double offset = 0.) const;

    [[nodiscard]] bool isBallPlacementArea(const Point & p, double offset = 0.) const;

    [[nodiscard]] bool isEnemyDefenseArea(const Point & p) const;

    [[nodiscard]] bool isFriendDefenseArea(const Point & p) const;

    [[nodiscard]] bool isDefenseArea(const Point & p) const;

  private:
    WorldModelWrapper::SharedPtr world_model;
  } point_checker;

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber;

  std::vector<std::function<void(void)>> callbacks;

  crane_msgs::msg::WorldModel latest_msg;

  bool has_updated = false;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
