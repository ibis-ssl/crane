// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_geometry/geometry_operations.hpp"
#include "crane_msgs/msg/world_model.hpp"

namespace crane
{
struct RobotInfo
{
  uint8_t id;

  Pose2D pose;

  Velocity2D vel;

  bool available = false;

  using SharedPtr = std::shared_ptr<RobotInfo>;
};

struct TeamInfo
{
  Rect defense_area;

  std::vector<std::shared_ptr<RobotInfo>> robots;

  std::vector<std::shared_ptr<RobotInfo>> getAvailableRobots()
  {
    std::vector<std::shared_ptr<RobotInfo>> available_robots;
    for (auto robot : robots) {
      if (robot->available) {
        available_robots.emplace_back(robot);
      }
    }
    return available_robots;
  }

  std::vector<uint8_t> getAvailableRobotIds()
  {
    std::vector<uint8_t> available_robot_ids;
    for (auto robot : robots) {
      if (robot->available) {
        available_robot_ids.emplace_back(robot->id);
      }
    }
    return available_robot_ids;
  }
};

struct Ball
{
  Point pos;

  Point vel;

  bool is_curve;
};

struct RobotIdentifier
{
  bool is_ours;

  uint8_t robot_id;
};
}  // namespace crane

template <>
struct rclcpp::TypeAdapter<Pose2D, geometry_msgs::msg::Pose2D>
{
  using is_specialized = std::true_type;
  using custom_type = Pose2D;
  using ros_message_type = geometry_msgs::msg::Pose2D;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.x = source.pos.x();
    destination.y = source.pos.y();
    destination.theta = source.theta;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.pos.x() = source.x;
    destination.pos.y() = source.y;
    destination.theta = source.theta;
  }
};

template <>
struct rclcpp::TypeAdapter<Point, geometry_msgs::msg::Pose2D>
{
  using is_specialized = std::true_type;
  using custom_type = Point;
  using ros_message_type = geometry_msgs::msg::Pose2D;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.x = source.x();
    destination.y = source.y();
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.x() = source.x;
    destination.y() = source.y;
  }
};

template <>
struct rclcpp::TypeAdapter<Velocity2D, geometry_msgs::msg::Pose2D>
{
  using is_specialized = std::true_type;
  using custom_type = Velocity2D;
  using ros_message_type = geometry_msgs::msg::Pose2D;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.x = source.linear.x();
    destination.y = source.linear.y();
    destination.theta = source.omega;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.linear << source.x, source.y;
    destination.omega = source.theta;
  }
};

template <>
struct rclcpp::TypeAdapter<crane::Ball, crane_msgs::msg::BallInfo>
{
  using is_specialized = std::true_type;
  using custom_type = crane::Ball;
  using ros_message_type = crane_msgs::msg::BallInfo;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.curved = source.is_curve;
    destination.detected = true;
    destination.disappeared = false;
    //    destination.detection_time =
    rclcpp::TypeAdapter<Point, geometry_msgs::msg::Pose2D>::convert_to_ros_message(
      source.pos, destination.pose);
    rclcpp::TypeAdapter<Point, geometry_msgs::msg::Pose2D>::convert_to_ros_message(
      source.vel, destination.velocity);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.is_curve = source.curved;
    rclcpp::TypeAdapter<Point, geometry_msgs::msg::Pose2D>::convert_to_custom(
      source.pose, destination.pos);
    rclcpp::TypeAdapter<Point, geometry_msgs::msg::Pose2D>::convert_to_custom(
      source.velocity, destination.vel);
  }
};

template <>
struct rclcpp::TypeAdapter<crane::RobotInfo, crane_msgs::msg::RobotInfoOurs>
{
  using is_specialized = std::true_type;
  using custom_type = crane::RobotInfo;
  using ros_message_type = crane_msgs::msg::RobotInfoOurs;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.disappeared = !source.available;
    destination.id = source.id;
    rclcpp::TypeAdapter<Pose2D, geometry_msgs::msg::Pose2D>::convert_to_ros_message(
      source.pose, destination.pose);
    rclcpp::TypeAdapter<Velocity2D, geometry_msgs::msg::Pose2D>::convert_to_ros_message(
      source.vel, destination.velocity);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.available = !source.disappeared;
    destination.id = source.id;
    rclcpp::TypeAdapter<Pose2D, geometry_msgs::msg::Pose2D>::convert_to_custom(
      source.pose, destination.pose);
    rclcpp::TypeAdapter<Velocity2D, geometry_msgs::msg::Pose2D>::convert_to_custom(
      source.velocity, destination.vel);
  }
};

template <>
struct rclcpp::TypeAdapter<crane::RobotInfo, crane_msgs::msg::RobotInfoTheirs>
{
  using is_specialized = std::true_type;
  using custom_type = crane::RobotInfo;
  using ros_message_type = crane_msgs::msg::RobotInfoTheirs;
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.disappeared = !source.available;
    destination.id = source.id;
    rclcpp::TypeAdapter<Pose2D, geometry_msgs::msg::Pose2D>::convert_to_ros_message(
      source.pose, destination.pose);
    rclcpp::TypeAdapter<Velocity2D, geometry_msgs::msg::Pose2D>::convert_to_ros_message(
      source.vel, destination.velocity);
  }
  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.available = !source.disappeared;
    destination.id = source.id;
    rclcpp::TypeAdapter<Pose2D, geometry_msgs::msg::Pose2D>::convert_to_custom(
      source.pose, destination.pose);
    rclcpp::TypeAdapter<Velocity2D, geometry_msgs::msg::Pose2D>::convert_to_custom(
      source.velocity, destination.vel);
  }
};

namespace crane
{
struct WorldModelWrapper
{
  typedef std::shared_ptr<WorldModelWrapper> SharedPtr;

  typedef std::unique_ptr<WorldModelWrapper> UniquePtr;

  WorldModelWrapper(rclcpp::Node & node)
  {
    // メモリ確保
    // ヒトサッカーの台数は超えないはず
    constexpr uint8_t MAX_ROBOT_NUM = 20;
    for (int i = 0; i < MAX_ROBOT_NUM; i++) {
      ours.robots.emplace_back(std::make_shared<RobotInfo>());
      theirs.robots.emplace_back(std::make_shared<RobotInfo>());
    }

    subscriber = node.create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10, [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void {
        latest_msg = *msg;
        this->update(*msg);
        has_updated = true;
        for (auto & callback : callbacks) {
          callback();
        }
      });
  }

  void update(const crane_msgs::msg::WorldModel & world_model)
  {
    for (auto & robot : world_model.robot_info_ours) {
      auto & info = ours.robots.at(robot.id);
      info->available = !robot.disappeared;
      if (info->available) {
        info->id = robot.id;
        info->pose.pos << robot.pose.x, robot.pose.y;
        info->pose.theta = robot.pose.theta;
        info->vel.linear << robot.velocity.x, robot.velocity.y;
        // todo : omega
      }
    }

    for (auto robot : world_model.robot_info_theirs) {
      auto & info = theirs.robots.at(robot.id);
      info->available = !robot.disappeared;
      if (info->available) {
        info->id = robot.id;
        info->pose.pos << robot.pose.x, robot.pose.y;
        info->pose.theta = robot.pose.theta;
        info->vel.linear << robot.velocity.x, robot.velocity.y;
        // todo : omega
      }
    }

    ball.pos << world_model.ball_info.pose.x, world_model.ball_info.pose.y;
    ball.vel << world_model.ball_info.velocity.x, world_model.ball_info.velocity.y;
    //    ball.is_curve = world_model.ball_info.curved;

    field_size << world_model.field_info.x, world_model.field_info.y;
    defense_area_size << world_model.defense_area_size.x, world_model.defense_area_size.y;

    goal_size << world_model.goal_size.x, world_model.goal_size.y;
    goal << (isYellow() ? field_size.x() * 0.5 : -field_size.x() * 0.5), 0.;

    if (goal.x() > 0) {
      ours.defense_area.max << goal.x(), goal.y() + world_model.defense_area_size.y / 2.;
      ours.defense_area.min << goal.x() - world_model.defense_area_size.x,
        goal.y() - world_model.defense_area_size.y / 2.;
    } else {
      ours.defense_area.max << goal.x() + world_model.defense_area_size.x,
        goal.y() + world_model.defense_area_size.y / 2.;
      ours.defense_area.min << goal.x(), goal.y() - world_model.defense_area_size.y / 2.;
    }
    theirs.defense_area.max << std::max(-ours.defense_area.max.x(), -ours.defense_area.min.x()),
      ours.defense_area.max.y();
    theirs.defense_area.min << std::min(-ours.defense_area.max.x(), -ours.defense_area.min.x()),
      ours.defense_area.min.y();
  }

  const crane_msgs::msg::WorldModel & getMsg() const { return latest_msg; }

  bool isYellow() const { return (latest_msg.is_yellow); }

  bool hasUpdated() const { return has_updated; }

  void addCallback(std::function<void(void)> && callback_func)
  {
    callbacks.emplace_back(callback_func);
  }

  auto getRobot(RobotIdentifier id)
  {
    if (id.is_ours) {
      return ours.robots.at(id.robot_id);
    } else {
      return theirs.robots.at(id.robot_id);
    }
  }

  auto getDistanceFromRobotToBall(RobotIdentifier id) -> double
  {
    return getDistanceFromRobot(id, ball.pos);
  }

  auto getSquareDistanceFromRobotToBall(RobotIdentifier id) -> double
  {
    return getSquareDistanceFromRobot(id, ball.pos);
  }

  auto generateFieldPoints(float grid_size) const
  {
    std::vector<Point> points;
    for (float x = 0.f; x <= field_size.x() / 2.f; x += grid_size) {
      for (float y = 0.f; y <= field_size.y() / 2.f; y += grid_size) {
        points.emplace_back(Point(x, y));
      }
    }
    return points;
  }

  auto getDistanceFromRobot(RobotIdentifier id, Point point) -> double
  {
    return (getRobot(id)->pose.pos - point).norm();
  }

  auto getSquareDistanceFromRobot(RobotIdentifier id, Point point) -> double
  {
    return (getRobot(id)->pose.pos - point).squaredNorm();
  }

  auto getNearestRobotsWithDistanceFromPoint(Point point, std::vector<std::shared_ptr<RobotInfo>> & robots) -> std::pair<std::shared_ptr<RobotInfo>, double>
  {
    std::shared_ptr<RobotInfo> nearest_robot = nullptr;
    double min_sq_distance = std::numeric_limits<double>::max();
    for (const auto & robot : robots) {
      if (!robot->available) {
        continue;
      }
      double sq_distance = (robot->pose.pos - point).squaredNorm();
      if (sq_distance < min_sq_distance) {
        min_sq_distance = sq_distance;
        nearest_robot = robot;
      }
    }
    return {nearest_robot, std::sqrt(min_sq_distance)};
  }

  bool isEnemyDefenseArea(const Point & p) const { return isInRect(ours.defense_area, p); }

  bool isFriendDefenseArea(const Point & p) const { return isInRect(theirs.defense_area, p); }

  bool isDefenseArea(Point p) const { return isFriendDefenseArea(p) || isEnemyDefenseArea(p); }

  bool isFieldInside(Point p) const
  {
    Rect field_rect;
    field_rect.min << -field_size.x() / 2.f, -field_size.y() / 2.f;
    field_rect.max << field_size.x() / 2.f, field_size.y() / 2.f;
    return isInRect(field_rect, p);
  }

  double getDefenseWidth() const { return ours.defense_area.max.y() - ours.defense_area.min.y(); }

  double getDefenseHeight() const { return ours.defense_area.max.x() - ours.defense_area.min.x(); }

  std::pair<Point, Point> getOurGoalPosts()
  {
    double x = getOurGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  std::pair<Point, Point> getTheirGoalPosts()
  {
    double x = getTheirGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  Rect getOurDefenseArea() { return ours.defense_area; }

  Point getOurGoalCenter() { return goal; }

  Point getTheirGoalCenter() { return Point(-goal.x(), goal.y()); }

  TeamInfo ours;

  TeamInfo theirs;

  Point field_size, defense_area_size, goal_size;

  Point goal;

  Ball ball;

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber;

  std::vector<std::function<void(void)>> callbacks;

  crane_msgs::msg::WorldModel latest_msg;

  bool has_updated = false;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
