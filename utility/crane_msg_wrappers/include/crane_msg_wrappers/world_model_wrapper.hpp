// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_

#include <Eigen/Core>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "play_situation_wrapper.hpp"

namespace crane
{
struct BallContact
{
  std::chrono::system_clock::time_point last_contact_end_time;
  std::chrono::system_clock::time_point last_contact_start_time;

  void update(bool is_contacted)
  {
    auto now = std::chrono::system_clock::now();
    if (is_contacted) {
      last_contact_end_time = now;
      if (not is_contacted_pre_frame) {
        last_contact_start_time = now;
      }
    } else {
      last_contact_start_time = last_contact_end_time;
    }
    is_contacted_pre_frame = is_contacted;
  }

  auto getContactDuration() { return (last_contact_end_time - last_contact_start_time); }

  auto findPastContact(double duration_sec)
  {
    auto past = std::chrono::system_clock::now() - std::chrono::duration<double>(duration_sec);
    return past < last_contact_end_time;
  }

private:
  bool is_contacted_pre_frame = false;
};

struct RobotIdentifier
{
  bool is_ours;

  uint8_t robot_id;
};

struct RobotInfo
{
  uint8_t id;

  RobotIdentifier getID() const { return {true, id}; }

  Pose2D pose;

  Velocity2D vel;

  bool available = false;

  using SharedPtr = std::shared_ptr<RobotInfo>;

  Vector2 center_to_kicker() const { return getNormVec(pose.theta) * 0.060; }

  Point kicker_center() const { return pose.pos + center_to_kicker(); }

  BallContact ball_contact;

  auto geometry() { return Circle{pose.pos, 0.060}; }
};

struct TeamInfo
{
  Box defense_area;

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

struct Hysteresis
{
  Hysteresis(double lower, double upper) : lower_threshold(lower), upper_threshold(upper){};

  double lower_threshold, upper_threshold;

  bool is_high = false;

  std::function<void(void)> upper_callback = []() {};
  std::function<void(void)> lower_callback = []() {};

  void update(double value)
  {
    if (not is_high && value > upper_threshold) {
      is_high = true;
      upper_callback();
    }

    if (is_high && value < lower_threshold) {
      is_high = false;
      lower_callback();
    }
  }
};

struct Ball
{
  Point pos;

  Point vel;

  bool is_curve;

  bool isMoving(double threshold_velocity = 0.01) const { return vel.norm() > threshold_velocity; }

  bool isStopped(double threshold_velocity = 0.01) const
  {
    return not isMoving(threshold_velocity);
  }

  bool isMovingTowards(
    const Point & p, double angle_threshold_deg = 60.0, double near_threshold = 0.2) const
  {
    if ((pos - p).norm() < near_threshold) {
      return false;
    } else {
      auto dir = (p - pos).normalized();
      return dir.dot(vel.normalized()) > cos(angle_threshold_deg * M_PI / 180.0);
    }
  }

private:
  Hysteresis ball_speed_hysteresis = Hysteresis(0.1, 0.6);
  friend class WorldModelWrapper;
};

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
    play_situation.update(world_model.play_situation);

    for (auto & our_robot : ours.robots) {
      our_robot->available = false;
    }

    for (auto & their_robot : theirs.robots) {
      their_robot->available = false;
    }

    ball.pos << world_model.ball_info.pose.x, world_model.ball_info.pose.y;
    ball.vel << world_model.ball_info.velocity.x, world_model.ball_info.velocity.y;
    ball.ball_speed_hysteresis.update(ball.vel.norm());

    for (auto & robot : world_model.robot_info_ours) {
      auto & info = ours.robots.at(robot.id);
      info->available = !robot.disappeared;
      if (info->available) {
        info->id = robot.id;
        info->pose.pos << robot.pose.x, robot.pose.y;
        info->pose.theta = robot.pose.theta;
        info->vel.linear << robot.velocity.x, robot.velocity.y;
        info->ball_contact.update((info->kicker_center() - ball.pos).norm() < 0.1);
      } else {
        info->ball_contact.update(false);
      }
    }

    for (auto robot : world_model.robot_info_theirs) {
      auto & info = theirs.robots.at(robot.id);
      info->available = !robot.disappeared;
      if (info->available) {
        info->id = robot.id;
        info->ball_contact.update(
          robot.ball_contact.current_time == robot.ball_contact.last_contacted_time);
        info->pose.pos << robot.pose.x, robot.pose.y;
        info->pose.theta = robot.pose.theta;
        info->vel.linear << robot.velocity.x, robot.velocity.y;
        // todo : omega
      } else {
        info->ball_contact.update(false);
      }
    }

    field_size << world_model.field_info.x, world_model.field_info.y;
    defense_area_size << world_model.defense_area_size.x, world_model.defense_area_size.y;

    goal_size << world_model.goal_size.x, world_model.goal_size.y;
    goal << (isYellow() ? field_size.x() * 0.5 : -field_size.x() * 0.5), 0.;

    if (goal.x() > 0) {
      ours.defense_area.max_corner() << goal.x(), goal.y() + world_model.defense_area_size.y / 2.;
      ours.defense_area.min_corner() << goal.x() - world_model.defense_area_size.x,
        goal.y() - world_model.defense_area_size.y / 2.;
    } else {
      ours.defense_area.max_corner() << goal.x() + world_model.defense_area_size.x,
        goal.y() + world_model.defense_area_size.y / 2.;
      ours.defense_area.min_corner() << goal.x(), goal.y() - world_model.defense_area_size.y / 2.;
    }
    theirs.defense_area.max_corner() << std::max(-ours.defense_area.max_corner().x(), -ours.defense_area.min_corner().x()),
      ours.defense_area.max_corner().y();
    theirs.defense_area.min_corner() << std::min(-ours.defense_area.max_corner().x(), -ours.defense_area.min_corner().x()),
      ours.defense_area.min_corner().y();

    ball_placement_target << world_model.ball_placement_target.x,
      world_model.ball_placement_target.y;
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

  auto getOurRobot(uint8_t id) { return ours.robots.at(id); }

  auto getTheirRobot(uint8_t id) { return theirs.robots.at(id); }

  auto getDistanceFromRobotToBall(RobotIdentifier id) -> double
  {
    return getDistanceFromRobot(id, ball.pos);
  }

  auto getDistanceFromRobotToBall(uint8_t our_id) -> double
  {
    return getDistanceFromRobot({true, our_id}, ball.pos);
  }

  auto getSquareDistanceFromRobotToBall(RobotIdentifier id) -> double
  {
    return getSquareDistanceFromRobot(id, ball.pos);
  }

  auto getSquareDistanceFromRobotToBall(uint8_t our_id) -> double
  {
    return getSquareDistanceFromRobot({true, our_id}, ball.pos);
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

  auto getDistanceFromRobot(uint8_t our_id, Point point) -> double
  {
    return (getOurRobot(our_id)->pose.pos - point).norm();
  }

  auto getSquareDistanceFromRobot(RobotIdentifier id, Point point) -> double
  {
    return (getRobot(id)->pose.pos - point).squaredNorm();
  }

  auto getSquareDistanceFromRobot(uint8_t our_id, Point point) -> double
  {
    return (getOurRobot(our_id)->pose.pos - point).squaredNorm();
  }

  auto getDistanceFromBall(Point point) -> double { return (ball.pos - point).norm(); }

  auto getSquareDistanceFromBall(Point point) -> double { return (ball.pos - point).squaredNorm(); }

  auto getNearestRobotsWithDistanceFromPoint(
    Point point, std::vector<std::shared_ptr<RobotInfo>> & robots)
    -> std::pair<std::shared_ptr<RobotInfo>, double>
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

  bool isEnemyDefenseArea(const Point & p) const { return isInBox(theirs.defense_area, p); }

  bool isFriendDefenseArea(const Point & p) const { return isInBox(ours.defense_area, p); }

  bool isDefenseArea(Point p) const { return isFriendDefenseArea(p) || isEnemyDefenseArea(p); }

  bool isFieldInside(Point p) const
  {
    Box field_box;
    field_box.min_corner() << -field_size.x() / 2.f, -field_size.y() / 2.f;
    field_box.max_corner() << field_size.x() / 2.f, field_size.y() / 2.f;
    return isInBox(field_box, p);
  }

  bool isBallPlacementArea(Point p) const
  {
    // During ball placement, all robots of the non-placing team have to keep
    // at least 0.5 meters distance to the line between the ball and the placement position
    // (the forbidden area forms a stadium shape).
    // ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement_interference
    //    Segment ball_placement_line;
    //    {Point(ball_placement_target), Point(ball.pos)};
    Segment ball_placement_line(ball_placement_target, ball.pos);
    return bg::distance(ball_placement_line, p) <= 0.5;
  }

  double getDefenseWidth() const { return ours.defense_area.max_corner().y() - ours.defense_area.min_corner().y(); }

  double getDefenseHeight() const { return ours.defense_area.max_corner().x() - ours.defense_area.min_corner().x(); }

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

  Box getOurDefenseArea() { return ours.defense_area; }

  Box getTheirDefenseArea() { return theirs.defense_area; }

  Point getOurGoalCenter() { return goal; }

  Point getTheirGoalCenter() { return Point(-goal.x(), goal.y()); }

  Point getBallPlacementTarget() { return ball_placement_target; }

  TeamInfo ours;

  TeamInfo theirs;

  Point field_size, defense_area_size, goal_size;

  Point goal;

  Point ball_placement_target;

  Ball ball;

  PlaySituationWrapper play_situation;

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber;

  std::vector<std::function<void(void)>> callbacks;

  crane_msgs::msg::WorldModel latest_msg;

  bool has_updated = false;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
