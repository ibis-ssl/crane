// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_BT_EXECUTOR__UTILS__CONTROL_TARGET_BUILDER_HPP_
#define CRANE_BT_EXECUTOR__UTILS__CONTROL_TARGET_BUILDER_HPP_

#include <experimental/optional>

#include <consai2_msgs/ControlTarget.h>

#include <deque>
#include <memory>

#include "crane_bt_executor/utils/eigen_adapter.hpp"

class WorldModel;

class BallInfo;

class RobotNode;

struct Obstacle
{
  Point center;
  float radius;
};

using Obstacles = std::deque<Obstacle>;

class ControlTargetBuilder
{
protected:
  consai2_msgs::ControlTarget cmd;
  bool avoid_ball = true;
  bool avoid_defence_area = true;
  bool avoid_enemy = true;
  bool avoid_friend = true;
  bool remake_avoid_point = false;
  std::experimental::optional<Point> avoid_point = std::experimental::nullopt;
//    bool generate_path = false;

public:
  explicit ControlTargetBuilder(uint8_t id)
  {
    cmd.robot_id = id;
    reset();
  }

  void reset()
  {
    cmd.control_enable = false;
    cmd.goal_velocity.x = 0.0f;
    cmd.goal_velocity.y = 0.0f;
    cmd.goal_velocity.theta = 0.0f;
    cmd.kick_power = 0.0f;
    cmd.chip_enable = false;
    cmd.dribble_power = 0.0f;
    avoid_point = std::experimental::nullopt;
  }

  void resetPath()
  {
    cmd.path.clear();
  }

  ControlTargetBuilder & addKick(float power = 1.0f)
  {
    cmd.kick_power = power;
    return *this;
  }

  ControlTargetBuilder & addDribble(float power)
  {
    cmd.dribble_power = power;
    return *this;
  }

  ControlTargetBuilder & addGoalVelocity(geometry_msgs::Pose2D vel)
  {
    cmd.goal_velocity = vel;
    return *this;
  }

  ControlTargetBuilder & addGoalVelocity(Velocity v, float omega)
  {
    cmd.goal_velocity.x = v.x();
    cmd.goal_velocity.y = v.y();
    cmd.goal_velocity.theta = omega;
    return *this;
  }

  ControlTargetBuilder & addNoAvoidanceFriend()
  {
    avoid_friend = false;
    return *this;
  }

  ControlTargetBuilder & addNoAvoidanceEnemy()
  {
    avoid_enemy = false;
    return *this;
  }

  ControlTargetBuilder & addNoAvoidanceRobot()
  {
    return addNoAvoidanceEnemy().addNoAvoidanceFriend();
  }

  ControlTargetBuilder & addNoAvoidanceBall()
  {
    avoid_ball = false;
    return *this;
  }

  ControlTargetBuilder & addNoAvoidanceDefenceArea()
  {
    avoid_defence_area = false;
    return *this;
  }

  ControlTargetBuilder &
  addTargetPose(Point pos, float theta, float threshold_dist = 0.2f, float threshold_angle = 0.0f);

  ControlTargetBuilder &
  addTargetPose(
    geometry_msgs::Pose2D pose, float threshold_dist = 0.2f,
    float threshold_angle = 0.0f);

  consai2_msgs::ControlTarget build(const WorldModel & world_model);

  std::experimental::optional<geometry_msgs::Pose2D> getFrontTarget()
  {
    if (cmd.path.empty()) {
      return std::experimental::nullopt;
    }
    return std::experimental::make_optional(cmd.path.front());
  }

  ControlTargetBuilder & setTargetTheta(float theta_rad)
  {
    if (!cmd.path.empty()) {
      cmd.path.front().theta = theta_rad;
    }
    return *this;
  }

  ControlTargetBuilder & addControlEnable()
  {
    cmd.control_enable = 1;
    return *this;
  }

  uint8_t getID()
  {
    return cmd.robot_id;
  }

protected:
  /**
   * @note 返り値の型はObstacle(center,radius)の配列だが，radiusをsegmentからの距離として使っている
   */
  Obstacles
  getObstaclesDistanceFromSegment(
    const Point & p1, const Point & p2,
    const Obstacles & obstacles, const float THRESHOLD_DISTANCE);

  Obstacles
  getObstaclesDistanceFromPoint(const Point & point, const Obstacles & obstacles);

public:
  void generateAvoidingPoints(const WorldModel & world_model, std::shared_ptr<RobotNode> robot);

  /**
   * @note この関数を呼ぶ前にgenerateAvoidPoints関数を呼ぶこと
   * @return 生成されたControlTarget
   */
  consai2_msgs::ControlTarget getControlTarget()
  {
    return cmd;
  }

protected:
  std::experimental::optional<Point>
  generateAvoidPoint(
    const WorldModel & world_model, const Obstacles & obst_dist,
    std::shared_ptr<RobotNode> robot);

  static float & getEnemyAvoidanceOffset()
  {
    static float offset = 0.3f;
    return offset;
  }

  static float & getFriendAvoidanceOffset()
  {
    static float offset = 0.3f;
    return offset;
  }

  static float & getBallAvoidanceOffset()
  {
    static float offset = 0.3f;
    return offset;
  }

  static float & getDefenceAreaAvoidanceOffset()
  {
    static float offset = 0.3f;
    return offset;
  }

  void addEnemyToObstacle(const WorldModel & world_model, Obstacles & obstacles);

  void addFriendToObstacle(const WorldModel & world_model, Obstacles & obstacles);

  void addBallToObstacle(const WorldModel & world_model, Obstacles & obstacles);
};
#endif  // CRANE_BT_EXECUTOR__UTILS__CONTROL_TARGET_BUILDER_HPP_
