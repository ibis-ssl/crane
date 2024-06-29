// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__TARGET_GEOMETRY_HPP_
#define CRANE_BASICS__TARGET_GEOMETRY_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <memory>

namespace crane
{
class TargetPointBase
{
public:
  virtual Point getPoint(const WorldModelWrapper::SharedPtr & world_model) = 0;
};

class TargetSegmentBase
{
public:
  virtual Segment getSegment(const WorldModelWrapper::SharedPtr & world_model) = 0;
};

class TargetBall : public TargetPointBase
{
public:
  Point getPoint(const WorldModelWrapper::SharedPtr & world_model) override
  {
    return world_model->ball.pos;
  }
};

class TargetBallLine : public TargetSegmentBase
{
public:
  Segment getSegment(const WorldModelWrapper::SharedPtr & world_model) override
  {
    return Segment(
      world_model->ball.pos, world_model->ball.pos + world_model->ball.vel.normalized() * 20.0);
  }
};

class TargetFriendRobot : public TargetPointBase
{
public:
  uint8_t id;

public:
  explicit TargetFriendRobot(uint8_t id) : id(id) {}

  Point getPoint(const WorldModelWrapper::SharedPtr & world_model) override
  {
    return world_model->getOurRobot(id)->pose.pos;
  }
};

class TargetEnemyRobot : public TargetPointBase
{
public:
  uint8_t id;

public:
  explicit TargetEnemyRobot(uint8_t id) : id(id) {}

  Point getPoint(const WorldModelWrapper::SharedPtr & world_model) override
  {
    return world_model->getTheirRobot(id)->pose.pos;
  }
};

class TargetPoint : public TargetPointBase
{
public:
  Point point;

public:
  explicit TargetPoint(Point point) : point(point) {}

  Point getPoint(const WorldModelWrapper::SharedPtr & world_model) override { return point; }
};

class TargetModule
{
private:
  std::shared_ptr<TargetPointBase> base;

public:
  TargetModule() {}

  explicit TargetModule(std::shared_ptr<TargetPointBase> base) : base(base) {}

  Point getPoint(const WorldModelWrapper::SharedPtr & world_model)
  {
    return base->getPoint(world_model);
  }

  static TargetModule buildBall()
  {
    auto ball = std::make_shared<TargetBall>();
    auto module = TargetModule(ball);
    return module;
  }

  static TargetModule buildFriend(uint8_t id)
  {
    auto friend_robot = std::make_shared<TargetFriendRobot>(id);
    auto module = TargetModule(friend_robot);
    return module;
  }

  static TargetModule buildEnemy(uint8_t id)
  {
    auto enemy_robot = std::make_shared<TargetEnemyRobot>(id);
    auto module = TargetModule(enemy_robot);
    return module;
  }

  static TargetModule buildPoint(Point point)
  {
    auto module = TargetModule(std::make_shared<TargetPoint>(point));
    return module;
  }
};

class TargetSegmentModule
{
private:
  std::shared_ptr<TargetSegmentBase> base;

public:
  TargetSegmentModule() {}

  explicit TargetSegmentModule(std::shared_ptr<TargetSegmentBase> base) : base(base) {}

  Segment getSegment(const WorldModelWrapper::SharedPtr & world_model)
  {
    return base->getSegment(world_model);
  }

  static TargetSegmentModule buildBallLine()
  {
    auto ball_line = std::make_shared<TargetBallLine>();
    auto module = TargetSegmentModule(ball_line);
    return module;
  }
};

}  // namespace crane
#endif  // CRANE_BASICS__TARGET_GEOMETRY_HPP_
