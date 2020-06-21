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

#ifndef CRANE_BT_EXECUTOR__UTILS__TARGET_HPP_
#define CRANE_BT_EXECUTOR__UTILS__TARGET_HPP_

#include <memory>
#include "crane_world_observer/world_model.hpp"
#include "crane_geometry/boost_geometry.hpp"


class TargetPointBase
{
public:
  virtual Point getPoint(const WorldModel::SharedPtr world_model) = 0;
};

class TargetSegmentBase
{
public:
  virtual Segment getSegment(const WorldModel::SharedPtr world_model) = 0;
};

class TargetBall : public TargetPointBase
{
public:
  Point getPoint(const WorldModel::SharedPtr world_model) override
  {
    return world_model->ball.pos;
  }
};

class TargetBallLine : public TargetSegmentBase
{
public:
  Segment getSegment(const WorldModel::SharedPtr world_model) override;
};

class TargetFriendRobot : public TargetPointBase
{
public:
  uint8_t id;

public:
  explicit TargetFriendRobot(uint8_t id)
  : id(id) {}

  Point getPoint(const WorldModel::SharedPtr world_model) override
  {
    return world_model->ours.robots.at(id)->pose.pos;
  }
};

class TargetEnemyRobot : public TargetPointBase
{
public:
  uint8_t id;

public:
  explicit TargetEnemyRobot(uint8_t id)
  : id(id) {}

  Point getPoint(const WorldModel::SharedPtr world_model) override
  {
    return world_model->theirs.robots.at(id)->pose.pos;
  }
};

class TargetPoint : public TargetPointBase
{
public:
  Point point;

public:
  TargetPoint(Point point)  // NOLINT
  : point(point) {}

  Point getPoint(const WorldModel::SharedPtr world_model) override
  {
    return point;
  }
};

class TargetModule
{
private:
  std::shared_ptr<TargetPointBase> base;

public:
  TargetModule() {}
  explicit TargetModule(std::shared_ptr<TargetPointBase> base)

  : base(base) {}

  Point getPoint(const WorldModel::SharedPtr world_model)
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

//  template<typename TOp>
//  static buildOperated(TargetModule a,TargetModule b)
//  {
//    auto module = TargetModule(std::make_shared<TargetOperation<TOp>>(a,b));
//    return module;
//  }
};


class TargetModule;

template<typename TOperator>
class TargetOperation : public TargetPointBase
{
public:
  TargetOperation(TargetModule a, TargetModule b)
  : a_(a), b_(b) {}
  Point getPoint(const WorldModel::SharedPtr world_model) override
  {
    return TOperator()(a_.getPoint(world_model), b_.getPoint(world_model));
  }

protected:
  TargetModule a_, b_;
};

//  class TargetLineModule
//  {
//  private:
//    std::shared_ptr<TargetSegmentBase> base;
//
//  public:
//    TargetLineModule() {}
//
//    explicit TargetLineModule(std::shared_ptr<TargetSegmentBase> base)
//    : base(base) {}
//
//    Segment getLine(const WorldModel::SharedPtr world_model)
//    {
//      return base->getSegment(world_model);
//    }
//
//    static TargetLineModule buildBallLine()
//    {
//      auto ball_line = std::make_shared<TargetBallLine>();
//      auto module = TargetLineModule(ball_line);
//      return module;
//    }
//  };
#endif  // CRANE_BT_EXECUTOR__UTILS__TARGET_HPP_
