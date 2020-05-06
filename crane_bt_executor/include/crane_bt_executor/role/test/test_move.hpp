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

#ifndef TEST_MOVE_HPP_
#define TEST_MOVE_HPP_

#include <memory>
#include <vector>
#include <iostream>
#include "crane_bt_executor/skill/move.hpp"
#include "crane_bt_executor/skill/stop.hpp"
#include "crane_bt_executor/skill/pass_receive.hpp"
#include "crane_bt_executor/skill/kick_to_target.hpp"
#include "crane_bt_executor/skill/spin_at_target.hpp"
#include "crane_bt_executor/role/role_base.hpp"
#include "crane_bt_executor/utils/target.hpp"

class TestMoveRole : public RoleBase
{
public:
  TestMoveRole()
  {
    MultiRobotBehavior m;
    uint8_t id = 0;
    for (int i = 0; i < 8; i++) {
      auto r = std::make_shared<SingleRobotSequence>();
      r->assignID(i);

      std::vector<Point> points;
      int place = 4 - i;
      auto target = TargetModule::buildPoint({0.5 * place, 0.5 * place});
      r->addChild(std::make_shared<Move>(target));
      r->addChild(std::make_shared<Stop>(2));
      Eigen::Rotation2D<float> rot;
      rot.angle() = M_PI_2;
      auto origin = TargetModule::buildPoint({0, 0});
      auto over_target = TargetModule::buildPoint(rot * Point(0.5 * place, 0.5 * place));
      r->addChild(std::make_shared<SpinAtTarget>(origin, over_target));
      if (r->robot_id_ == 0) {
        r->addChild(std::make_shared<KickToTarget>(TargetModule::buildPoint({4, 3}), 0.5f));
      }
      if (r->robot_id_ == 6) {
        r->addChild(std::make_shared<PassReceive>(TargetModule::buildPoint({4, 3})));
      }
      r->addChild(std::make_shared<Stop>());
      registerRobot(r);
      id++;
    }
    std::cout << "TestMoveRole" << std::endl;
  }
  void configure(RoleCommand cmd) override
  {}
  void onAssignUpdate() override {}
  void onParamUpdate() override {}

protected:
  std::vector<Point> points_;
};
#endif  // TEST_MOVE_HPP_
