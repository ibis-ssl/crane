// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef TEST_MOVE_HPP_
#define TEST_MOVE_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "crane_bt_executor/role/role_base.hpp"
#include "crane_bt_executor/skill/goalie.hpp"
#include "crane_bt_executor/skill/kick_to_target.hpp"
#include "crane_bt_executor/skill/move.hpp"
#include "crane_bt_executor/skill/pass_receive.hpp"
#include "crane_bt_executor/skill/spin_at_target.hpp"
#include "crane_bt_executor/skill/stop.hpp"
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
      if (r->robot_id_ == 4) {
        r->addChild(std::make_shared<Goalie>());
      }
      if (r->robot_id_ == 1) {
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
  void configure(RoleCommand cmd) override {}
  void onAssignUpdate() override {}
  void onParamUpdate() override {}

protected:
  std::vector<Point> points_;
};
#endif  // TEST_MOVE_HPP_
