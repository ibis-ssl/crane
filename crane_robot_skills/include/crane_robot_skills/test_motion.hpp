// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__TEST_MOTION_HPP_
#define CRANE_ROBOT_SKILLS__TEST_MOTION_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class TestMotionPosition : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit TestMotionPosition(RobotCommandWrapperBase::SharedPtr & base)
  : SkillBase("TestMotion", base)
  {
    setParameter("motion", "一文字");
    setParameter("origin", Point::Zero());
    setParameter("section_time", 5.);
    setParameter("sleep_time", 1.);
    setParameter("distance", 1.);
  }

  Status update() override
  {
    command.stopHere();
    return Status::RUNNING;
  }

  // 往復
  Point getPositionA(const int section, const double parameter) const
  {
    const double distance = getParameter<double>("distance");
    const Point origin = getParameter<Point>("origin");
    switch (section) {
      case 0:
        return origin + Point(distance * (parameter - 0.5), 0);
      case 1:
        return origin + Point(distance * (0.5 - parameter), 0);
      default:
        throw std::runtime_error("Invalid section");
    }
  }

  // 四角
  Point getPositionB(const int section, const double parameter) const
  {
    const double distance = getParameter<double>("distance");
    const Point origin = getParameter<Point>("origin");

    const double up = distance * (parameter - 0.5);
    const double down = distance * (0.5 - parameter);

    switch (section) {
      case 0:
        // x : minus -> plus, y: plus
        return origin + Point(up, distance * 0.5);
      case 1:
        // x : plus, y: plus -> minus
        return origin + Point(distance * 0.5, down);
      case 2:
        // x: plus -> minus, y: minus
        return origin + Point(down, -distance * 0.5);
      case 3:
        // x: minus, y: minus -> plus
        return origin + Point(-distance * 0.5, up);
      default:
        throw std::runtime_error("Invalid section");
    }
  }

  // 円
  Point getPositionC(const double parameter) const
  {
    const double distance = getParameter<double>("distance");
    const Point origin = getParameter<Point>("origin");
    const double radius = distance * 0.5;
    const double theta = 2.0 * M_PI * parameter;
    return origin + getNormVec(theta) * radius;
  }

  void print(std::ostream & os) const override { os << "[Idle]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__TEST_MOTION_HPP_
