// Copyright (c) 2024 ibis-ssl
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
private:
  int current_section = 0;
  rclcpp::Time latest_section_start_time;
  rclcpp::Clock clock;
  std::unordered_map<std::string, std::function<Point(const int, const double)>> motion_functions;
public:
  explicit TestMotionPosition(RobotCommandWrapperBase::SharedPtr & base)
  : SkillBase("TestMotion", base), clock(RCL_ROS_TIME)
  {
    // segment / square / circle
    setParameter("motion", "segment");
    setParameter("origin", Point::Zero());
    setParameter("section_time", 5.);
    setParameter("sleep_time", 1.);
    setParameter("distance", 1.);
    latest_section_start_time = clock.now();
    motion_functions["segment"] = [&](const int section, const double parameter) {
      return getPositionSegment(section, parameter);
    };
    motion_functions["square"] = [&](const int section, const double parameter) {
      return getPositionSquare(section, parameter);
    };
    motion_functions["circle"] = [&](const int section, const double parameter) {
      return getPositionCircle(section, parameter);
    };
  }

  void reset(const std::string & motion)
  {
    setParameter("motion", motion);
    current_section = 0;
    latest_section_start_time = clock.now();
  }

  Status update() override
  {
    const std::string motion_name = getParameter<std::string>("motion");
    const double section_time = getParameter<double>("section_time");
    const double sleep_time = getParameter<double>("sleep_time");
    if (auto motion = motion_functions.find(motion_name); motion != motion_functions.end()) {
      if ( clock.now() - latest_section_start_time >= rclcpp::Duration::from_seconds(section_time + sleep_time)) {
        latest_section_start_time = clock.now();
        current_section++;
      }
      const double parameter = (clock.now() - latest_section_start_time).seconds() / section_time;
      const Point position = motion->second(current_section, parameter);
      command.setTargetPosition(position);
    }else {
      std::stringstream what;
      what << "TestMotionPositionでサポートされていないモーション \"" << motion_name << "\"が指定されています。";
      what << "有効なのは [";
      for (const auto & motion : motion_functions) {
        what << motion.first << ", ";
      }
      what << "] です。";
      throw std::runtime_error(what.str());
    }
    return Status::RUNNING;
  }

  // 往復
  Point getPositionSegment(const int section, const double parameter) const
  {
    const double distance = getParameter<double>("distance");
    const Point origin = getParameter<Point>("origin");
    switch (section % 2) {
      case 0:
        return origin + Point(distance * (parameter - 0.5), 0);
      case 1:
        return origin + Point(distance * (0.5 - parameter), 0);
      default:
        throw std::runtime_error("Invalid section");
    }
  }

  // 四角
  Point getPositionSquare(const int section, const double parameter) const
  {
    const double distance = getParameter<double>("distance");
    const Point origin = getParameter<Point>("origin");

    const double up = distance * (parameter - 0.5);
    const double down = distance * (0.5 - parameter);

    switch (section % 4) {
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
  Point getPositionCircle([[maybe_unused]] const int section, const double parameter) const
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
