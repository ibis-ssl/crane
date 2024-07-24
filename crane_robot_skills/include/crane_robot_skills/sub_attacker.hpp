// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SUB_ATTACKER_HPP_
#define CRANE_ROBOT_SKILLS__SUB_ATTACKER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/interval.hpp>
#include <crane_game_analyzer/evaluations/evaluations.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane::skills
{
class SubAttacker : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit SubAttacker(RobotCommandWrapperBase::SharedPtr & base);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & os) const override { os << "[SubAttacker]"; }

  static std::vector<std::pair<double, Point>> getPositionsWithScore(
    Segment ball_line, Point next_target, const WorldModelWrapper::SharedPtr & world_model);

  static std::vector<Point> getPoints(Segment ball_line, double interval);

  static std::vector<Point> getPoints(Point center, float unit, int unit_num);

  static std::vector<Point> getDPPSPoints(
    Point center, double r_resolution, int theta_div_num,
    const WorldModelWrapper::SharedPtr & world_model);

  static double getPointScore(
    Point p, Point next_target, const WorldModelWrapper::SharedPtr & world_model);
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SUB_ATTACKER_HPP_
