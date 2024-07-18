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
class SubAttacker : public SkillBase<>
{
public:
  enum class ReceivePhase {
    NONE,
    MOVE_ROUGH,
    MOVE_TO_EXPECTED_BALL_LINE,
    MOVE_TO_ACTUAL_BALL_LINE,
  };
  struct SessionInfo
  {
    int receiver_id;
  } session_info;

  struct PositionsWithScore
  {
    Point passer_pos;

    Point receiver_pos;

    double score;
  };

  explicit SubAttacker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  void print(std::ostream & os) const override { os << "[Receive]"; }

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
