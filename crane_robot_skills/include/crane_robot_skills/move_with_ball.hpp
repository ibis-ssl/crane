// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
#define CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_

#include <chrono>
#include <crane_basics/eigen_adapter.hpp>
#include <crane_basics/time.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <optional>
#include <string>

namespace crane::skills
{
/**
 * ボールを持って移動する
 * 前提：GetBallContactが成功していること
 */

enum class MoveWithBallStates { SUCCESS, RUNNING, FAILURE };
class MoveWithBall : public SkillBase
{
public:
  explicit MoveWithBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm);

  Status update(const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  Point getTargetPoint(const Point & target_pos);

  void setTargetPoint(const Point & target_point)
  {
    setParameter("target_x", target_point.x());
    setParameter("target_y", target_point.y());
    //    setParameter("target_theta", target_pose.theta);
  }

  auto parseTargetPoint() const -> Point
  {
    return Point{getParameter<double>("target_x"), getParameter<double>("target_y")};
  }

  void print(std::ostream & out) const override
  {
    out << "[MoveWithBall] " << phase;
    out << ", target: " << getParameter<double>("target_x") << ", "
        << getParameter<double>("target_y");
  }

  std::string phase;

  std::optional<std::chrono::steady_clock::time_point> ball_stabilizing_start_time = std::nullopt;

  double target_theta;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__MOVE_WITH_BALL_HPP_
