// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_
#define CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
class GoOverBall : public SkillBase<>
{
public:
  explicit GoOverBall(uint8_t id, std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<>("GoOverBall", id, world_model, DefaultStates::DEFAULT)
  {
    setParameter("next_target_x", 0.0);
    setParameter("next_target_y", 0.0);
    setParameter("margin", 0.5);
    setParameter("reach_threshold", 0.05);
    addStateFunction(
      DefaultStates::DEFAULT,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> SkillBase::Status {
        if (not has_started) {
          Point next_target{
            getParameter<double>("next_target_x"), getParameter<double>("next_target_y")};
          Vector2 r =
            (world_model->ball.pos - next_target).normalized() * getParameter<double>("margin");
          final_target_pos = world_model->ball.pos + r;
          intermediate_target_pos = std::make_pair(
            world_model->ball.pos + getVerticalVec(r), world_model->ball.pos - getVerticalVec(r));
          has_started = true;
        }

        command.lookAtBallFrom(final_target_pos);

        auto final_distance = (robot->pose.pos - final_target_pos).norm();
        auto [intermediate_distance, intermediate_point] = [&]() {
          auto d1 = (robot->pose.pos - intermediate_target_pos.first).norm();
          auto d2 = (robot->pose.pos - intermediate_target_pos.second).norm();
          if (d1 < d2) {
            return std::make_pair(d1, intermediate_target_pos.first);
          } else {
            return std::make_pair(d2, intermediate_target_pos.second);
          }
        }();

        std::cout << "final_distance: " << final_distance << std::endl;
        std::cout << "intermediate_distance: " << intermediate_distance << std::endl;

        if (intermediate_distance < final_distance && not has_passed_intermediate_target) {
          command.setTargetPosition(intermediate_point);
          if (intermediate_distance < getParameter<double>("reach_threshold")) {
            std::cout << "Reached intermediate target" << std::endl;
            has_passed_intermediate_target = true;
          }
        } else {
          command.setTargetPosition(final_target_pos);
        }

        if (final_distance < getParameter<double>("reach_threshold")) {
          return SkillBase::Status::SUCCESS;
        } else {
          return SkillBase::Status::RUNNING;
        }
      });
  }

  void print(std::ostream & out) const override
  {
    out << "[GoOverBall] ";
    if (
      has_passed_intermediate_target &&
      (robot->pose.pos - final_target_pos).norm() > getParameter<double>("reach_threshold")) {
      out << "中間地点へ向かっています";
    } else {
      out << "最終地点へ向かっています, 距離　" << (robot->pose.pos - final_target_pos).norm();
    }
  }

private:
  bool has_started = false;

  bool has_passed_intermediate_target = false;
  //  bool has_
  Point final_target_pos;

  std::pair<Point, Point> intermediate_target_pos;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__GO_OVER_BALL_HPP_
