// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/stream.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_planner_plugins/defense_functions.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class TotalDefensePlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Goalie> goalie;

  std::vector<std::shared_ptr<RobotCommandWrapperPosition>> first_threat_defenders;

  std::vector<std::shared_ptr<RobotCommandWrapperPosition>> second_threat_defenders;

public:
  COMPOSITION_PUBLIC
  explicit TotalDefensePlanner(WorldModelWrapper::SharedPtr & world_model)
  : PlannerBase("total_defense", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override;

  std::vector<Point> getDefenseArcPoints(const int robot_num, const Segment & ball_line) const;

  // defense_pointを中心にrobot_num台のロボットをdefense_line上に等間隔に配置する
  std::vector<Point> getDefenseLinePoints(const int robot_num, const Segment & ball_line) const;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override
  {
    std::vector<uint8_t> selected;
    std::vector<uint8_t> remaining_robots = selectable_robots;
    // キーパーを確保
    auto goalie_id = world_model->getOurGoalieId();
    if (ranges::count(selectable_robots, goalie_id) != 0) {
      selected.push_back(goalie_id);
      remaining_robots |= ranges::actions::remove_if([goalie_id](auto elem){return elem == goalie_id;});
      auto base = std::make_shared<RobotCommandWrapperBase>(
        "goalie", world_model->getOurGoalieId(), world_model);
      goalie = std::make_shared<skills::Goalie>(base);
    }

    // TODO(HansRobo): Attackerを供出するかどうかの実装

    // 直接脅威へのディフェンダー
    Segment ball_line{world_model->goal, world_model->ball.pos};
    auto parameter = getDefenseLinePointParameter(ball_line, world_model);
    if (not parameter) {
      return {};
    }
    const auto defense_point = getDefenseLinePoint(parameter.value(), world_model);
    auto selected_first_defenders = this->getSelectedRobotsByScore(
      selectable_robots_num - selected.size(), remaining_robots,
      [this, defense_point](const std::shared_ptr<RobotInfo> & robot) {
        // defense pointに近いほどスコアが高い
        return 100. - world_model->getSquareDistanceFromRobot(robot->id, defense_point);
      },
      prev_roles, context);

    ranges::copy(selected_first_defenders, ranges::back_inserter(selected));
    ranges::remove_if(remaining_robots, [selected_first_defenders](const uint8_t id) {
      return ranges::any_of(
        selected_first_defenders, [id](const uint8_t selected_id) { return selected_id == id; });
    });

    // TODO(HansRobo): 間接脅威へのディフェンダー

    return selected;
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_
