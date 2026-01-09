// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__DEFENDER_TACTIC_HPP_
#define CRANE_PLANNER_PLUGINS__DEFENDER_TACTIC_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_planner_plugins/defense_functions.hpp>
#include <crane_planner_plugins/tactic_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class DefenderTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC
  explicit DefenderTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("defender", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    Segment ball_line{world_model->goal(), world_model->ball().pos};
    auto parameter = getDefenseLinePointParameter(ball_line, world_model);
    if (not parameter) {
      return {};
    }
    const auto defense_point = getDefenseLinePoint(parameter.value(), world_model);
    auto selected = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this, defense_point](const std::shared_ptr<RobotInfo> & robot) {
        if (robot->id == world_model->getOurGoalieId()) {
          // ゴールキーパーは選出しない
          return -100.;
        } else {
          // defense pointに近いほどスコアが高い
          return 100. - robot->getSquareDistance(defense_point);
        }
      },
      prev_roles);

    return selected;
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__DEFENDER_TACTIC_HPP_
