// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__DEFENDER_TACTIC_HPP_
#define CRANE_TACTICS__DEFENDER_TACTIC_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_tactics/defense_functions.hpp>
#include <crane_tactics/tactic_base.hpp>
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

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // 守備ライン位置に近いロボットを優先
    auto wm = world_model;  // shared_ptrをコピー
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      Segment ball_line{wm->goal(), wm->ball().pos};
      auto parameter = getDefenseLinePointParameter(ball_line, wm);
      if (not parameter) {
        return 1000.0;  // パラメータが無効な場合は大きなコスト
      }
      const auto defense_point = getDefenseLinePoint(parameter.value(), wm);
      return robot->getDistance(defense_point);
    };
  }
};

}  // namespace crane
#endif  // CRANE_TACTICS__DEFENDER_TACTIC_HPP_
