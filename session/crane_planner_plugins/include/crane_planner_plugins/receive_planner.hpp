// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__RECEIVE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__RECEIVE_PLANNER_HPP_

#include <crane_game_analyzer/evaluations/evaluations.hpp>
#include <crane_geometry/eigen_adapter.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/pass_info.hpp>
#include <crane_msgs/msg/receiver_plan.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_msgs/srv/pass_request.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/receiver.hpp>
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
/**
 * ReceivePlannerは現在進行形でパスされているボールに対して，
 * ボールを受けるロボット(passer),その次にボールを受けるロボット(receiver)を指定するだけで
 * 最適なパス地点を計算し，その2台に対する指令を生成するプランナーです
 *
 * 何もないとき：パスの通る隙の多い地点に陣取る
 * ボールが向かってきているとき：最適なパス地点を計算し，その地点に向かう
 *
 */
class ReceivePlanner : public PlannerBase
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

    double score = 0.0;
  };

  COMPOSITION_PUBLIC
  explicit ReceivePlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("receive", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    auto selected = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        return 100. / world_model->getSquareDistanceFromRobotToBall(robot->id);
      },
      prev_roles);
    if (selected.empty()) {
      return {};
    } else {
      receiver_skill = std::make_shared<skills::Receiver>(selected.front(), world_model);
      return {selected.front()};
    }
  }

private:
  std::shared_ptr<skills::Receiver> receiver_skill = nullptr;
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__RECEIVE_PLANNER_HPP_
