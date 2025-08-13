// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_planner_plugins/ball_calibration_data_collector_planner.hpp>

namespace crane
{
static constexpr double kicker_x_offset = 4.0;

BallCalibrationDataCollectorPlanner::BallCalibrationDataCollectorPlanner(
  WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node &)
: PlannerBase("BallCalibrationDataCollector", world_model),
  current_state_(NewCollectorState::INITIALIZE),
  current_power_index_(0)
{
  auto current_time = rclcpp::Clock().now();
  state_start_time_ = current_time;
  last_ball_motion_time_ = current_time;
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
BallCalibrationDataCollectorPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext & context)
{
  // パラメータ定数定義
  const std::vector<double> kick_power_sequence = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

  if (robots.size() < 2 || !kicker || !retriever) {
    return {PlannerBase::Status::FAILURE, {}};
  }

  auto transition_to = [&](const NewCollectorState & next) {
    std::stringstream ss;
    ss << "[STATE TRANSITION] " << magic_enum::enum_name(current_state_);
    ss << " -> " << magic_enum::enum_name(next);
    current_state_ = next;
    state_start_time_ = rclcpp::Clock().now();
    RCLCPP_INFO(rclcpp::get_logger("BallCalibrationDataCollectorPlanner"), ss.str().c_str());
  };

  auto now = rclcpp::Clock().now();

  switch (current_state_) {
    case NewCollectorState::INITIALIZE: {
      // 初期配置: 両ロボットを開始位置に配置
      kicker->setTargetPosition(getKickerTargetPosition());
      retriever->setTargetPosition(getRetrieverWaitingPosition());

      if (kicker->getTargetDistance() < 0.1 && retriever->getTargetDistance() < 0.1) {
        if (std::abs(world_model->ball().pos.x() - kicker_x_offset * world_model->getOurSideSign()) > 1.0) {
          // ボールが
          transition_to(NewCollectorState::BALL_INTERCEPT);
        } else {
          // 両ロボットが目標位置に到達したらキック接近に移行
          transition_to(NewCollectorState::KICK_APPROACH);
        }
      }
    } break;

    case NewCollectorState::KICK_APPROACH: {
      // ボール後方への精密位置取り
      auto ball_pos = world_model->ball().pos;
      Point kick_approach_pos = ball_pos + Point(-0.2, 0.0);  // ボール後方0.2m

      kicker->setTargetPosition(kick_approach_pos).lookAtBall();

      // レトリーバーは待機
      retriever->setTargetPosition(getRetrieverWaitingPosition());

      // 位置・角度到達でキック実行に移行
      if (std::abs(world_model->ball().pos.x() - kicker_x_offset * world_model->getOurSideSign()) > 1.0) {
        transition_to(NewCollectorState::BALL_INTERCEPT);
      }else if (
        kicker->getRobot()->getDistance(kick_approach_pos) < 0.05 &&
        std::abs(kicker->getRobot()->pose.theta) < 0.1) {
        transition_to(NewCollectorState::KICK_EXECUTE);
      }
    } break;

    case NewCollectorState::KICK_EXECUTE: {
      // キック実行フェーズ
      // キッカー有効化とボールへの突進
      kicker->kickStraight(getNextKickPower(kick_power_sequence));
      kicker->setTargetPosition(world_model->ball().pos).disableBallAvoidance();

      // レトリーバーは予測回収位置への先行移動開始
      retriever->setTargetPosition(calculateOptimalInterceptPosition());

      // ボール速度変化でキック完了判定→迎撃開始
      if (world_model->ball().vel.norm() > 0.5) {
        transition_to(NewCollectorState::BALL_INTERCEPT);
        last_ball_motion_time_ = now;
      }
    } break;

    case NewCollectorState::BALL_INTERCEPT: {
      // 予測位置での効率的回収
      auto ball_pos = world_model->ball().pos;
      auto intercept_pos = calculateOptimalInterceptPosition();

      retriever->setTargetPosition(intercept_pos)
        .lookAtFrom(kicker->getRobot()->pose.pos, ball_pos).disableBallAvoidance();

      // キッカーは次回パス受け取り位置へ移動
      kicker->setTargetPosition(getKickerTargetPosition());

      // ボール制御確立で返球フェーズに移行
      if (retriever->getRobot()->getDistance(ball_pos) < 0.15) {
        std::cout << "[touch] intercept_pos: " << intercept_pos.x() << ", " << intercept_pos.y()
                  << ", " << std::endl;
        transition_to(NewCollectorState::BALL_RETURN);
      }else if ((now - state_start_time_).seconds() > 5.0) {
        std::cout << "[timeout] intercept_pos: " << intercept_pos.x() << ", " << intercept_pos.y()
                  << ", " << std::endl;
        transition_to(NewCollectorState::BALL_RETURN);
      }
    } break;

    case NewCollectorState::BALL_RETURN: {
      // キッカーへの正確な返球
      Point target = world_model->ball().pos;
      target += (getKickerTargetPosition() - target).normalized() * 0.5;
      if (world_model->ball().isStopped(0.1)) {
        retriever->setTargetPosition(target).lookAtFrom(getKickerTargetPosition(), target).disableBallAvoidance();
      } else {
        retriever->stopHere();
      }
      retriever->kickStraight([&]() {
        // 距離ベースのパワー決定
        if (double pass_distance = retriever->getRobot()->getDistance(getKickerTargetPosition());
            pass_distance > 4.0) {
          return 0.9;
        } else if (pass_distance > 2.0) {
          return 0.7;
        } else {
          return 0.5;
        }
      }());

      // キッカーはパス受け取り準備
      kicker->setTargetPosition(getKickerTargetPosition()).lookAt(retriever->getRobot()->pose.pos).disableBallAvoidance();
      kicker->kickStraight(0.0).dribble(0.3);

      // パス完了判定（時間ベース + キッカーのボール制御）
      double time_since_start = (now - state_start_time_).seconds();
      bool kicker_has_ball = kicker->getRobot()->getDistance(world_model->ball().pos) < 0.2;

      if (time_since_start > 3.0 || kicker_has_ball) {
        static int count = 0;
        if (++count > 20) {
          // 次サイクル開始
          kicker->dribble(0.0);
          transition_to(NewCollectorState::KICK_APPROACH);  // 初期化スキップして直接キック開始
          count = 0;
        }

      }
    } break;
  }

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  robot_commands.emplace_back(kicker->getMsg());
  robot_commands.emplace_back(retriever->getMsg());
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto BallCalibrationDataCollectorPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  // 最小2台、最大2台のロボットを選択
  auto selected = this->getSelectedRobotsByScore(
    std::min(static_cast<uint8_t>(2), selectable_robots_num), selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // IDが小さい順に優先
      return 20.0 - static_cast<double>(robot->id);
    },
    prev_roles, context);

  if (selected.size() >= 2) {
    // RobotCommandWrapperを作成
    kicker =
      std::make_shared<RobotCommandWrapper>("ball_calibration_kicker", selected[0], world_model);
    retriever =
      std::make_shared<RobotCommandWrapper>("ball_calibration_retriever", selected[1], world_model);
  }

  return selected;
}

Point BallCalibrationDataCollectorPlanner::getKickerTargetPosition() const
{
  // 自陣ゴール位置からx方向にオフセット
  return Point(kicker_x_offset * world_model->getOurSideSign(), 0.0);
}

Point BallCalibrationDataCollectorPlanner::getRetrieverWaitingPosition() const
{
  // フィールド中央やや前方で待機
  return Point(1.0, 0.0);
}

double BallCalibrationDataCollectorPlanner::getNextKickPower(
  const std::vector<double> & kick_power_sequence)
{
  if (current_power_index_ >= kick_power_sequence.size()) {
    current_power_index_ = 0;  // 循環
  }
  return kick_power_sequence[current_power_index_++];
}

Point BallCalibrationDataCollectorPlanner::calculateOptimalInterceptPosition() const
{
  auto ball = world_model->ball();

  // ボール軌道の予測
  Point predicted_stop = ball.getPredictedPosition(10.0);

  Segment ball_line{ball.pos, predicted_stop};
  Segment goal_line{
    world_model->getTheirGoalCenter() + Point(0, world_model->fieldSize().y()),
    world_model->getTheirGoalCenter() - Point(0, world_model->fieldSize().y())};
  return [&]() -> Point {
    if (auto intersections = getIntersections(ball_line, goal_line); not intersections.empty()) {
      // 交点がある -> フィールド外に飛び出す
      std::cout << "intersection: " << intersections.front().x() << ", "
                << intersections.front().y() << std::endl;
      return intersections.front() - ball.vel.normalized() * 0.3;
    } else {
      return predicted_stop + (ball.pos - kicker->getRobot()->pose.pos).normalized() * 0.3;
    }
  }();
}

}  // namespace crane
