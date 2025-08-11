// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_planner_plugins/ball_calibration_data_collector_planner.hpp>

namespace crane
{

BallCalibrationDataCollectorPlanner::BallCalibrationDataCollectorPlanner(
  WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: PlannerBase("BallCalibrationDataCollector", world_model),
  current_state_(CollectorState::SETUP_POSITIONS),
  kicker_robot_id_(0),
  retriever_robot_id_(1),
  kicker_skill_(nullptr),
  retriever_skill_(nullptr),
  kicker_x_offset_(1.0),
  kick_power_sequence_{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
  current_power_index_(0),
  ball_stop_timeout_(5.0),
  field_boundary_margin_(0.3),
  data_collection_cycles_(20),
  current_cycle_(0),
  kick_executed_(false),
  node_(node)
{
  // パラメータの宣言と初期化
  node_.declare_parameter("calibration.kicker_x_offset", kicker_x_offset_);
  node_.declare_parameter("calibration.ball_stop_timeout", ball_stop_timeout_);
  node_.declare_parameter("calibration.field_boundary_margin", field_boundary_margin_);
  node_.declare_parameter(
    "calibration.data_collection_cycles", static_cast<int64_t>(data_collection_cycles_));

  // パラメータの読み込み
  kicker_x_offset_ = node_.get_parameter("calibration.kicker_x_offset").as_double();
  ball_stop_timeout_ = node_.get_parameter("calibration.ball_stop_timeout").as_double();
  field_boundary_margin_ = node_.get_parameter("calibration.field_boundary_margin").as_double();
  data_collection_cycles_ =
    static_cast<size_t>(node_.get_parameter("calibration.data_collection_cycles").as_int());

  state_start_time_ = node_.get_clock()->now();
  last_ball_motion_time_ = state_start_time_;

  RCLCPP_INFO(
    node_.get_logger(),
    "BallCalibrationDataCollector initialized: offset=%.2fm, timeout=%.1fs, cycles=%zu",
    kicker_x_offset_, ball_stop_timeout_, data_collection_cycles_);
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
BallCalibrationDataCollectorPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext & context)
{
  if (robots.size() < 2) {
    RCLCPP_WARN(node_.get_logger(), "データ収集には最低2台のロボットが必要です");
    return {PlannerBase::Status::FAILURE, {}};
  }

  // ロボットIDの設定
  kicker_robot_id_ = robots[0].id;
  retriever_robot_id_ = robots[1].id;

  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  auto current_time = node_.get_clock()->now();

  // 状態遷移ロジック
  switch (current_state_) {
    case CollectorState::SETUP_POSITIONS: {
      // キッカーを目標位置に配置
      auto kicker_command =
        std::make_shared<RobotCommandWrapper>("calibration_kicker", kicker_robot_id_, world_model);
      auto kicker_target = getKickerTargetPosition();
      kicker_command->setTargetPosition(kicker_target);
      robot_commands.emplace_back(kicker_command->getMsg());

      // 球拾いロボットを待機位置に配置
      auto retriever_command = std::make_shared<RobotCommandWrapper>(
        "calibration_retriever", retriever_robot_id_, world_model);
      auto retriever_target = getRetrieverWaitingPosition();
      retriever_command->setTargetPosition(retriever_target);
      robot_commands.emplace_back(retriever_command->getMsg());

      // 位置到達チェック
      auto kicker_robot = world_model->getOurRobot(kicker_robot_id_);
      auto retriever_robot = world_model->getOurRobot(retriever_robot_id_);

      if (kicker_robot && retriever_robot) {
        double kicker_dist = (kicker_robot->pose.pos - kicker_target).norm();
        double retriever_dist = (retriever_robot->pose.pos - retriever_target).norm();

        if (kicker_dist < 0.1 && retriever_dist < 0.1) {
          current_state_ = CollectorState::KICK_PREPARATION;
          state_start_time_ = current_time;
          RCLCPP_INFO(node_.get_logger(), "位置設定完了、キック準備に移行");
        }
      }
    } break;

    case CollectorState::KICK_PREPARATION: {
      // キックスキルの初期化
      if (!kicker_skill_) {
        auto kicker_command =
          std::make_shared<RobotCommandWrapper>("kick", kicker_robot_id_, world_model);
        kicker_skill_ = std::make_shared<skills::Kick>(kicker_command);

        // キックパワーの設定
        double kick_power = getNextKickPower();
        kicker_skill_->setParameter("kick_power", kick_power);
        kicker_skill_->setParameter("target_theta", 0.0);  // 正面方向

        RCLCPP_INFO(
          node_.get_logger(), "キック準備: パワー=%.2f (サイクル %zu/%zu)", kick_power,
          current_cycle_ + 1, data_collection_cycles_);
      }

      // キッカーロボット
      auto status = kicker_skill_->run();
      robot_commands.emplace_back(kicker_skill_->getRobotCommand());

      // 球拾いロボットは待機
      auto retriever_command = std::make_shared<RobotCommandWrapper>(
        "calibration_retriever", retriever_robot_id_, world_model);
      retriever_command->setTargetPosition(getRetrieverWaitingPosition());
      robot_commands.emplace_back(retriever_command->getMsg());

      // キック実行チェック
      if (status == crane::skills::Status::SUCCESS || kick_executed_) {
        current_state_ = CollectorState::EXECUTING_KICK;
        state_start_time_ = current_time;
        kick_executed_ = true;
        RCLCPP_INFO(node_.get_logger(), "キック実行開始");
      }
    } break;

    case CollectorState::EXECUTING_KICK: {
      // キック実行中
      if (kicker_skill_) {
        auto status = kicker_skill_->run();
        robot_commands.emplace_back(kicker_skill_->getRobotCommand());

        if (status == crane::skills::Status::SUCCESS) {
          current_state_ = CollectorState::WAITING_BALL_STOP;
          state_start_time_ = current_time;
          last_ball_motion_time_ = current_time;
          RCLCPP_INFO(node_.get_logger(), "キック完了、ボール停止待機");
        }
      }

      // 球拾いロボットは待機
      auto retriever_command = std::make_shared<RobotCommandWrapper>(
        "calibration_retriever", retriever_robot_id_, world_model);
      retriever_command->setTargetPosition(getRetrieverWaitingPosition());
      robot_commands.emplace_back(retriever_command->getMsg());
    } break;

    case CollectorState::WAITING_BALL_STOP: {
      // ボールの動きを監視
      if (world_model->ball().vel.norm() > 0.1) {
        last_ball_motion_time_ = current_time;
      }

      // フィールド外予測または完全停止チェック
      bool should_retrieve = false;
      double time_since_motion = (current_time - last_ball_motion_time_).seconds();
      double time_since_start = (current_time - state_start_time_).seconds();

      if (willBallStopOutsideField()) {
        RCLCPP_INFO(node_.get_logger(), "フィールド外停止予測、早期キャッチモード");
        should_retrieve = true;
      } else if (isBallFullyStopped() && time_since_motion > 1.0) {
        RCLCPP_INFO(node_.get_logger(), "ボール完全停止確認");
        should_retrieve = true;
        ball_stop_position_ = world_model->ball().pos;
      } else if (time_since_start > ball_stop_timeout_) {
        RCLCPP_WARN(node_.get_logger(), "ボール停止タイムアウト");
        should_retrieve = true;
      }

      if (should_retrieve) {
        current_state_ = CollectorState::BALL_RETRIEVAL;
        state_start_time_ = current_time;
      }

      // ロボットは現在位置で待機
      auto kicker_command =
        std::make_shared<RobotCommandWrapper>("calibration_kicker", kicker_robot_id_, world_model);
      kicker_command->setTargetPosition(getKickerTargetPosition());
      robot_commands.emplace_back(kicker_command->getMsg());

      auto retriever_command = std::make_shared<RobotCommandWrapper>(
        "calibration_retriever", retriever_robot_id_, world_model);
      retriever_command->setTargetPosition(getRetrieverWaitingPosition());
      robot_commands.emplace_back(retriever_command->getMsg());
    } break;

    case CollectorState::BALL_RETRIEVAL: {
      // 球拾いスキルの初期化
      if (!retriever_skill_) {
        auto retriever_command =
          std::make_shared<RobotCommandWrapper>("receive", retriever_robot_id_, world_model);
        retriever_skill_ = std::make_shared<skills::Receive>(retriever_command);
        retriever_skill_->setParameter("policy", std::string("closest"));
        retriever_skill_->setParameter("dribble_power", 0.3);
      }

      // 球拾いロボット
      retriever_skill_->run();
      robot_commands.emplace_back(retriever_skill_->getRobotCommand());

      // キッカーロボットはパス受け取り位置へ移動
      auto kicker_command =
        std::make_shared<RobotCommandWrapper>("calibration_kicker", kicker_robot_id_, world_model);
      kicker_command->setTargetPosition(getKickerTargetPosition());
      robot_commands.emplace_back(kicker_command->getMsg());

      // ボール取得完了チェック
      auto retriever_robot = world_model->getOurRobot(retriever_robot_id_);
      if (retriever_robot && (retriever_robot->pose.pos - world_model->ball().pos).norm() < 0.2) {
        current_state_ = CollectorState::RETURN_PASS;
        state_start_time_ = current_time;
        RCLCPP_INFO(node_.get_logger(), "ボール回収完了、返球開始");
      }
    } break;

    case CollectorState::RETURN_PASS: {
      // 球拾いロボットがキッカーロボットにパス
      auto retriever_command = std::make_shared<RobotCommandWrapper>(
        "calibration_retriever", retriever_robot_id_, world_model);

      auto kicker_robot = world_model->getOurRobot(kicker_robot_id_);
      if (kicker_robot) {
        retriever_command->setTargetPosition(kicker_robot->pose.pos, 0.3);  // 適度なパワー
        retriever_command->kickStraight(0.3);
      }
      robot_commands.emplace_back(retriever_command->getMsg());

      // キッカーロボットはパス受け取り準備
      auto kicker_command =
        std::make_shared<RobotCommandWrapper>("calibration_kicker", kicker_robot_id_, world_model);
      kicker_command->setTargetPosition(getKickerTargetPosition());
      robot_commands.emplace_back(kicker_command->getMsg());

      // パス完了チェック（時間ベース）
      double time_since_start = (current_time - state_start_time_).seconds();
      if (time_since_start > 3.0) {
        current_state_ = CollectorState::CYCLE_COMPLETE;
        state_start_time_ = current_time;
        RCLCPP_INFO(node_.get_logger(), "返球完了、サイクル終了");
      }
    } break;

    case CollectorState::CYCLE_COMPLETE: {
      current_cycle_++;

      if (current_cycle_ >= data_collection_cycles_) {
        RCLCPP_INFO(
          node_.get_logger(), "全サイクル完了 (%zu/%zu)", current_cycle_, data_collection_cycles_);
        return {PlannerBase::Status::SUCCESS, {}};
      }

      // 次のサイクルへ
      resetCycle();
      current_state_ = CollectorState::SETUP_POSITIONS;
      state_start_time_ = current_time;
      RCLCPP_INFO(
        node_.get_logger(), "次のサイクル開始 (%zu/%zu)", current_cycle_ + 1,
        data_collection_cycles_);

      return calculateRobotCommand(robots, context);  // 再帰呼び出し
    } break;
  }

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

  return selected;
}

std::string BallCalibrationDataCollectorPlanner::getStateString(CollectorState state) const
{
  switch (state) {
    case CollectorState::SETUP_POSITIONS:
      return "SETUP_POSITIONS";
    case CollectorState::KICK_PREPARATION:
      return "KICK_PREPARATION";
    case CollectorState::EXECUTING_KICK:
      return "EXECUTING_KICK";
    case CollectorState::WAITING_BALL_STOP:
      return "WAITING_BALL_STOP";
    case CollectorState::BALL_RETRIEVAL:
      return "BALL_RETRIEVAL";
    case CollectorState::RETURN_PASS:
      return "RETURN_PASS";
    case CollectorState::CYCLE_COMPLETE:
      return "CYCLE_COMPLETE";
    default:
      return "UNKNOWN";
  }
}

bool BallCalibrationDataCollectorPlanner::willBallStopOutsideField() const
{
  auto predicted_stop = world_model->ball().getPredictedPosition(10.0);  // 10秒後の位置

  // フィールド境界チェック（マージン考慮）
  auto field_size = world_model->fieldSize();
  double field_x_max = field_size.x() * 0.5 - field_boundary_margin_;
  double field_x_min = -field_size.x() * 0.5 + field_boundary_margin_;
  double field_y_max = field_size.y() * 0.5 - field_boundary_margin_;
  double field_y_min = -field_size.y() * 0.5 + field_boundary_margin_;

  return predicted_stop.x() > field_x_max || predicted_stop.x() < field_x_min ||
         predicted_stop.y() > field_y_max || predicted_stop.y() < field_y_min;
}

bool BallCalibrationDataCollectorPlanner::isBallFullyStopped() const
{
  return world_model->ball().vel.norm() < 0.05;  // 5cm/s以下で停止とみなす
}

Point BallCalibrationDataCollectorPlanner::getKickerTargetPosition() const
{
  // 自陣ゴール位置からx方向にオフセット
  auto field_size = world_model->fieldSize();
  Point our_goal_center(-field_size.x() * 0.5, 0.0);
  return Point(our_goal_center.x() + kicker_x_offset_, 0.0);
}

Point BallCalibrationDataCollectorPlanner::getRetrieverWaitingPosition() const
{
  // フィールド中央やや前方で待機
  return Point(1.0, 0.0);
}

double BallCalibrationDataCollectorPlanner::getNextKickPower()
{
  if (current_power_index_ >= kick_power_sequence_.size()) {
    current_power_index_ = 0;  // 循環
  }
  return kick_power_sequence_[current_power_index_++];
}

void BallCalibrationDataCollectorPlanner::resetCycle()
{
  kicker_skill_.reset();
  retriever_skill_.reset();
  kick_executed_ = false;
  ball_stop_position_ = Point(0, 0);
}

}  // namespace crane
