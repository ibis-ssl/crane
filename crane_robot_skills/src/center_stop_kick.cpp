// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_robot_skills/center_stop_kick.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crane::skills
{

void CenterStopKick::initialize()
{
  last_ball_motion_time_ = rclcpp::Clock().now();

  // ボール回避用状態の初期化
  has_started_positioning_ = false;
  last_ball_position_ = Point::Zero();
  kick_executed_ = false;
  kick_start_time_ = rclcpp::Clock().now();

  // リトライ制御の初期化
  retry_count_ = 0;
  result_check_start_ = rclcpp::Time(0);

  // 物理モデルの初期化
  initializePhysicsModels();

  addStateFunction(CenterStopKickState::ENTRY_POINT, [this]() -> Status {
    command->stopHere();
    last_ball_motion_time_ = rclcpp::Clock().now();

    // フィールド中心への停止距離を計算
    target_stop_distance_ = calculateTargetStopDistance();

    return Status::RUNNING;
  });

  addStateFunction(CenterStopKickState::WAIT_BALL_STOP, [this]() -> Status {
    command->stopHere();

    // 目標停止距離を再計算（ボール位置が変わった場合）
    target_stop_distance_ = calculateTargetStopDistance();

    return Status::RUNNING;
  });

  addStateFunction(CenterStopKickState::POSITION_BEHIND_BALL, [this]() -> Status {
    Point current_ball_pos = world_model()->ball().pos;

    // ボール位置変化検出（テレポート対応）
    if (has_started_positioning_) {
      double ball_position_change = (current_ball_pos - last_ball_position_).norm();
      const double teleport_threshold = 0.2;  // 0.2m以上の変化でテレポートと判定

      if (ball_position_change > teleport_threshold) {
        RCLCPP_WARN(
          rclcpp::get_logger("CenterStopKick"),
          "ボールテレポート検出: 位置変化 %.3fm、目標位置を再計算します", ball_position_change);

        // 位置取り状態をリセットして再計算を強制
        has_started_positioning_ = false;
        target_stop_distance_ = calculateTargetStopDistance();
      }
    }

    // 初回実行時または位置変化検出時：内部状態を初期化
    if (not has_started_positioning_) {
      has_started_positioning_ = true;
      last_ball_position_ = current_ball_pos;  // 現在のボール位置を記録
    }
    // 回り込みターゲット（base=max=approach_distance_で一定オフセット）
    Point approach_target = computeAroundBallApproachTargetDynamic(
      current_ball_pos, target_position_, robot()->pose.pos, approach_distance_,
      approach_distance_);

    command->setTargetPosition(approach_target)
      .lookAtFrom(target_position_, world_model()->ball().pos)
      .setOmegaLimit(10.0)
      .disableBallAvoidance()
      .disableGoalAreaAvoidance()
      .setMaxVelocity("CenterStopKickState::POSITION_BEHIND_BALL", 3.0);

    return Status::RUNNING;
  });

  addStateFunction(CenterStopKickState::KICK_EXECUTE, [this]() -> Status {
    // 目標停止距離を再計算
    double current_target_distance = calculateTargetStopDistance();

    // 必要なキック力を計算
    calculated_kick_power_ = calculateRequiredKickPower(current_target_distance);

    if (!kick_executed_) {
      kick_start_time_ = rclcpp::Clock().now();
      kick_executed_ = true;
    }

    command->setTargetPosition(world_model()->ball().pos)
      .lookAtBall()
      .setOmegaLimit(10.0)
      .kickStraight(calculated_kick_power_)
      .disableBallAvoidance()
      .disableGoalAreaAvoidance()
      .setMaxVelocity("CenterStopKickState::KICK_EXECUTE", 5.0);

    return Status::RUNNING;
  });

  addStateFunction(CenterStopKickState::KICK_COMPLETE, [this]() -> Status {
    command->stopHere();

    auto now = rclcpp::Clock().now();

    // 初回入室時の処理
    if (result_check_start_.seconds() == 0) {
      result_check_start_ = now;
    }

    // ボール停止確認（1秒待機）
    if ((now - result_check_start_).seconds() < 1.0) {
      visualizer->text()
        .position(robot()->pose.pos.x(), robot()->pose.pos.y() + 0.5)
        .text("結果確認中...")
        .fill("yellow")
        .fontSize(80)
        .build();
      return Status::RUNNING;
    }

    // 中心からの距離チェック
    double distance_to_center = world_model()->ball().pos.norm();

    if (distance_to_center <= center_tolerance_) {
      // 成功
      visualizer->text()
        .position(robot()->pose.pos.x(), robot()->pose.pos.y() + 0.5)
        .text("中心停止キック成功")
        .fill("green")
        .fontSize(80)
        .build();

      RCLCPP_INFO(
        rclcpp::get_logger("CenterStopKick"), "成功: 距離=%.3fm (許容=%.3fm), 試行回数=%d",
        distance_to_center, center_tolerance_, retry_count_ + 1);

      return Status::SUCCESS;
    } else if (retry_count_ < max_retry_count_) {
      // リトライ
      retry_count_++;

      // 状態リセット
      resetForRetry();
      return Status::RUNNING;
    } else {
      // リトライ上限到達
      visualizer->text()
        .position(robot()->pose.pos.x(), robot()->pose.pos.y() + 0.5)
        .text("リトライ上限到達")
        .fill("red")
        .fontSize(80)
        .build();

      RCLCPP_WARN(
        rclcpp::get_logger("CenterStopKick"), "リトライ上限到達: 最終距離=%.3fm",
        distance_to_center);

      return Status::SUCCESS;
    }
  });

  // ENTRY_POINT -> WAIT_BALL_STOP（自動遷移）
  addTransition(
    CenterStopKickState::ENTRY_POINT, CenterStopKickState::WAIT_BALL_STOP,
    [this]() -> bool { return true; });

  // WAIT_BALL_STOP -> POSITION_BEHIND_BALL（ボール停止確認）
  addTransition(
    CenterStopKickState::WAIT_BALL_STOP, CenterStopKickState::POSITION_BEHIND_BALL,
    [this]() -> bool {
      auto now = rclcpp::Clock().now();

      if (not world_model()->ball().isStopped(ball_stop_threshold_)) {
        last_ball_motion_time_ = now;
        return false;
      }

      bool should_transition = (now - last_ball_motion_time_).seconds() > stop_time_threshold_;
      // 状態遷移時にボール回避状態をリセット
      if (should_transition) {
        has_started_positioning_ = false;
        last_ball_position_ = Point::Zero();  // ボール位置記録もリセット
      }

      return should_transition;
    });

  // POSITION_BEHIND_BALL -> KICK_EXECUTE（位置・速度の条件のみ）
  addTransition(
    CenterStopKickState::POSITION_BEHIND_BALL, CenterStopKickState::KICK_EXECUTE, [this]() -> bool {
      auto kick_position = getKickPosition();

      bool position_ok = robot()->getDistance(kick_position) < position_tolerance_;
      bool velocity_ok = robot()->vel.linear.norm() < 0.1;  // ほぼ停止

      return position_ok && velocity_ok;
    });

  // KICK_EXECUTE -> KICK_COMPLETE（キック完了確認）
  addTransition(
    CenterStopKickState::KICK_EXECUTE, CenterStopKickState::KICK_COMPLETE,
    [this]() -> bool { return isKickCompleted(); });

  // KICK_COMPLETE -> WAIT_BALL_STOP（リトライ遷移）
  addTransition(
    CenterStopKickState::KICK_COMPLETE, CenterStopKickState::WAIT_BALL_STOP, [this]() -> bool {
      // result_check_start_が設定されている場合のみリトライ判定
      if (result_check_start_.seconds() == 0) {
        return false;
      }

      auto now = rclcpp::Clock().now();

      // 1秒未満は待機
      if ((now - result_check_start_).seconds() < 1.0) {
        return false;
      }

      // 距離チェック
      double distance_to_center = world_model()->ball().pos.norm();

      // 成功の場合はリトライしない
      if (distance_to_center <= center_tolerance_) {
        return false;
      }

      // リトライ可能な場合のみ遷移
      return retry_count_ < max_retry_count_;
    });
}

Point CenterStopKick::getKickPosition() const
{
  auto ball_pos = world_model()->ball().pos;
  return ball_pos - (target_position_ - ball_pos).normalized() * approach_distance_;
}

double CenterStopKick::calculateTargetStopDistance() const
{
  auto ball_pos = world_model()->ball().pos;
  return (target_position_ - ball_pos).norm();
}

double CenterStopKick::calculateRequiredKickPower(double target_distance)
{
  if (!kicker_model_) {
    RCLCPP_WARN(
      rclcpp::get_logger("CenterStopKick"),
      "KickerModelが初期化されていません。デフォルトキック力を使用します");
    return 0.5;
  }

  try {
    double required_power = kicker_model_->calculateKickPowerForStopDistance(target_distance);

    // キック力を0.0-1.0の範囲にクランプ
    required_power = std::clamp(required_power, 0.0, 1.0);

    RCLCPP_DEBUG(
      rclcpp::get_logger("CenterStopKick"), "キック力計算: 距離=%.3fm -> キック力=%.3f",
      target_distance, required_power);

    return required_power;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CenterStopKick"), "キック力計算エラー: %s。デフォルト値を使用します",
      e.what());
    return 0.5;
  }
}

void CenterStopKick::initializePhysicsModels()
{
  try {
    // BallPhysicsModelのデフォルトインスタンスを作成
    ball_physics_model_ = std::make_shared<BallPhysicsModel>(BallPhysicsModel::createDefault());

    // KickerModelを作成してBallPhysicsModelと統合
    kicker_model_ = std::make_shared<KickerModel>();
    kicker_model_->setBallPhysicsModel(ball_physics_model_);

    RCLCPP_INFO(rclcpp::get_logger("CenterStopKick"), "物理モデル初期化完了");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("CenterStopKick"), "物理モデル初期化エラー: %s", e.what());
  }
}

bool CenterStopKick::isKickCompleted() const
{
  if (!kick_executed_) {
    return false;
  }

  // キック後にボールが動き始めるまで少し待つ
  auto now = rclcpp::Clock().now();
  if ((now - kick_start_time_).seconds() < 0.2) {
    return false;
  }

  // ボールが動き始めたかチェック
  if (world_model()->ball().vel.norm() > ball_motion_velocity_threshold_) {
    // ボールが目標方向（フィールド中心）に向かっているかチェック
    Point ball_pos = world_model()->ball().pos;
    Point ball_vel_direction = world_model()->ball().vel.normalized();
    Point target_direction = (target_position_ - ball_pos).normalized();

    // 方向の一致度をチェック（cos(30度) ≈ 0.866）
    double direction_similarity = ball_vel_direction.dot(target_direction);

    if (direction_similarity > 0.866) {
      RCLCPP_INFO(
        rclcpp::get_logger("CenterStopKick"), "キック完了確認: ボール速度=%.3fm/s, 方向一致度=%.3f",
        world_model()->ball().vel.norm(), direction_similarity);
      return true;
    }
  }

  // タイムアウト（キック実行から3秒経過）
  if ((now - kick_start_time_).seconds() > 3.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("CenterStopKick"), "キック完了タイムアウト。キック完了と判定します");
    return true;
  }

  return false;
}

void CenterStopKick::resetForRetry()
{
  // 結果確認タイマーをリセット
  result_check_start_ = rclcpp::Time(0);

  // キック実行状態をリセット
  kick_executed_ = false;

  // ボール回避状態をリセット
  has_started_positioning_ = false;
  last_ball_position_ = Point::Zero();

  // タイムスタンプをリセット
  last_ball_motion_time_ = rclcpp::Clock().now();
  kick_start_time_ = rclcpp::Clock().now();

  RCLCPP_DEBUG(rclcpp::get_logger("CenterStopKick"), "リトライ用状態リセット完了");
}

}  // namespace crane::skills
