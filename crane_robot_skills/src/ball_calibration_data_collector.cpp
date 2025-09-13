// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/ball_calibration_data_collector.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crane::skills
{

void BallCalibrationDataCollector::initialize()
{
  last_ball_motion_time_ = rclcpp::Clock().now();

  // ボール回避用状態の初期化
  has_started_positioning_ = false;
  has_passed_intermediate_ = false;
  last_ball_position_ = Point::Zero();

  addStateFunction(BallCalibrationState::ENTRY_POINT, [this]() -> Status {
    command->stopHere();
    last_ball_motion_time_ = rclcpp::Clock().now();
    return Status::RUNNING;
  });

  addStateFunction(BallCalibrationState::WAIT_BALL_STOP, [this]() -> Status {
    command->stopHere();
    return Status::RUNNING;
  });

  addStateFunction(BallCalibrationState::POSITION_BEHIND_BALL, [this]() -> Status {
    Point current_ball_pos = world_model()->ball().pos;

    // ボール位置変化検出（テレポート対応）
    if (has_started_positioning_) {
      double ball_position_change = (current_ball_pos - last_ball_position_).norm();
      const double teleport_threshold = 0.2;  // 0.2m以上の変化でテレポートと判定

      if (ball_position_change > teleport_threshold) {
        RCLCPP_WARN(
          rclcpp::get_logger("BallCalibrationDataCollector"),
          "ボールテレポート検出: 位置変化 %.3fm、目標位置を再計算します", ball_position_change);

        // 位置取り状態をリセットして再計算を強制
        has_started_positioning_ = false;
        has_passed_intermediate_ = false;
      }
    }

    // 初回実行時または位置変化検出時：目標位置と中間経由点を計算
    if (not has_started_positioning_) {
      final_target_pos_ = getKickPosition();

      // ボール中心から最終目標への方向ベクトル
      Vector2 direction_to_target = (final_target_pos_ - current_ball_pos).normalized();
      Vector2 margin_vec = direction_to_target * ball_avoidance_margin_;

      // ボール回避のための中間経由点を計算（ボールを中心とした垂直方向）
      auto vertical_vec = getVerticalVec(margin_vec);
      intermediate_pos_1_ = current_ball_pos + vertical_vec;
      intermediate_pos_2_ = current_ball_pos - vertical_vec;

      has_started_positioning_ = true;
      last_ball_position_ = current_ball_pos;  // 現在のボール位置を記録

      RCLCPP_INFO(
        rclcpp::get_logger("BallCalibrationDataCollector"),
        "位置取り目標設定: ボール(%.3f,%.3f) -> 最終目標(%.3f,%.3f)", current_ball_pos.x(),
        current_ball_pos.y(), final_target_pos_.x(), final_target_pos_.y());
    }

    // ロボットの現在位置から各地点への距離を計算
    double final_distance = (robot()->pose.pos - final_target_pos_).norm();
    double intermediate_distance_1 = (robot()->pose.pos - intermediate_pos_1_).norm();
    double intermediate_distance_2 = (robot()->pose.pos - intermediate_pos_2_).norm();

    // より近い中間経由点を選択
    Point selected_intermediate = (intermediate_distance_1 < intermediate_distance_2)
                                    ? intermediate_pos_1_
                                    : intermediate_pos_2_;
    double selected_intermediate_distance =
      std::min(intermediate_distance_1, intermediate_distance_2);

    // 経路選択：中間経由点経由 vs 直接最終目標
    Point target_position;
    if (selected_intermediate_distance < final_distance && !has_passed_intermediate_) {
      // 中間経由点に向かう
      target_position = selected_intermediate;

      if (selected_intermediate_distance < intermediate_reach_threshold_) {
        has_passed_intermediate_ = true;
      }
    } else {
      // 最終目標に向かう
      target_position = final_target_pos_;
    }

    command->setTargetPosition(target_position)
      .lookAtFrom(kick_target_, world_model()->ball().pos)
      .setOmegaLimit(10.0)
      .disableBallAvoidance()
      .disableGoalAreaAvoidance()
      .setMaxVelocity("BallCalibrationState::POSITION_BEHIND_BALL", 3.0);

    return Status::RUNNING;
  });

  addStateFunction(BallCalibrationState::KICK_EXECUTE, [this]() -> Status {
    command->setTargetPosition(world_model()->ball().pos)
      .lookAtBall()
      .setOmegaLimit(10.0)
      .kickStraight(getCurrentKickPower())
      .disableBallAvoidance()
      .disableGoalAreaAvoidance()
      .setMaxVelocity("BallCalibrationState::KICK_EXECUTE", 5.0);

    return Status::RUNNING;
  });

  // ENTRY_POINT -> WAIT_BALL_STOP（自動遷移）
  addTransition(
    BallCalibrationState::ENTRY_POINT, BallCalibrationState::WAIT_BALL_STOP,
    [this]() -> bool { return true; });

  // WAIT_BALL_STOP -> POSITION_BEHIND_BALL（ボール停止確認）
  addTransition(
    BallCalibrationState::WAIT_BALL_STOP, BallCalibrationState::POSITION_BEHIND_BALL,
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
        has_passed_intermediate_ = false;
        last_ball_position_ = Point::Zero();  // ボール位置記録もリセット
      }

      return should_transition;
    });

  // POSITION_BEHIND_BALL -> KICK_EXECUTE（位置・速度の条件のみ）
  addTransition(
    BallCalibrationState::POSITION_BEHIND_BALL, BallCalibrationState::KICK_EXECUTE,
    [this]() -> bool {
      auto kick_position = getKickPosition();

      bool position_ok = robot()->getDistance(kick_position) < position_tolerance_;
      bool velocity_ok = robot()->vel.linear.norm() < 0.1;  // ほぼ停止

      return position_ok && velocity_ok;
    });

  // KICK_EXECUTE -> WAIT_BALL_STOP（キック完了・次サイクル）
  addTransition(
    BallCalibrationState::KICK_EXECUTE, BallCalibrationState::WAIT_BALL_STOP, [this]() -> bool {
      if (world_model()->ball().vel.norm() > ball_motion_velocity_threshold_) {
        // キック完了、次のパワーインデックスに進む
        advanceKickPowerIndex();
        last_ball_motion_time_ = rclcpp::Clock().now();

        RCLCPP_INFO(
          rclcpp::get_logger("BallCalibrationDataCollector"),
          "キック完了: パワー=%.2f, 次インデックス=%d", getCurrentKickPower(),
          current_power_index_);

        return true;
      }
      return false;
    });
}

Point BallCalibrationDataCollector::getKickPosition() const
{
  auto ball_pos = world_model()->ball().pos;
  return ball_pos - (kick_target_ - ball_pos).normalized() * approach_distance_;
}

double BallCalibrationDataCollector::getCurrentKickPower() const
{
  if (
    current_power_index_ >= 0 &&
    static_cast<size_t>(current_power_index_) < kick_power_sequence_.size()) {
    return kick_power_sequence_[static_cast<size_t>(current_power_index_)];
  } else {
    return 0.5;
  }
}

void BallCalibrationDataCollector::advanceKickPowerIndex()
{
  current_power_index_ = (current_power_index_ + 1) % static_cast<int>(kick_power_sequence_.size());
}
}  // namespace crane::skills
