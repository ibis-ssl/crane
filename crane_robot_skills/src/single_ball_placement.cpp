// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/geometry/geometries/concepts/point_concept.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <magic_enum/magic_enum.hpp>
#include <memory>

namespace crane::skills
{
std::string SingleBallPlacement::getStateName(int s)
{
  return std::string(magic_enum::enum_name(static_cast<SingleBallPlacementStates>(s)));
}

void SingleBallPlacement::initialize()
{
  setParameter("placement_x", 0.);
  setParameter("placement_y", 0.);
  setParameter("pass_enable", false);

  // マイナスするとコート内も判定される
  setParameter("コート端判定のオフセット", 0.0);

  addStateFunction(static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), [this]() {
    command->stopHere();
    return Status::RUNNING;
  });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT),
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), [this]() {
      pull_back_target = std::nullopt;
      contact_count_ = 0;
      return false;
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT),
    static_cast<int>(SingleBallPlacementStates::RECEIVE_BALL), [this]() {
      if (not getParameter<bool>("pass_enable")) {
        return false;
      }
      // ボールが自分に向かって動いているとき
      return world_model()->ball().isMoving(1.0) &&
             world_model()->ball().isMovingTowards(robot()->pose.pos);
    });

  addStateFunction(static_cast<int>(SingleBallPlacementStates::RECEIVE_BALL), [this]() {
    if (not receive) {
      receive = std::make_shared<skills::Receive>(command);
      receive->setParameter("enable_software_bumper", false);
      receive->setParameter("policy", std::string("closest"));
      receive->setParameter("enable_active_receive", true);
    }

    receive->run();
    return Status::RUNNING;
  });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::RECEIVE_BALL),
    static_cast<int>(SingleBallPlacementStates::SLEEP), [this]() {
      // 近くで停止したらSLEEP後にゆっくり離れる
      if (
        world_model()->ball().isStopped(0.1) &&
        robot()->getDistance(world_model()->ball().pos) < 0.2) {
        if (sleep) {
          sleep.reset();
        }
        return true;
      } else {
        return false;
      }
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT),
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE), [this]() {
      auto placement_target = world_model()->getBallPlacementTarget();
      if (
        placement_target &&
        bg::distance(world_model()->ball().pos, placement_target.value()) > 0.1) {
        command->setOmegaLimit(1.0);
        return true;
      } else {
        // 動かす必要がなければそのまま
        return false;
      }
    });

  // 端にある場合、コート側からアプローチする
  addStateFunction(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE), [this]() {
      pull_back_target = world_model()->ball().pos;
      const auto offset = getParameter<double>("コート端判定のオフセット");
      const auto threshold_x = world_model()->fieldSize().x() * 0.5 + offset;
      const auto threshold_y = world_model()->fieldSize().y() * 0.5 + offset;
      if (std::abs(pull_back_target->x()) > threshold_x) {
        pull_back_target->x() = std::copysign(threshold_x - 0.2, pull_back_target->x());
      }
      if (std::abs(pull_back_target->y()) > threshold_y) {
        pull_back_target->y() = std::copysign(threshold_y - 0.2, pull_back_target->y());
      }

      command->setTargetPosition(pull_back_target.value());
      command->lookAtBallFrom(pull_back_target.value());
      command->disableAnyAreaAvoidance();
      double max_vel = std::min(1.5, robot()->getDistance(pull_back_target.value()) + 0.1);
      command->setMaxVelocity("SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE", max_vel);
      return Status::RUNNING;
    });

  // 必要ない場合はコート端処理をスキップ
  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE),
    static_cast<int>(SingleBallPlacementStates::GO_OVER_BALL), [this]() {
      return world_model()->point_checker.isFieldInside(
        world_model()->ball().pos, getParameter<double>("コート端判定のオフセット") - 0.05);
    });

  // pull_back_targetに到達したら次のステートへ
  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE),
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH), [this]() {
      if (not pull_back_target) {
        return false;
      } else {
        skill_status = Status::RUNNING;
        return robot()->getDistance(pull_back_target.value()) < 0.05;
      }
    });

  // PULL_BACK_FROM_EDGE_TOUCH
  addStateFunction(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH), [this]() {
      command->disableAnyAreaAvoidance();
      command->setTargetPosition(world_model()->ball().pos);
      command->setMaxVelocity("SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH", 0.5);

      // 引っ張る
      command->dribble(0.5);

      return skill_status;
    });

  // ボールを逃したらやり直し
  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH),
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), [this]() {
      using boost::math::constants::degree;
      return getAngleDiff(
               robot()->pose.theta, getAngle(world_model()->ball().pos - robot()->pose.pos)) >
             20. * degree<double>();
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH),
    static_cast<int>(SingleBallPlacementStates::GO_OVER_BALL), [this]() {
      return world_model()->point_checker.isFieldInside(
        world_model()->ball().pos, getParameter<double>("コート端判定のオフセット"));
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH),
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL), [this]() {
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();

      // ボールセンサーが利用可能な場合はその反応も確認する
      if (robot()->getBallSensorAvailable(now, rclcpp::Duration::from_seconds(0.1))) {
        // センサーが反応していて、かつ500ms以上接触している
        return robot()->ball_sensor &&
               robot()->ball_contact.getContactDuration().count() / 1e6 > 500;
      } else {
        // センサーが利用不可の場合は接触時間のみで判定（フォールバック）
        return robot()->ball_contact.getContactDuration().count() / 1e6 > 500;
      }
    });

  // 失敗の場合は最初に戻る
  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH),
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE),
    [this]() { return skill_status == Status::FAILURE; });

  // PULL_BACK_FROM_EDGE_PULL
  addStateFunction(static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL), [this]() {
    command->setDribblerTargetPosition(pull_back_target.value());
    // 角度はそのまま引っ張りたいので指定はしない
    command->dribble(0.6);
    command->setMaxVelocity("SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL", 0.15);
    command->disableAnyAreaAvoidance();
    return Status::RUNNING;
  });

  // pull_back_targetに到着したら一旦離れる（Visionからボールが見える状況にする）
  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL),
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_OVER_SLEEP), [this]() {
      if ((robot()->kicker_center() - pull_back_target.value()).norm() < 0.03) {
        if (sleep) {
          sleep.reset();
        }
        return true;
      } else {
        return false;
      }
    });

  addStateFunction(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_OVER_SLEEP), [this]() {
      if (not sleep) {
        sleep = std::make_shared<Sleep>(command);
        sleep->setParameter("duration", 2.0);
      }
      skill_status = sleep->run();
      command->stopHere();
      command->disableAnyAreaAvoidance();
      command->setOmegaLimit(0.0);
      if (robot()->vel.linear.norm() < 0.05 && world_model()->ball().isStopped(0.05)) {
        command->dribble(0.0);
      } else {
        command->dribble(0.3);
      }
      return Status::RUNNING;
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_OVER_SLEEP),
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_OVER_LEAVE), [this]() {
      pull_back_angle = robot()->pose.theta;
      // sleepが終わったら成功
      return skill_status == Status::SUCCESS;
    });

  addStateFunction(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_OVER_LEAVE), [this]() {
      // メモ：().normalized() * 0.8したらなぜかゼロベクトルが出来上がってしまう
      Vector2 diff = (robot()->pose.pos - pull_back_target.value());
      diff.normalize();
      diff = diff * 0.8;
      auto leave_pos = pull_back_target.value() + diff;

      command->setTargetTheta(pull_back_angle);
      command->setTargetPosition(leave_pos);
      command->setOmegaLimit(0.0);
      command->setMaxVelocity("SingleBallPlacementStates::PULL_BACK_FROM_EDGE_OVER_LEAVE", 1.0);
      command->disableAnyAreaAvoidance();
      return skill_status;
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_OVER_LEAVE),
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), [this]() {
      // pull_back_targetからはなれたらENTRY_POINTへ移動
      return robot()->getDistance(pull_back_target.value()) > 0.3;
    });

  // ボールが離れたら始めに戻る
  // 2025/04/12 ボールが見えなくなったときに悪影響があるので一旦解除
  //  addTransition(
  //    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL,
  //    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
  //    [this]() { return robot()->getDistance(world_model()->ball().pos) > 0.15; });

  addStateFunction(static_cast<int>(SingleBallPlacementStates::GO_OVER_BALL), [this]() {
    command->setMaxVelocity("SingleBallPlacementStates::GO_OVER_BALL", 1.5);
    auto placement_target = getPlacementTarget();
    const auto & ball_pos = world_model()->ball().pos;

    constexpr double INTERVAL = 0.15;
    constexpr double MAX_INTERVAL = 0.6;  // 初期は大きめ、上限あり
    Point approach = computeAroundBallApproachTargetDynamic(
      ball_pos, placement_target, robot()->pose.pos, INTERVAL, MAX_INTERVAL);

    // フィールド境界内にクランプ（壁際での回り込みスタックを防ぐ）
    constexpr double field_margin = 0.05;  // 5cmマージン
    const double max_x = world_model()->fieldSize().x() * 0.5 - field_margin;
    const double max_y = world_model()->fieldSize().y() * 0.5 - field_margin;
    approach.x() = std::clamp(approach.x(), -max_x, max_x);
    approach.y() = std::clamp(approach.y(), -max_y, max_y);

    command->setTargetPosition(approach, 0.0);
    command->lookAtFrom(placement_target, ball_pos);
    command->disablePlacementAvoidance();
    command->disableGoalAreaAvoidance();
    command->dribble(0.0);
    command->setOmegaLimit(10.0);

    if (command->getTargetDistance() < 0.02) {
      skill_status = Status::SUCCESS;
    } else {
      skill_status = Status::RUNNING;
    }
    return Status::RUNNING;
  });

  // GO_OVER_BALL中にボールが壁際に移動した場合、壁際処理に戻る
  addTransition(
    static_cast<int>(SingleBallPlacementStates::GO_OVER_BALL),
    static_cast<int>(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE), [this]() {
      // ボールがフィールド境界外に出た場合、壁際処理に戻る
      return !world_model()->point_checker.isFieldInside(
        world_model()->ball().pos, getParameter<double>("コート端判定のオフセット"));
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::GO_OVER_BALL),
    static_cast<int>(SingleBallPlacementStates::PASS_TO_TARGET), [this]() {
      if (not getParameter<bool>("pass_enable")) {
        return false;
      }
      auto placement_target = getPlacementTarget();
      return (placement_target - world_model()->ball().pos).norm() > 2.0;
    });

  addStateFunction(static_cast<int>(SingleBallPlacementStates::PASS_TO_TARGET), [this]() {
    command->disablePlacementAvoidance();
    command->disableBallAvoidance();
    command->setMaxVelocity("SingleBallPlacementStates::PASS_TO_TARGET", 0.2);
    command->setMaxAcceleration("SingleBallPlacementStates::PASS_TO_TARGET", 1.0);
    auto placement_target = getPlacementTarget();
    command->lookAtFrom(placement_target, robot()->pose.pos);
    command->setTargetPosition(
      world_model()->ball().pos +
      (placement_target - world_model()->ball().pos).normalized() * 0.3);
    command->kickStraight(0.3);

    return Status::RUNNING;
  });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::GO_OVER_BALL),
    static_cast<int>(SingleBallPlacementStates::CONTACT_BALL),
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(static_cast<int>(SingleBallPlacementStates::CONTACT_BALL), [this]() {
    command->disablePlacementAvoidance();
    command->disableBallAvoidance();
    command->setMaxVelocity("SingleBallPlacementStates::CONTACT_BALL", 0.2);
    command->setMaxAcceleration("SingleBallPlacementStates::CONTACT_BALL", 1.0);
    auto placement_target = getPlacementTarget();
    command->lookAtFrom(placement_target, world_model()->ball().pos);
    command->setTargetPosition(world_model()->ball().pos);

    return Status::RUNNING;
  });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::CONTACT_BALL),
    static_cast<int>(SingleBallPlacementStates::MOVE_TO_TARGET), [this]() {
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();

      // ボールセンサーが利用可能な場合は必ずセンサー反応を待つ
      if (robot()->getBallSensorAvailable(now, rclcpp::Duration::from_seconds(0.1))) {
        // センサーが反応している場合のみカウント
        if (robot()->ball_sensor) {
          if (++contact_count_ > 2) {
            contact_count_ = 0;
            return true;
          } else {
            return false;
          }
        } else {
          // センサーは利用可能だが反応していない → まだ接触していない
          contact_count_ = 0;
          return false;
        }
      } else {
        // センサーが利用不可の場合のみ距離で判定（フォールバック）
        contact_count_ = 0;
        return robot()->getDistance(world_model()->ball().pos) < 0.15;
      }
    });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::CONTACT_BALL),
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), [this]() {
      // ロボットの向きがボールの方を向いていなかったらやり直し
      using boost::math::constants::degree;
      return std::abs(getAngleDiff(
               getAngle(world_model()->ball().pos - robot()->pose.pos), robot()->pose.theta)) >
             45 * degree<double>();
    });

  addStateFunction(static_cast<int>(SingleBallPlacementStates::MOVE_TO_TARGET), [this]() {
    auto placement_target = getPlacementTarget();

    [[maybe_unused]] Point ball_pos = [&]() -> Point {
      if (robot()->ball_sensor) {
        return robot()->pose.pos + getNormVec(robot()->pose.theta) * 0.09;
      } else {
        return world_model()->ball().pos;
      }
    }();

    // 目標までの距離に応じた速度制限を計算（元の速度モードロジックを再利用）
    double dist = (placement_target - robot()->pose.pos).norm();
    double acc = 0.5;
    double max_vel = std::min({std::sqrt(2. * dist * acc), 1.0, robot()->vel.linear.norm() + 0.1});
    command->setTargetPosition(placement_target);
    command->lookAt(placement_target);
    command->disableAnyAreaAvoidance();
    command->setMaxVelocity("SingleBallPlacementStates::MOVE_TO_TARGET", max_vel);
    command->setMaxAcceleration("SingleBallPlacementStates::MOVE_TO_TARGET", 1.0);
    command->setOmegaLimit(0.3);
    // 開始時にボールに接していることが前提にある
    if ((world_model()->ball().pos - placement_target).norm() < 0.10) {
      // 到着したら成功 ( ルールでは15cm以内だがマージンとして10cm以内に配置 )
      return skill_status = Status::SUCCESS;
    } else if (dist < 0.2 && robot()->ball_sensor == true) {
      // ・ビジョンから消えている & 目標位置にいる & ボールセンサーが反応している
      // 離れてみて成功しているか確認する
      return skill_status = Status::SUCCESS;
    } else {
      command->dribble(0.2);
      return skill_status = Status::RUNNING;
    }
  });

  // addTransition(
  //   SingleBallPlacementStates::MOVE_TO_TARGET, SingleBallPlacementStates::ENTRY_POINT, [this]() {
  //     // ロボットの向きがボールの方を向いていなかったらやり直し
  //     using boost::math::constants::degree;
  //     return std::abs(getAngleDiff(
  //              getAngle(world_model()->ball().pos - robot()->pose.pos), robot()->pose.theta)) >
  //            20 * degree<double>();
  //   });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::MOVE_TO_TARGET),
    static_cast<int>(SingleBallPlacementStates::SLEEP), [this]() {
      if (sleep) {
        sleep.reset();
      }
      auto placement_target = getPlacementTarget();
      if (skill_status == Status::SUCCESS) {
        return true;
      } else if (
        robot()->ball_sensor == true && world_model()->getMsg().ball_info.detected == false &&
        (placement_target - robot()->kicker_center()).norm() < 0.15) {
        // ボールセンサが動いているのにボールが見えない場合でロボットが配置位置にいる場合は成功
        return true;
      } else {
        return false;
      }
    });
  addTransition(
    static_cast<int>(SingleBallPlacementStates::MOVE_TO_TARGET),
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), [this]() {
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();
      bool flag = robot()->ball_sensor_stamp.has_value() &&
                  now.get_clock_type() == robot()->ball_sensor_stamp->get_clock_type() &&
                  (now - *(robot()->ball_sensor_stamp)).seconds() >= 1.0;

      if (flag && !robot()->ball_sensor) {
        RCLCPP_INFO(
          rclcpp::get_logger("SingleBallPlacement"),
          "Ball sensor is not working, so return to ENTRY_POINT: %fs",
          std::abs((now - *(robot()->ball_sensor_stamp)).seconds()));
        return true;
      } else {
        return false;
      }
    });
  // ボールが離れたら始めに戻る
  // addTransition(
  //   SingleBallPlacementStates::MOVE_TO_TARGET,
  //   SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
  //   [this]() { return skill_status == Status::FAILURE; });

  addStateFunction(static_cast<int>(SingleBallPlacementStates::SLEEP), [this]() {
    if (not sleep) {
      sleep = std::make_shared<Sleep>(command);
      sleep->setParameter("duration", 2.0);
    }
    skill_status = sleep->run();
    command->stopHere();
    command->disableAnyAreaAvoidance();
    command->setOmegaLimit(0.0);
    command->dribble(0.0);
    return Status::RUNNING;
  });

  // addTransition(
  //   SingleBallPlacementStates::SLEEP, SingleBallPlacementStates::ENTRY_POINT, [this]() {
  //   Point placement_target;
  //   placement_target << getParameter<double>("placement_x"), getParameter<double>("placement_y");
  //   // ルール 5.2 0.15m以内で認められる。再配置が必要場合のみ、 ENTRY_POINTへ移動
  //   return (world_model()->ball().pos - placement_target).norm() > 0.15;
  // });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::SLEEP),
    static_cast<int>(SingleBallPlacementStates::LEAVE_BALL), [this]() {
      pull_back_angle = robot()->pose.theta;
      return skill_status == Status::SUCCESS;
    });

  addStateFunction(static_cast<int>(SingleBallPlacementStates::LEAVE_BALL), [this]() {
    // メモ：().normalized() * 0.8したらなぜかゼロベクトルが出来上がってしまう
    // ボール判定15cm + ロボット判定50cm + ロボット半径10cm + マージン5com = 80cm
    Vector2 diff = -getNormVec(robot()->pose.theta) * 0.8;
    Point leave_pos = robot()->pose.pos + diff;
    auto placement_target = getPlacementTarget();

    if ((robot()->pose.pos - placement_target).norm() > 0.8) {
      // 離れたならば、その場に留まる
      leave_pos = robot()->pose.pos;
    }
    command->setTargetTheta(pull_back_angle);
    command->setTargetPosition(leave_pos);
    command->setOmegaLimit(0.0);
    command->setMaxVelocity("SingleBallPlacementStates::MOVE_TO_TARGET", 1.0);
    command->disableAnyAreaAvoidance();
    command->dribble(0.0);
    return skill_status;
  });

  addTransition(
    static_cast<int>(SingleBallPlacementStates::LEAVE_BALL),
    static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), [this]() {
      auto placement_target = getPlacementTarget();
      // ルール 5.2 0.15m以内で認められる。再配置が必要場合のみ、 ENTRY_POINTへ移動
      // return (world_model()->ball().pos - placement_target).norm() > 0.15;
      return ((world_model()->ball().pos - placement_target).norm() > 0.15) &&
             world_model()->getMsg().ball_info.detected;
    });
}
}  // namespace crane::skills
