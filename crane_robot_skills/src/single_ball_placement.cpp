// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/single_ball_placement.hpp>

namespace crane::skills
{
void SingleBallPlacement::initialize()
{
  setParameter("placement_x", 0.);
  setParameter("placement_y", 0.);

  // マイナスするとコート内も判定される
  setParameter("コート端判定のオフセット", 0.0);

  addStateFunction(SingleBallPlacementStates::ENTRY_POINT, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();
    command->stopHere();
    return Status::RUNNING;
  });

  addTransition(
    SingleBallPlacementStates::ENTRY_POINT, SingleBallPlacementStates::ENTRY_POINT, [this]() {
      pull_back_target = std::nullopt;
      return false;
    });

  addTransition(
    SingleBallPlacementStates::ENTRY_POINT, SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() {
      auto placement_target = world_model()->getBallPlacementTarget();
      if (
        placement_target && bg::distance(world_model()->ball.pos, placement_target.value()) > 0.1) {
        command->setOmegaLimit(1.0);
        return true;
      } else {
        // 動かす必要がなければそのまま
        return false;
      }
    });

  // 端にある場合、コート側からアプローチする
  addStateFunction(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();

    pull_back_target = world_model()->ball.pos;
    const auto offset = getParameter<double>("コート端判定のオフセット");
    const auto threshold_x = world_model()->field_size.x() * 0.5 + offset;
    const auto threshold_y = world_model()->field_size.y() * 0.5 + offset;
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
    command->setMaxVelocity(max_vel);
    return Status::RUNNING;
  });

  // 必要ない場合はコート端処理をスキップ
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE, SingleBallPlacementStates::GO_OVER_BALL,
    [this]() {
      return world_model()->point_checker.isFieldInside(
        world_model()->ball.pos, getParameter<double>("コート端判定のオフセット") - 0.05);
    });

  // pull_back_targetに到達したら次のステートへ
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH, [this]() {
      if (not pull_back_target) {
        return false;
      } else {
        skill_status = Status::RUNNING;
        return robot()->getDistance(pull_back_target.value()) < 0.05;
      }
    });

  // PULL_BACK_FROM_EDGE_TOUCH
  addStateFunction(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();
    command->disableAnyAreaAvoidance();
    command->setTargetPosition(world_model()->ball.pos);
    command->setMaxVelocity(0.5);

    const auto & ball_pos = world_model()->ball.pos;
    const Vector2 field = world_model()->field_size * 0.5;
    // 引っ張る
    command->dribble(0.5);

    return skill_status;
  });

  // ボールを逃したらやり直し
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH, SingleBallPlacementStates::ENTRY_POINT,
    [this]() {
      using boost::math::constants::degree;
      return getAngleDiff(
               robot()->pose.theta, getAngle(world_model()->ball.pos - robot()->pose.pos)) >
             20. * degree<double>();
    });

  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH, SingleBallPlacementStates::GO_OVER_BALL,
    [this]() {
      return world_model()->point_checker.isFieldInside(
        world_model()->ball.pos, getParameter<double>("コート端判定のオフセット"));
    });

  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL, [this]() {
      const auto & ball_pos = world_model()->ball.pos;
      const Vector2 field = world_model()->field_size * 0.5;
      // 500ms以上ボールに触れたらバック
      return robot()->ball_contact.getContactDuration().count() / 1e6 > 500;
    });

  // 失敗の場合は最初に戻る
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() { return skill_status == Status::FAILURE; });

  // PULL_BACK_FROM_EDGE_PULL
  addStateFunction(SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();
    command->setDribblerTargetPosition(pull_back_target.value());
    // 角度はそのまま引っ張りたいので指定はしない
    command->dribble(0.6);
    command->setMaxVelocity(0.15);
    command->disableAnyAreaAvoidance();
    return Status::RUNNING;
  });

  // pull_back_targetに到着したら始めに戻る（GO_OVER_BALLに転送される）
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() { return (robot()->kicker_center() - pull_back_target.value()).norm() < 0.03; });

  // ボールが離れたら始めに戻る
  // 2025/04/12 ボールが見えなくなったときに悪影響があるので一旦解除
  //  addTransition(
  //    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL,
  //    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
  //    [this]() { return robot()->getDistance(world_model()->ball.pos) > 0.15; });

  addStateFunction(SingleBallPlacementStates::GO_OVER_BALL, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();
    command->usePositionMode();
    command->setMaxVelocity(1.5);
    Point placement_target;
    placement_target << getParameter<double>("placement_x"), getParameter<double>("placement_y");
    const auto & ball_pos = world_model()->ball.pos;
    Point target = ball_pos + (ball_pos - placement_target).normalized() * 0.2;
    // ボールを避けて回り込む
    if (
      ((robot()->pose.pos - ball_pos).normalized())
        .dot((placement_target - ball_pos).normalized()) > 0.1) {
      Point around_point = [&]() {
        Vector2 vertical_vec = getVerticalVec((target - ball_pos).normalized()) * 0.3;
        Point around_point1 = ball_pos + vertical_vec;
        Point around_point2 = ball_pos - vertical_vec;
        if (robot()->getDistance(around_point1) < robot()->getDistance(around_point2)) {
          return around_point1;
        } else {
          return around_point2;
        }
      }();
      command->setTargetPosition(around_point);
    } else {
      command->setTargetPosition(target);
    }
    if (robot()->getDistance(world_model()->ball.pos) < 0.2) {
      // ロボットがボールに近い場合は一度引きの動作を入れる
      // これは端からのPULLが終わった後の誤作動を防ぐための動きである
      target << 0, 0;
    }
    command->lookAtBall();
    command->disablePlacementAvoidance();
    command->disableGoalAreaAvoidance();
    command->enableBallAvoidance();
    command->dribble(0.0);
    command->setOmegaLimit(10.0);

    if (robot()->getDistance(target) < 0.02) {
      skill_status = Status::SUCCESS;
    } else {
      skill_status = Status::RUNNING;
    }
    return Status::RUNNING;
  });

  addTransition(
    SingleBallPlacementStates::GO_OVER_BALL, SingleBallPlacementStates::CONTACT_BALL,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(SingleBallPlacementStates::CONTACT_BALL, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();
    command->usePositionMode();
    command->disablePlacementAvoidance();
    command->disableBallAvoidance();
    command->setMaxVelocity(0.2);
    command->setMaxAcceleration(1.0);
    Point placement_target;
    placement_target << getParameter<double>("placement_x"), getParameter<double>("placement_y");
    command->lookAtFrom(placement_target, world_model()->ball.pos);
    command->setTargetPosition(world_model()->ball.pos);

    return Status::RUNNING;
  });

  addTransition(
    SingleBallPlacementStates::CONTACT_BALL, SingleBallPlacementStates::MOVE_TO_TARGET, [this]() {
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();
      static int count = 0;
      if (now.get_clock_type() == robot()->ball_sensor_stamp.get_clock_type()) {
        if (std::abs((now - robot()->ball_sensor_stamp).seconds()) < 0.01 && robot()->ball_sensor) {
          if (++count > 2) {
            count = 0;
            return true;
          } else {
            return false;
          }
        } else {
          count = 0;
          return false;
        }
      } else {
        // ボールセンサが動いていないとき
        return robot()->getDistance(world_model()->ball.pos) < 0.15;
      }
    });

  addTransition(
    SingleBallPlacementStates::CONTACT_BALL, SingleBallPlacementStates::ENTRY_POINT, [this]() {
      // ロボットの向きがボールの方を向いていなかったらやり直し
      using boost::math::constants::degree;
      return std::abs(getAngleDiff(
               getAngle(world_model()->ball.pos - robot()->pose.pos), robot()->pose.theta)) >
             20 * degree<double>();
    });

  addStateFunction(SingleBallPlacementStates::MOVE_TO_TARGET, [this]() {
    Point placement_target;
    placement_target << getParameter<double>("placement_x"), getParameter<double>("placement_y");

    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();

    double vel_norm = [&]() {
      double dist = (placement_target - robot()->pose.pos).norm();
      double acc = 0.5;
      return std::min({std::sqrt(2. * dist * acc), 1.0, robot()->vel.linear.norm() + 0.1});
    }();
    Velocity vel = (placement_target - robot()->pose.pos).normalized() * vel_norm +
                   0.5 *
                     getVerticalVec(placement_target - world_model()->ball.pos)
                       .normalized()
                       .dot((world_model()->ball.pos - robot()->pose.pos).normalized()) *
                     getVerticalVec(placement_target - world_model()->ball.pos).normalized();
    command->usePolarVelocityMode();
    command->setVelocity(vel);
    command->lookAt(placement_target);
    command->disableAnyAreaAvoidance();
    command->setMaxVelocity(1.0);
    command->setMaxAcceleration(1.0);
    command->setOmegaLimit(1.0);
    // 開始時にボールに接していることが前提にある
    if (
      not robot()->ball_contact.findPastContact(1.0) or
      robot()->getDistance(world_model()->ball.pos) > 0.4) {
      // 1秒以上ボールが離れたら失敗
      return skill_status = Status::FAILURE;
    } else if (world_model()->getDistanceFromBall(placement_target) < 0.10) {
      // 到着したら成功 ( ルールでは15cm以内だがマージンとして10cm以内に配置 )
      return skill_status = Status::SUCCESS;
    } else {
      command->dribble(0.5);
      return skill_status = Status::RUNNING;
    }
  });

  addTransition(
    SingleBallPlacementStates::MOVE_TO_TARGET, SingleBallPlacementStates::ENTRY_POINT, [this]() {
      // ロボットの向きがボールの方を向いていなかったらやり直し
      using boost::math::constants::degree;
      return std::abs(getAngleDiff(
               getAngle(world_model()->ball.pos - robot()->pose.pos), robot()->pose.theta)) >
             20 * degree<double>();
    });

  addTransition(
    SingleBallPlacementStates::MOVE_TO_TARGET, SingleBallPlacementStates::SLEEP, [this]() {
      if (sleep) {
        sleep.reset();
      }

      return skill_status == Status::SUCCESS;
    });

  // ボールが離れたら始めに戻る
  addTransition(
    SingleBallPlacementStates::MOVE_TO_TARGET,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() { return skill_status == Status::FAILURE; });

  addStateFunction(SingleBallPlacementStates::SLEEP, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();
    if (not sleep) {
      sleep = std::make_shared<Sleep>(command);
      sleep->setParameter("duration", 2.0);
    }
    skill_status = sleep->run();
    command->usePositionMode();
    command->stopHere();
    command->disableAnyAreaAvoidance();
    command->setOmegaLimit(0.0);
    if (robot()->vel.linear.norm() < 0.05 && world_model()->ball.isStopped(0.05)) {
      command->dribble(0.0);
    } else {
      command->dribble(0.3);
    }

    return Status::RUNNING;
  });

  addTransition(SingleBallPlacementStates::SLEEP, SingleBallPlacementStates::ENTRY_POINT, [this]() {
    Point placement_target;
    placement_target << getParameter<double>("placement_x"), getParameter<double>("placement_y");
    // ルール 5.2 0.15m以内で認められる。再配置が必要場合のみ、 ENTRY_POINTへ移動
    return (world_model()->ball.pos - placement_target).norm() > 0.15;
  });

  addTransition(SingleBallPlacementStates::SLEEP, SingleBallPlacementStates::LEAVE_BALL, [this]() {
    pull_back_angle = robot()->pose.theta;
    return skill_status == Status::SUCCESS;
  });

  addStateFunction(SingleBallPlacementStates::LEAVE_BALL, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text(state_string)
      .fill("white")
      .fontSize(100)
      .build();
    // メモ：().normalized() * 0.8したらなぜかゼロベクトルが出来上がってしまう
    Vector2 diff = (robot()->pose.pos - world_model()->ball.pos);
    diff.normalize();
    diff = diff * 0.8;
    auto leave_pos = world_model()->ball.pos + diff;

    command->setTargetTheta(pull_back_angle);
    command->setTargetPosition(leave_pos);
    command->setOmegaLimit(0.0);
    command->setMaxVelocity(1.0);
    command->disableAnyAreaAvoidance();
    return skill_status;
  });

  addTransition(
    SingleBallPlacementStates::LEAVE_BALL, SingleBallPlacementStates::ENTRY_POINT, [this]() {
      Point placement_target;
      placement_target << getParameter<double>("placement_x"), getParameter<double>("placement_y");
      // ルール 5.2 0.15m以内で認められる。再配置が必要場合のみ、 ENTRY_POINTへ移動
      return (world_model()->ball.pos - placement_target).norm() > 0.15;
    });
}

void SingleBallPlacement::print(std::ostream & os) const
{
  os << "[SingleBallPlacement]";

  using enum SingleBallPlacementStates;
  switch (getCurrentState()) {
    case GO_OVER_BALL:
      go_over_ball->print(os);
      break;
    case CONTACT_BALL:
      os << " CONTACT_BALL";
      break;
    case MOVE_TO_TARGET:
      os << " MOVE_TO_TARGET";
      break;
    case SLEEP:
      sleep->print(os);
      break;
    case LEAVE_BALL:
      os << " LEAVE_BALL";
      break;
    default:
      os << " UNKNOWN";
      break;
  }
}
}  // namespace crane::skills
