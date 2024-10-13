// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/single_ball_placement.hpp>

namespace crane::skills
{

SingleBallPlacement::SingleBallPlacement(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<SingleBallPlacementStates>(
    "SingleBallPlacement", base, SingleBallPlacementStates::ENTRY_POINT)
{
  setParameter("placement_x", 0.);
  setParameter("placement_y", 0.);

  // マイナスするとコート内も判定される
  setParameter("コート端判定のオフセット", 0.0);

  addStateFunction(
    SingleBallPlacementStates::ENTRY_POINT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      command.stopHere();
      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::ENTRY_POINT, SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() {
      auto placement_target = world_model()->getBallPlacementTarget();
      if (
        placement_target && bg::distance(world_model()->ball.pos, placement_target.value()) > 0.1) {
        return true;
      } else {
        // 動かす必要がなければそのまま
        return false;
      }
    });

  addTransition(
    SingleBallPlacementStates::ENTRY_POINT, SingleBallPlacementStates::LEAVE_BALL, [this]() {
      if (bg::distance(world_model()->ball.pos, robot()->pose.pos) < 0.1) {
        // ボールに近すぎたら離れる
        return true;
      } else {
        return false;
      }
    });

  // 端にある場合、コート側からアプローチする
  addStateFunction(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      if (not pull_back_target) {
        pull_back_target = world_model()->ball.pos;
        const auto offset = getParameter<double>("コート端判定のオフセット");
        const auto threshold_x = world_model()->field_size.x() * 0.5 + offset;
        const auto threshold_y = world_model()->field_size.y() * 0.5 + offset;
        if (std::abs(pull_back_target->x()) > threshold_x) {
          pull_back_target->x() = std::copysign(threshold_x, pull_back_target->x());
        }
        if (std::abs(pull_back_target->y()) > threshold_y) {
          pull_back_target->y() = std::copysign(threshold_y, pull_back_target->y());
        }

        if (pull_back_target->x() > 0.) {
          pull_back_target->x() -= 0.3;
        } else {
          pull_back_target->x() += 0.3;
        }
        if (pull_back_target->y() > 0.) {
          pull_back_target->y() -= 0.3;
        } else {
          pull_back_target->y() += 0.3;
        }
      }
      command.setTargetPosition(pull_back_target.value());
      command.lookAtBallFrom(pull_back_target.value()).disablePlacementAvoidance();
      command.disableGoalAreaAvoidance().disableBallAvoidance().disableRuleAreaAvoidance();
      double max_vel = std::min(1.5, robot()->getDistance(pull_back_target.value()) + 0.1);
      command.setMaxVelocity(max_vel);
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
  addStateFunction(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      command.disablePlacementAvoidance()
        .disableBallAvoidance()
        .disableGoalAreaAvoidance()
        .disableRuleAreaAvoidance();
      command.setTargetPosition(world_model()->ball.pos);
      command.setTerminalVelocity(0.5);
      command.setMaxVelocity(1.0);

      const auto & ball_pos = world_model()->ball.pos;
      const Vector2 field = world_model()->field_size * 0.5;
      if (
        std::abs(ball_pos.x()) > (field.x() - 0.05) &&
        std::abs(ball_pos.y()) > (field.y() - 0.05)) {
        // ボールが角にある場合は引っ張る
        command.dribble(0.5);
      } else {
        // 角ではない場合は蹴る
        command.kickStraight(0.1);
      }
      return skill_status;
    });

  // skill_status == Status::SUCCESSの場合に次のステートへ
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
      // ボールが角にある場合は引っ張る
      bool is_corner =
        std::abs(ball_pos.x()) > (field.x() - 0.05) && std::abs(ball_pos.y()) > (field.y() - 0.05);
      std::cout << "is_corner: " << is_corner << ", contact duration: "
                << robot()->ball_contact.getContactDuration().count() / 1e6 << std::endl;
      return is_corner && robot()->ball_contact.getContactDuration().count() / 1e6 > 500;
    });

  // 失敗の場合は最初に戻る
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE, [this]() {
      if (not get_ball_contact) {
        return false;
      } else {
        return skill_status == Status::FAILURE;
      }
    });

  // PULL_BACK_FROM_EDGE_PULL
  addStateFunction(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      command.setDribblerTargetPosition(pull_back_target.value());
      // 角度はそのまま引っ張りたいので指定はしない
      command.dribble(0.6);
      command.setMaxVelocity(0.15);
      command.disablePlacementAvoidance();
      command.disableGoalAreaAvoidance();
      command.disableBallAvoidance();
      command.disableRuleAreaAvoidance();
      return Status::RUNNING;
    });

  // pull_back_targetに到着したら始めに戻る（GO_OVER_BALLに転送される）
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() { return (robot()->kicker_center() - pull_back_target.value()).norm() < 0.03; });

  // ボールが離れたら始めに戻る
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() { return robot()->getDistance(world_model()->ball.pos) > 0.15; });

  addStateFunction(
    SingleBallPlacementStates::GO_OVER_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      command.setMaxVelocity(1.5);
      Point placement_target;
      placement_target << getParameter<double>("placement_x"), getParameter<double>("placement_y");
      const auto & ball_pos = world_model()->ball.pos;
      Point target = ball_pos + (ball_pos - placement_target).normalized() * 0.3;
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
        command.setTargetPosition(around_point);
      } else {
        command.setTargetPosition(target);
      }
      if (robot()->getDistance(world_model()->ball.pos) < 0.2) {
        // ロボットがボールに近い場合は一度引きの動作を入れる
        // これは端からのPULLが終わった後の誤作動を防ぐための動きである
        target << 0, 0;
      }
      command.lookAtBallFrom(target);
      command.disablePlacementAvoidance();
      command.disableGoalAreaAvoidance();
      command.enableBallAvoidance();
      command.disableRuleAreaAvoidance();
      command.dribble(0.0);

      if (robot()->getDistance(target) < 0.05) {
        skill_status = Status::SUCCESS;
      } else {
        skill_status = Status::RUNNING;
      }
      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::GO_OVER_BALL, SingleBallPlacementStates::CONTACT_BALL,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::CONTACT_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      if (not get_ball_contact) {
        get_ball_contact = std::make_shared<GetBallContact>(command_base);
      }

      skill_status = get_ball_contact->run(visualizer);
      command.disablePlacementAvoidance();
      command.setMaxVelocity(0.5);
      command.setMaxAcceleration(1.0);

      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::CONTACT_BALL, SingleBallPlacementStates::MOVE_TO_TARGET,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::MOVE_TO_TARGET,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      if (not move_with_ball) {
        move_with_ball = std::make_shared<MoveWithBall>(command_base);
        move_with_ball->setParameter("target_x", getParameter<double>("placement_x"));
        move_with_ball->setParameter("target_y", getParameter<double>("placement_y"));
        move_with_ball->setParameter("dribble_power", 0.3);
        move_with_ball->setParameter("ball_stabilizing_time", 3.);
      }

      skill_status = move_with_ball->run(visualizer);
      command.disablePlacementAvoidance();
      command.disableGoalAreaAvoidance();
      command.disableRuleAreaAvoidance();
      command.setMaxVelocity(0.5);
      command.setMaxAcceleration(1.0);
      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::MOVE_TO_TARGET, SingleBallPlacementStates::SLEEP,
    [this]() { return skill_status == Status::SUCCESS; });

  // ボールが離れたら始めに戻る
  addTransition(
    SingleBallPlacementStates::MOVE_TO_TARGET,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this]() { return skill_status == Status::FAILURE; });

  addStateFunction(
    SingleBallPlacementStates::SLEEP,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      if (not sleep) {
        sleep = std::make_shared<Sleep>(command_base);
        sleep->setParameter("duration", 2.0);
      }
      skill_status = sleep->run(visualizer);
      command.stopHere();
      command.disablePlacementAvoidance();
      command.disableGoalAreaAvoidance();
      command.disableBallAvoidance();
      command.disableRuleAreaAvoidance();
      return Status::RUNNING;
    });

  addTransition(SingleBallPlacementStates::SLEEP, SingleBallPlacementStates::LEAVE_BALL, [this]() {
    pull_back_angle = robot()->pose.theta;
    return skill_status == Status::SUCCESS;
  });

  addStateFunction(
    SingleBallPlacementStates::LEAVE_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
      visualizer->addPoint(robot()->pose.pos, 0, "white", 1.0, state_string);
      if (not set_target_position) {
        set_target_position = std::make_shared<CmdSetTargetPosition>(command_base);
      }
      // メモ：().normalized() * 0.8したらなぜかゼロベクトルが出来上がってしまう
      Vector2 diff = (robot()->pose.pos - world_model()->ball.pos);
      diff.normalize();
      diff = diff * 0.8;
      auto leave_pos = world_model()->ball.pos + diff;
      set_target_position->setParameter("x", leave_pos.x());
      set_target_position->setParameter("y", leave_pos.y());
      set_target_position->setParameter("reach_threshold", 0.05);

      command.setTargetTheta(pull_back_angle);
      command.disablePlacementAvoidance();
      command.disableBallAvoidance();
      command.disableGoalAreaAvoidance();
      command.disableRuleAreaAvoidance();
      skill_status = set_target_position->run(visualizer);
      return skill_status;
    });

  addTransition(
    SingleBallPlacementStates::LEAVE_BALL, SingleBallPlacementStates::ENTRY_POINT,
    [this]() { return skill_status == Status::SUCCESS; });
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
      get_ball_contact->print(os);
      break;
    case MOVE_TO_TARGET:
      move_with_ball->print(os);
      break;
    case SLEEP:
      sleep->print(os);
      break;
    case LEAVE_BALL:
      set_target_position->print(os);
      break;
    default:
      os << " UNKNOWN";
      break;
  }
}
}  // namespace crane::skills
