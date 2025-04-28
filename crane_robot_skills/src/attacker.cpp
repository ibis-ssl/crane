// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_basics/ddps.hpp>
#include <crane_basics/pass.hpp>
#include <crane_robot_skills/attacker.hpp>

namespace crane::skills
{
void Attacker::initialize()
{
  setParameter("moving_ball_velocity", 1.0);
  setPreUpdateFunction([&]() { command->clearSkillStates(); });
  receive_skill.setParameter("policy", std::string("closest"));
  addStateFunction(AttackerState::ENTRY_POINT, [this]() -> Status {
    command->setTargetPosition(world_model()->ball.pos);
    pass_receiver_id = std::nullopt;
    visualizer->circle()
      .center(robot()->pose.pos)
      .radius(2.0)
      .stroke("black")
      .fill("black", 0.5)
      .build();
    return Status::RUNNING;
  });

  // "ENTRY_POINT"のstate functionは実行されない（skill_base.hppのStateMachine::update参照）
  // ので自分への遷移関数で初期化処理を実装
  addTransition(AttackerState::ENTRY_POINT, AttackerState::ENTRY_POINT, [this]() -> bool {
    pass_receiver_id = std::nullopt;
    return false;
  });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::FORCED_PASS, [this]() -> bool {
    // セットプレイのときは強制パス
    auto game_command = world_model()->getMsg().play_situation.command.value;
    // ボールの停止条件は、INPLAY切り替わりの遅延対策
    if (
      (game_command == crane_msgs::msg::PlaySituation::OUR_DIRECT_FREE ||
       game_command == crane_msgs::msg::PlaySituation::OUR_KICKOFF_START) &&
      world_model()->ball.isStopped()) {
      if (auto best_receiver = selectPassReceiver(); best_receiver) {
        forced_pass_receiver_id = best_receiver->id;
        auto receiver = world_model()->getOurRobot(forced_pass_receiver_id);
        pass_receiver_id = best_receiver->id;
        kick_skill.setParameter("target", receiver->pose.pos);
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  });

  // ----- ダブルタッチ防止の為、FORCED_PASS -> ENTRY_POINT の状態遷移は設けない ------- //

  addStateFunction(AttackerState::FORCED_PASS, [this]() -> Status {
    // パス
    command->disableBallAvoidance();
    if (pass_receiver_id) {
      kick_target = world_model()->getOurRobot(pass_receiver_id.value())->pose.pos;
    }
    kick_skill.setParameter("target", kick_target);
    Segment kick_line{world_model()->ball.pos, kick_target};
    // 近くに敵ロボットがいればチップキック
    bool chip_kick = false;
    if (auto nearest_enemy = world_model()->getNearestRobotWithDistanceFromSegment(
          kick_line, world_model()->theirs.getAvailableRobots());
        nearest_enemy.has_value()) {
      if (
        nearest_enemy->distance < 0.4 &&
        nearest_enemy->robot->getDistance(world_model()->ball.pos) < 2.0) {
        chip_kick = true;
      }
    }

    auto pass_analysis = getPassAnalysis(
      world_model()->ball.pos, kick_target, world_model()->theirs.getAvailableRobots());
    if (pass_analysis.need_chip) {
      kick_skill.setParameter("chip_kick", true);
      kick_skill.setParameter("use_target_chip_distance", true);
      kick_skill.setParameter("target_chip_distance", pass_analysis.required_chip_distance + 0.2);
      kick_skill.setParameter("with_dribble", true);
      kick_skill.setParameter("dribble_power", 0.7);
      // kick_skill.setParameter("kick_power", 0.9);
    } else {
      kick_skill.setParameter("chip_kick", false);
      kick_skill.setParameter("use_target_kick_speed", true);
      kick_skill.setParameter("target_kick_speed", 2.0);
      // kick_skill.setParameter("kick_power", 0.5);
      kick_skill.setParameter("dribble_power", 0.0);
    }
    kick_skill.run();

    return Status::RUNNING;
  });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::RECEIVE, [this]() -> bool {
    // ボールが遠くにいる/動いている/自分に向かってきている
    if (
      robot()->getDistance(world_model()->ball.pos) > 1.0 &&
      world_model()->ball.isMoving(getParameter<double>("moving_ball_velocity")) &&
      world_model()->ball.isMovingTowards(robot()->pose.pos)) {
      return true;
    } else {
      return false;
    }
  });

  addTransition(AttackerState::RECEIVE, AttackerState::ENTRY_POINT, [this]() -> bool {
    using std::chrono_literals::operator""s;
    if (world_model()->ball.isStopped(getParameter<double>("moving_ball_velocity"))) {
      // ボールが止まっている
      return true;
    } else if (world_model()->ball.isMovingAwayFrom(robot()->pose.pos)) {
      // ボールが自分から離れていっている（多分受取に失敗した）
      return true;
    } else if (robot()->ball_contact.getContactDuration() > 0.2s) {
      // 受取に成功してドリブラで触れている
      return true;
    } else {
      return false;
    }
  });

  addStateFunction(AttackerState::RECEIVE, [this]() -> Status {
    auto redirect_target = [&]() -> Point {
      double angle = GoalKick::getBestAngleToShootFromPoint(
        10.0 * M_PI / 180., robot()->pose.pos, world_model(), visualizer);
      Segment shoot_line{robot()->pose.pos, robot()->pose.pos + getNormVec(angle) * 10.};
      Segment goal_line;
      goal_line.first << world_model()->getTheirGoalCenter().x(),
        -world_model()->field_size.y() * 0.5;
      goal_line.second << world_model()->getTheirGoalCenter().x(),
        world_model()->field_size.y() * 0.5;
      if (auto intersection_points = getIntersections(shoot_line, goal_line);
          intersection_points.empty()) {
        return world_model()->getTheirGoalCenter();
      } else {
        return intersection_points.front();
      }
    }();

    auto [best_angle, goal_angle_width] =
      world_model()->getLargestGoalAngleRangeFromPoint(robot()->pose.pos);
    double angle_diff_deg =
      std::abs(getAngleDiff(getAngle(world_model()->ball.pos - robot()->pose.pos), best_angle)) *
      180.0 / M_PI;

    // ゴールが見えている && リダイレクト角度が45度以内
    bool redirect = goal_angle_width * 180.0 / M_PI > 10. && angle_diff_deg < 45.;

    if (redirect) {
      printTextOnRobot("RECEIVE::REDIRECT");
      receive_skill.setParameter("enable_redirect", true);
      receive_skill.setParameter("redirect_target", redirect_target);
      receive_skill.setParameter("policy", std::string("closest"));
      receive_skill.setParameter("redirect_kick_power", 0.4);
      return receive_skill.run();
    } else {
      printTextOnRobot("RECEIVE::NORMAL");
      receive_skill.setParameter("enable_redirect", false);
      receive_skill.setParameter("policy", std::string("min_slack"));
      receive_skill.setParameter("dribble_power", 0.0);
      receive_skill.setParameter("enable_software_bumper", false);
      return receive_skill.run();
    }
  });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::KICK, [this]() -> bool { return true; });

  addTransition(AttackerState::KICK, AttackerState::ENTRY_POINT, [this]() -> bool {
    return world_model()->ball.isMoving(1.0);
    ;
  });

  addStateFunction(AttackerState::KICK, [this]() -> Status {
    auto [best_angle, goal_angle_width] =
      world_model()->getLargestGoalAngleRangeFromPoint(world_model()->ball.pos);

    auto our_robots = world_model()->ours.getAvailableRobots(robot()->id, true);
    const auto enemy_robots = world_model()->theirs.getAvailableRobots();

    auto pass_scores =
      world_model()->getMsg().game_analysis.pass_scores |
      ranges::view::filter([&](const auto & score_with_id) {
        // 自分自身とキーパーを除外
        auto target_robot_pos = world_model()->getOurRobot(score_with_id.id)->pose.pos;
        return score_with_id.id != robot()->id &&
               score_with_id.id != world_model()->getOurGoalieId() &&
               ((std::abs(world_model()->ball.pos.x() - world_model()->getTheirGoalCenter().x()) >
                 std::abs(target_robot_pos.x() - world_model()->getTheirGoalCenter().x())) ||
                std::abs(target_robot_pos.x() - world_model()->getTheirGoalCenter().x()) < 2.0);
      }) |
      ranges::to<std::vector>();

    if (not pass_scores.empty()) {
      // pass_scoresの先頭が一番スコアが高い
      kick_target = world_model()->getOurRobot(pass_scores.front().id)->pose.pos;
      pass_receiver_id = pass_scores.front().id;
    } else {
      pass_receiver_id = std::nullopt;
    }

    double x_diff_with_their_goal =
      std::abs(world_model()->getTheirGoalCenter().x() - world_model()->ball.pos.x());

    if (goal_angle_width > 180.0 / M_PI > 5.) {
      // GOAL_KICK
      printTextOnRobot("KICK::GOAL_KICK");
      goal_kick_skill.setParameter("キック角度の最低要求精度[deg]", 5.0);
      kick_skill.setParameter("kick_power", 0.8);
      return goal_kick_skill.run();
    } else if (pass_receiver_id.has_value()) {
      // STANDARD_PASS
      printTextOnRobot("KICK::STANDARD_PASS");
      kick_target = world_model()->getOurRobot(pass_receiver_id.value())->pose.pos;
      visualizer->line()
        .start(world_model()->ball.pos)
        .end(kick_target)
        .stroke("red")
        .strokeWidth(10)
        .build();

      kick_skill.setParameter("target", kick_target);
      kick_skill.setParameter("kick_power", 0.6);
      Segment ball_to_target{world_model()->ball.pos, kick_target};
      if (auto nearest_enemy = world_model()->getNearestRobotWithDistanceFromSegment(
            ball_to_target, world_model()->theirs.getAvailableRobots());
          nearest_enemy.has_value()) {
        if (nearest_enemy->robot->getDistance(world_model()->ball.pos) < 2.0) {
          kick_skill.setParameter("chip_kick", true);
          kick_skill.setParameter("kick_power", 0.8);
        }
      }
      return kick_skill.run();
    } else if (goal_angle_width > 180.0 / M_PI > 2.) {
      // LOW_CHANCE_GOAL_KICK
      printTextOnRobot("KICK::LOW_CHANCE_GOAL_KICK");
      return goal_kick_skill.run();
    } else if (
      robot()->getDistance(world_model()->ball.pos) < 1.0 &&
      x_diff_with_their_goal >= world_model()->field_size.x() * 0.5) {
      // MOVE_BALL_TO_OPPONENT_HALF
      printTextOnRobot("KICK::MOVE_BALL_TO_OPPONENT_HALF");
      kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
      kick_skill.setParameter("kick_power", 0.8);
      kick_skill.setParameter("chip_kick", true);
      command->disableBallAvoidance();
      return kick_skill.run();
    } else {
      // FINAL_GUARD
      printTextOnRobot("KICK::FINAL_GUARD");
      return goal_kick_skill.run();
    }
  });
}

std::shared_ptr<RobotInfo> Attacker::selectPassReceiver()
{
  auto our_robots = world_model()->ours.getAvailableRobots(robot()->id, true);
  const auto enemy_robots = world_model()->theirs.getAvailableRobots();
  double best_score = 0.0;
  std::shared_ptr<RobotInfo> best_bot = nullptr;
  for (auto & our_robot : our_robots) {
    Segment ball_to_target{world_model()->ball.pos, our_robot->pose.pos};
    auto target = our_robot->pose.pos;
    double score = 1.0;
    // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
    auto [best_angle, goal_angle_width] = world_model()->getLargestGoalAngleRangeFromPoint(target);
    score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);

    // 敵ゴールに近いときはスコアを上げる
    double normed_distance_to_their_goal = ((target - world_model()->getTheirGoalCenter()).norm() -
                                            (world_model()->field_size.x() * 0.5)) /
                                           (world_model()->field_size.x() * 0.5);
    // マイナスのときはゴールに近い
    score *= (1.0 - normed_distance_to_their_goal);

    if (auto nearest_enemy =
          world_model()->getNearestRobotWithDistanceFromSegment(ball_to_target, enemy_robots);
        nearest_enemy) {
      // ボールから遠い敵がパスコースを塞いでいる場合は諦める
      if (
        nearest_enemy->robot->getDistance(world_model()->ball.pos) > 1.0 &&
        nearest_enemy->distance < 0.4) {
        score = 0.0;
      }
      // パスラインに敵がいるときはスコアを下げる
      score *= 1.0 / (1.0 + nearest_enemy->distance);
    }

    if (score > best_score) {
      best_score = score;
      best_bot = our_robot;
    }
  }

  return best_bot;
}
}  // namespace crane::skills
