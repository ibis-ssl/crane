// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/attacker.hpp>

namespace crane::skills
{
Attacker::Attacker(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<AttackerState, RobotCommandWrapperPosition>(
    "Attacker", base, AttackerState::ENTRY_POINT),
  kick_target(getContextReference<Point>("kick_target")),
  forced_pass_receiver_id(getContextReference<int>("forced_pass_receiver")),
  forced_pass_phase(getContextReference<int>("forced_pass_phase", 0)),
  kick_skill(base),
  goal_kick_skill(base),
  receive_skill(base),
  redirect_skill(base),
  steal_ball_skill(base)
{
  receive_skill.setParameter("policy", std::string("min_slack"));
  setParameter("receiver_id", 0);
  addStateFunction(
    AttackerState::ENTRY_POINT,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "ENTRY_POINT" << std::endl;
      return Status::RUNNING;
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::FORCED_PASS, [this]() -> bool {
    // セットプレイのときは強制パス
    auto game_command = world_model()->play_situation.getSituationCommandID();
    if (
      game_command == crane_msgs::msg::PlaySituation::OUR_DIRECT_FREE ||
      game_command == crane_msgs::msg::PlaySituation::OUR_INDIRECT_FREE ||
      game_command == crane_msgs::msg::PlaySituation::OUR_KICKOFF_START) {
      auto best_receiver = selectPassReceiver();
      forced_pass_receiver_id = best_receiver->id;
      forced_pass_phase = 1;
      return true;
    } else {
      return false;
    }
  });

  addStateFunction(
    AttackerState::FORCED_PASS,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      auto receiver = world_model()->getOurRobot(forced_pass_receiver_id);
      switch (forced_pass_phase) {
        case 0: {
          // 90度別の方向で構えて敵のプレッシャーをかわす
          Point target =
            world_model()->ball.pos +
            getVerticalVec(receiver->pose.pos - world_model()->ball.pos).normalized() * 0.3;
          command.setTargetPosition(target).lookAtBallFrom(target).enableBallAvoidance();
          if (robot()->getDistance(target) < 0.1) {
            forced_pass_phase = 1;
          }
          break;
        }
        case 1: {
          // 敵陣側に味方ロボットがいればパス
          if((receiver->pose.pos - world_model()->getOurGoalCenter()).norm() > (world_model()->ball.pos - world_model()->getOurGoalCenter()).norm()){
            kick_skill.setParameter("target", receiver->pose.pos);
          }else{
            kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
          }

          kick_skill.setParameter("dot_threshold", 0.95);
          kick_skill.setParameter("kick_power", 0.5);
          Segment kick_line{world_model()->ball.pos, receiver->pose.pos};
          // 近くに敵ロボットがいればチップキック
          if(const auto enemy_robots = world_model()->theirs.getAvailableRobots(); not enemy_robots.empty()) {
            const auto & [nearest_enemy, enemy_distance] =
              world_model()->getNearestRobotWithDistanceFromSegment(
                kick_line, enemy_robots);
            if (enemy_distance < 0.4 && nearest_enemy->getDistance(world_model()->ball.pos) < 2.0) {
              kick_skill.setParameter("kick_with_chip", true);
            }
          }
          kick_skill.run(visualizer);
          break;
        }
        default:
          return Status::FAILURE;
      }
      return Status::RUNNING;
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::CUT_THEIR_PASS, [this]() -> bool {
    return not world_model()->isOurBallByBallOwnerCalculator() &&
           world_model()->ball.isMoving(0.2) &&
           world_model()->ball.isMovingTowards(robot()->pose.pos);
  });

  addTransition(AttackerState::CUT_THEIR_PASS, AttackerState::ENTRY_POINT, [this]() -> bool {
    return world_model()->isOurBallByBallOwnerCalculator() or world_model()->ball.isStopped(0.2);
  });

  addStateFunction(
    AttackerState::CUT_THEIR_PASS,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      visualizer->addCircle(robot()->pose.pos, 0.25, 1, "blue", "white", 0.5);
      return receive_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::STEAL_BALL, [this]() -> bool {
    // 止まっているボールを相手が持っているとき
    const auto enemy_robots = world_model()->theirs.getAvailableRobots();
    return not enemy_robots.empty() &&
           not world_model()->isOurBallByBallOwnerCalculator() &&
           world_model()->ball.isStopped(0.1) &&
           world_model()
               ->getNearestRobotWithDistanceFromPoint(
                 world_model()->ball.pos, enemy_robots)
               .second < 0.5;
  });

  addTransition(AttackerState::STEAL_BALL, AttackerState::ENTRY_POINT, [this]() -> bool {
    return world_model()->isOurBallByBallOwnerCalculator();
  });

  addStateFunction(
    AttackerState::STEAL_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      visualizer->addCircle(robot()->pose.pos, 0.25, 1, "blue", "white", 1.0);
      return steal_ball_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::REDIRECT_GOAL_KICK, [this]() -> bool {
    // ボールが遠くにいる
    if (
      robot()->getDistance(world_model()->ball.pos) > 1.0 && world_model()->ball.vel.norm() > 0.5) {
      auto [best_angle, goal_angle_width] =
        world_model()->getLargestGoalAngleRangeFromPoint(robot()->pose.pos);
      double angle_diff_deg =
        std::abs(getAngleDiff(getAngle(world_model()->ball.pos - robot()->pose.pos), best_angle)) *
        180.0 / M_PI;
      if (goal_angle_width * 180.0 / M_PI > 10. && angle_diff_deg < 90.) {
        // ゴールが見えている && リダイレクト角度が90度以内
        return true;
      }
    }
    return false;
  });

  addTransition(AttackerState::REDIRECT_GOAL_KICK, AttackerState::ENTRY_POINT, [this]() -> bool {
    // ボールが止まっている
    if (world_model()->ball.vel.norm() < 0.5) {
      return true;
    } else if (not world_model()->isOurBallByBallOwnerCalculator()) {
      // 敵にボールを奪われた
      return true;
    } else {
      return false;
    }
  });

  addStateFunction(
    AttackerState::REDIRECT_GOAL_KICK,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      auto target = [&]() -> Point {
        double angle = GoalKick::getBestAngleToShootFromPoint(
          10.0 * M_PI / 180., robot()->pose.pos, world_model());
        Segment shoot_line{robot()->pose.pos, robot()->pose.pos + getNormVec(angle) * 10.};
        Segment goal_line;
        goal_line.first << world_model()->getTheirGoalCenter().x(),
          -world_model()->field_size.y() * 0.5;
        goal_line.second << world_model()->getTheirGoalCenter().x(),
          world_model()->field_size.y() * 0.5;
        std::vector<Point> intersection_points;
        bg::intersection(shoot_line, goal_line, intersection_points);
        if (intersection_points.empty()) {
          return world_model()->getTheirGoalCenter();
        } else {
          return intersection_points.front();
        }
      }();

      redirect_skill.setParameter("redirect_target", target);
      redirect_skill.setParameter("policy", std::string("max_slack"));
      redirect_skill.setParameter("kick_power", 0.8);
      return redirect_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::GOAL_KICK, [this]() -> bool {
    auto [best_angle, goal_angle_width] =
      world_model()->getLargestGoalAngleRangeFromPoint(world_model()->ball.pos);
    // ボールが近い条件はいらないかも？
    return robot()->getDistance(world_model()->ball.pos) < 2.0 &&
           goal_angle_width * 180.0 / M_PI > 5.;
  });

  addTransition(AttackerState::GOAL_KICK, AttackerState::ENTRY_POINT, [this]() -> bool {
    // ボールが早い
    return world_model()->ball.isMoving(1.0);
  });

  addStateFunction(
    AttackerState::GOAL_KICK,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      return goal_kick_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::CLEARING_KICK, [this]() -> bool {
    // 未実装：やばいときに蹴る
    return false;
  });

  addStateFunction(
    AttackerState::CLEARING_KICK,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
      kick_skill.setParameter("kick_power", 1.0);
      kick_skill.setParameter("dot_threshold", 0.9);
      kick_skill.setParameter("kick_with_chip", true);
      return kick_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::STANDARD_PASS, [this]() -> bool {
    if (robot()->getDistance(world_model()->ball.pos) > 1.0 or world_model()->ball.isStopped(1.0)) {
      return false;
    }

    auto our_robots = world_model()->ours.getAvailableRobots(robot()->id);
    const auto enemy_robots = world_model()->theirs.getAvailableRobots();
    // TODO(HansRobo): しっかりパス先を選定する
    //    int receiver_id = getParameter<int>("receiver_id");
    double best_score = 0.0;
    Point best_target;
    for (auto & our_robot : our_robots) {
      Segment ball_to_target{world_model()->ball.pos, our_robot->pose.pos};
      auto target = our_robot->pose.pos;
      double score = 1.0;
      // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
      auto [best_angle, goal_angle_width] =
        world_model()->getLargestGoalAngleRangeFromPoint(target);
      score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);

      // 敵ゴールに近いときはスコアを上げる
      double normed_distance_to_their_goal =
        ((target - world_model()->getTheirGoalCenter()).norm() -
         (world_model()->field_size.x() * 0.5)) /
        (world_model()->field_size.x() * 0.5);
      // マイナスのときはゴールに近い
      score *= (1.0 - normed_distance_to_their_goal);

      if(not enemy_robots.empty()) {
        auto [nearest_enemy, enemy_distance] = world_model()->getNearestRobotWithDistanceFromSegment(
          ball_to_target, world_model()->theirs.getAvailableRobots());
        // ボールから遠い敵がパスコースを塞いでいる場合は諦める
        if (nearest_enemy->getDistance(world_model()->ball.pos) > 1.0 && enemy_distance < 0.4) {
          score = 0.0;
        }
        // パスラインに敵がいるときはスコアを下げる
        score *= 1.0 / (1.0 + enemy_distance);
      }

      if (score > best_score) {
        best_score = score;
        best_target = target;
      }
    }

    return best_score > 0.5;
  });

  addTransition(AttackerState::STANDARD_PASS, AttackerState::ENTRY_POINT, [this]() -> bool {
    // ボールが早い
    return world_model()->ball.isMoving(1.0);
  });

  addStateFunction(
    AttackerState::STANDARD_PASS,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      auto our_robots = world_model()->ours.getAvailableRobots(robot()->id);
      const auto enemy_robots = world_model()->theirs.getAvailableRobots();
      // TODO(HansRobo): しっかりパス先を選定する
      //    int receiver_id = getParameter<int>("receiver_id");
      double best_score = 0.0;
      Point best_target;
      for (auto & our_robot : our_robots) {
        Segment ball_to_target{world_model()->ball.pos, our_robot->pose.pos};
        auto target = our_robot->pose.pos;
        double score = 1.0;
        // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
        auto [best_angle, goal_angle_width] =
          world_model()->getLargestGoalAngleRangeFromPoint(target);
        score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);

        // 敵ゴールに近いときはスコアを上げる
        double normed_distance_to_their_goal =
          ((target - world_model()->getTheirGoalCenter()).norm() -
           (world_model()->field_size.x() * 0.5)) /
          (world_model()->field_size.x() * 0.5);
        // マイナスのときはゴールに近い
        score *= (1.0 - normed_distance_to_their_goal);

        if(not enemy_robots.empty()) {
          auto [nearest_enemy, enemy_distance] =
            world_model()->getNearestRobotWithDistanceFromSegment(
              ball_to_target, world_model()->theirs.getAvailableRobots());
          // ボールから遠い敵がパスコースを塞いでいる場合は諦める
          if (nearest_enemy->getDistance(world_model()->ball.pos) > 1.0 && enemy_distance < 0.4) {
            score = 0.0;
          }
          // パスラインに敵がいるときはスコアを下げる
          score *= 1.0 / (1.0 + enemy_distance);
        }

        if (score > best_score) {
          best_score = score;
          best_target = target;
        }
      }

      visualizer->addLine(world_model()->ball.pos, best_target, 1, "red");

      kick_skill.setParameter("target", best_target);
      Segment ball_to_target{world_model()->ball.pos, best_target};
      if(not enemy_robots.empty()) {
        auto [nearest_enemy, enemy_distance] =
          world_model()->getNearestRobotWithDistanceFromSegment(
            ball_to_target, world_model()->theirs.getAvailableRobots());
        if (nearest_enemy->getDistance(world_model()->ball.pos) < 2.0) {
          kick_skill.setParameter("kick_with_chip", true);
        }
      }
      kick_skill.setParameter("kick_power", 0.8);
      kick_skill.setParameter("dot_threshold", 0.95);
      return kick_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::LOW_CHANCE_GOAL_KICK, [this]() -> bool {
    // ボールが近く、相手コートにいるとき（本当にチャンスが無いとき(隙間が1deg以下)は除外）
    double x_diff_with_their_goal =
      std::abs(world_model()->getTheirGoalCenter().x() - world_model()->ball.pos.x());
    auto [best_angle, goal_angle_width] =
      world_model()->getLargestGoalAngleRangeFromPoint(world_model()->ball.pos);
    return robot()->getDistance(world_model()->ball.pos) < 1.0 &&
           x_diff_with_their_goal < world_model()->field_size.x() * 0.5 &&
           goal_angle_width * 180.0 / M_PI > 1.;
  });

  addTransition(AttackerState::LOW_CHANCE_GOAL_KICK, AttackerState::ENTRY_POINT, [this]() -> bool {
    // 敵にボールを奪われた
    return not world_model()->isOurBallByBallOwnerCalculator();
  });

  addStateFunction(
    AttackerState::LOW_CHANCE_GOAL_KICK,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      return goal_kick_skill.run(visualizer);
    });

  addTransition(
    AttackerState::ENTRY_POINT, AttackerState::MOVE_BALL_TO_OPPONENT_HALF, [this]() -> bool {
      // ボールに近く、自コートにいるとき
      double x_diff_with_their_goal =
        std::abs(world_model()->getTheirGoalCenter().x() - world_model()->ball.pos.x());
      return robot()->getDistance(world_model()->ball.pos) < 1.0 &&
             x_diff_with_their_goal >= world_model()->field_size.x() * 0.5;
    });

  addTransition(
    AttackerState::MOVE_BALL_TO_OPPONENT_HALF, AttackerState::ENTRY_POINT, [this]() -> bool {
      // 敵にボールを奪われた
      return not world_model()->isOurBallByBallOwnerCalculator();
    });

  addStateFunction(
    AttackerState::MOVE_BALL_TO_OPPONENT_HALF,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
      kick_skill.setParameter("kick_power", 1.0);
      kick_skill.setParameter("dot_threshold", 0.9);
      kick_skill.setParameter("kick_with_chip", true);
      return kick_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::RECEIVE_BALL, [this]() -> bool {
    return world_model()->ball.isMoving(0.2) &&
           world_model()->ball.isMovingTowards(robot()->pose.pos);
  });

  addTransition(AttackerState::RECEIVE_BALL, AttackerState::ENTRY_POINT, [this]() -> bool {
    // ボールが止まっている
    if (world_model()->ball.vel.norm() < 0.5) {
      return true;
    } else if (not world_model()->isOurBallByBallOwnerCalculator()) {
      // 敵にボールを奪われた
      return true;
    } else {
      return false;
    }
  });

  addStateFunction(
    AttackerState::RECEIVE_BALL,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      return receive_skill.run(visualizer);
    });

  addTransition(AttackerState::RECEIVE_BALL, AttackerState::ENTRY_POINT, [this]() -> bool {
    // 一定以上ボールに触れたら終了
    using std::chrono_literals::operator""s;
    return robot()->ball_contact.getContactDuration() > 0.2s;
  });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::GO_TO_BALL, [this]() -> bool {
    // 最終防壁
    return true;
  });

  addTransition(AttackerState::GO_TO_BALL, AttackerState::ENTRY_POINT, [this]() -> bool {
    // 最終防壁なので毎回戻す
    return true;
  });

  addStateFunction(
    AttackerState::GO_TO_BALL,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      // ボールに向かって移動
      command.setTargetPosition(world_model()->ball.pos);
      return Status::RUNNING;
    });
}

std::shared_ptr<RobotInfo> Attacker::selectPassReceiver()
{
  auto our_robots = world_model()->ours.getAvailableRobots(robot()->id);
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


    if(not enemy_robots.empty()) {
      auto [nearest_enemy, enemy_distance] = world_model()->getNearestRobotWithDistanceFromSegment(
        ball_to_target, enemy_robots);
      // ボールから遠い敵がパスコースを塞いでいる場合は諦める
      if (nearest_enemy->getDistance(world_model()->ball.pos) > 1.0 && enemy_distance < 0.4) {
        score = 0.0;
      }
      // パスラインに敵がいるときはスコアを下げる
      score *= 1.0 / (1.0 + enemy_distance);
    }

    if (score > best_score) {
      best_score = score;
      best_bot = our_robot;
    }
  }

  return best_bot;
}
}  // namespace crane::skills
