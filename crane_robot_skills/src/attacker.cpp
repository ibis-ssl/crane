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
  kick_skill(base),
  goal_kick_skill(base),
  receive_skill(base),
  redirect_skill(base)
{
  setParameter("receiver_id", 0);
  addStateFunction(
    AttackerState::ENTRY_POINT,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      std::cout << "ENTRY_POINT" << std::endl;
      kick_target = [&]() -> Point {
        auto [best_angle, goal_angle_width] =
          world_model()->getLargestGoalAngleRangeFromPoint(world_model()->ball.pos);
        // シュートの隙がないときは仲間へパス
        if (goal_angle_width < 0.07) {
          auto our_robots = world_model()->ours.getAvailableRobots(robot()->id);
          int receiver_id = getParameter<int>("receiver_id");
          Point target;
          if (auto receiver = std::find_if(
                our_robots.begin(), our_robots.end(), [&](auto e) { return e->id == receiver_id; });
              receiver != our_robots.end()) {
            target = receiver->get()->pose.pos;
          } else {
            auto nearest_robot = world_model()->getNearestRobotWithDistanceFromPoint(
              world_model()->ball.pos, our_robots);
            target = nearest_robot.first->pose.pos;
          }

          // 特に自コートでは後ろ向きの攻撃をしない
          if (
            (world_model()->ball.pos.x() - target.x()) > 0 &&
            (target - world_model()->getTheirGoalCenter()).norm() > 4.0) {
            target = world_model()->getTheirGoalCenter();
          }
          return target;
        } else {
          // シュートの隙があるときはシュート
          return world_model()->ball.pos + getNormVec(best_angle) * 0.5;
        }
      }();
      return Status::RUNNING;
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::FORCED_PASS, [this]() -> bool {
    // セットプレイのときは強制パス
    auto game_command = world_model()->play_situation.getSituationCommandID();
    if (
      game_command == crane_msgs::msg::PlaySituation::OUR_DIRECT_FREE ||
      game_command == crane_msgs::msg::PlaySituation::OUR_INDIRECT_FREE ||
      game_command == crane_msgs::msg::PlaySituation::OUR_KICKOFF_START) {
      return true;
    } else {
      return false;
    }
  });

  addStateFunction(
    AttackerState::FORCED_PASS,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      return Status::RUNNING;
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::REDIRECT_GOAL_KICK, [this]() -> bool {
    if (robot()->getDistance(world_model()->ball.pos) > 1.0) {
      const auto [min_slack_point, max_slack_point] =
        world_model()->getMinMaxSlackInterceptPoint({robot()});
    }
    return false;
  });

  addStateFunction(
    AttackerState::REDIRECT_GOAL_KICK,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      return Status::RUNNING;
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::GOAL_KICK, [this]() -> bool {
    auto [best_angle, goal_angle_width] =
      world_model()->getLargestGoalAngleRangeFromPoint(world_model()->ball.pos);
    return robot()->getDistance(world_model()->ball.pos) < 1.0 &&
           goal_angle_width * 180.0 / M_PI > 10.;
  });

  addStateFunction(
    AttackerState::GOAL_KICK,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      return goal_kick_skill.run(visualizer);
    });

  addTransition(
    AttackerState::ENTRY_POINT, AttackerState::CLEARING_KICK, [this]() -> bool { return false; });

  addStateFunction(
    AttackerState::CLEARING_KICK,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
      kick_skill.setParameter("kick_power", 1.0);
      kick_skill.setParameter("kick_with_chip", true);
      return kick_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::STANDARD_PASS, [this]() -> bool {
    auto our_robots = world_model()->ours.getAvailableRobots(robot()->id);
    // TODO(HansRobo): しっかりパス先を選定する
    //    int receiver_id = getParameter<int>("receiver_id");
    double best_score = 0.0;
    Point best_target;
    for (auto & our_robot : our_robots) {
      Segment ball_to_target{world_model()->ball.pos, our_robot->pose.pos};
      auto [nearest_enemy, enemy_distance] = world_model()->getNearestRobotWithDistanceFromSegment(
        ball_to_target, world_model()->theirs.getAvailableRobots());
      auto target = our_robot->pose.pos;
      double score = 1.0;
      // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
      auto [best_angle, goal_angle_width] =
        world_model()->getLargestGoalAngleRangeFromPoint(target);
      score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
      // ボールから遠い敵がパスコースを塞いでいる場合は諦める
      if (nearest_enemy->getDistance(world_model()->ball.pos) > 1.0 && enemy_distance < 0.4) {
        score = 0.0;
      }

      // 敵ゴールに近いときはスコアを上げる
      double normed_distance_to_their_goal =
        ((target - world_model()->getTheirGoalCenter()).norm() -
         (world_model()->field_size.x() * 0.5)) /
        (world_model()->field_size.x() * 0.5);
      // マイナスのときはゴールに近い
      score *= (1.0 - normed_distance_to_their_goal);

      // パスラインに敵がいるときはスコアを下げる
      score *= 1.0 / (1.0 + enemy_distance);

      if (score > best_score) {
        best_score = score;
        best_target = target;
      }
    }

    return best_score > 0.5;
  });

  addStateFunction(
    AttackerState::STANDARD_PASS,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      auto our_robots = world_model()->ours.getAvailableRobots(robot()->id);
      // TODO(HansRobo): しっかりパス先を選定する
      //    int receiver_id = getParameter<int>("receiver_id");
      double best_score = 0.0;
      Point best_target;
      for (auto & our_robot : our_robots) {
        Segment ball_to_target{world_model()->ball.pos, our_robot->pose.pos};
        auto [nearest_enemy, enemy_distance] =
          world_model()->getNearestRobotWithDistanceFromSegment(
            ball_to_target, world_model()->theirs.getAvailableRobots());
        auto target = our_robot->pose.pos;
        double score = 1.0;
        // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
        auto [best_angle, goal_angle_width] =
          world_model()->getLargestGoalAngleRangeFromPoint(target);
        score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
        // ボールから遠い敵がパスコースを塞いでいる場合は諦める
        if (nearest_enemy->getDistance(world_model()->ball.pos) > 1.0 && enemy_distance < 0.4) {
          score = 0.0;
        }

        // 敵ゴールに近いときはスコアを上げる
        double normed_distance_to_their_goal =
          ((target - world_model()->getTheirGoalCenter()).norm() -
           (world_model()->field_size.x() * 0.5)) /
          (world_model()->field_size.x() * 0.5);
        // マイナスのときはゴールに近い
        score *= (1.0 - normed_distance_to_their_goal);

        // パスラインに敵がいるときはスコアを下げる
        score *= 1.0 / (1.0 + enemy_distance);

        if (score > best_score) {
          best_score = score;
          best_target = target;
        }
      }
      kick_skill.setParameter("target", best_target);
      Segment ball_to_target{world_model()->ball.pos, best_target};
      auto [nearest_enemy, enemy_distance] = world_model()->getNearestRobotWithDistanceFromSegment(
        ball_to_target, world_model()->theirs.getAvailableRobots());
      if (nearest_enemy->getDistance(world_model()->ball.pos) < 2.0) {
        kick_skill.setParameter("kick_with_chip", true);
      }
      kick_skill.setParameter("kick_power", 0.5);
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

  addStateFunction(
    AttackerState::LOW_CHANCE_GOAL_KICK,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
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

  addStateFunction(
    AttackerState::MOVE_BALL_TO_OPPONENT_HALF,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
      kick_skill.setParameter("kick_power", 1.0);
      kick_skill.setParameter("kick_with_chip", true);
      return kick_skill.run(visualizer);
    });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::RECEIVE_BALL, [this]() -> bool {
    // TODO(HansRobo): もうちょっと条件を考える
    // 当てはまらないときは受け取りに行く
    return true;
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
}
}  // namespace crane::skills
