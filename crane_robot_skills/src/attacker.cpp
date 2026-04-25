// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/ddps.hpp>
#include <crane_physics/pass.hpp>
#include <crane_robot_skills/attacker.hpp>
#include <magic_enum/magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crane::skills
{
std::string Attacker::getStateName(int s)
{
  return std::string(magic_enum::enum_name(static_cast<AttackerState>(s)));
}

namespace
{
constexpr double GOAL_ANGLE_THRESHOLD_DEG = 3.0;
constexpr double GOAL_ANGLE_THRESHOLD_RAD = GOAL_ANGLE_THRESHOLD_DEG * M_PI / 180.0;
constexpr double LOW_CHANCE_GOAL_ANGLE_THRESHOLD_DEG = 2.0;
constexpr double PASS_OBSTACLE_DISTANCE = 0.4;
constexpr double BALL_CONTROL_DISTANCE = 1.0;
constexpr double CHIP_KICK_DISTANCE = 2.0;
constexpr double MOVING_BALL_VELOCITY = 1.0;
constexpr double ENEMY_NEAR_BALL_DISTANCE = 2.0;
constexpr double ENEMY_SLACK_RECEIVE_THRESHOLD = 0.3;  // RECEIVE抑制: 敵がこの秒数以上早いと諦める
constexpr double MIN_PASS_SCORE_ATTACKER = 0.2;        // パス品質の下限（二重チェック用）
constexpr double KICK_TIMEOUT_SEC = 3.0;               // KICK状態でボール無接触の場合のタイムアウト
constexpr double KICK_NO_CONTACT_THRESHOLD_SEC = 0.1;  // ボール接触とみなす最小継続時間
constexpr double KICK_BALL_STOPPED_VEL = 0.3;          // タイムアウト判定でのボール停止閾値
}  // namespace
void Attacker::initialize()
{
  setParameter("moving_ball_velocity", MOVING_BALL_VELOCITY);

  receive_skill.setParameter("policy", std::string("closest"));
  addStateFunction(static_cast<int>(AttackerState::ENTRY_POINT), [this]() -> Status {
    command->setTargetPosition(world_model()->ball().pos);
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
  addTransition(
    static_cast<int>(AttackerState::ENTRY_POINT), static_cast<int>(AttackerState::ENTRY_POINT),
    [this]() -> bool {
      in_kick_state = false;
      pass_receiver_id = std::nullopt;
      receive_skill.clearVisualizer();
      kick_skill.clearVisualizer();
      goal_kick_skill.clearVisualizer();
      return false;
    });

  addTransition(
    static_cast<int>(AttackerState::ENTRY_POINT), static_cast<int>(AttackerState::FORCED_PASS),
    [this]() -> bool {
      // セットプレイのときは強制パス
      auto game_command = world_model()->getMsg().play_situation.command.value;
      // ボールの停止条件は、INPLAY切り替わりの遅延対策
      if (
        (game_command == crane_msgs::msg::PlaySituation::OUR_DIRECT_FREE ||
         game_command == crane_msgs::msg::PlaySituation::OUR_KICKOFF_START) &&
        world_model()->ball().isStopped()) {
        // pass_target_id が有効な場合のみパス先を設定（無い場合は FORCED_PASS で敵ゴールへキック）
        if (world_model()->getMsg().game_analysis.pass_target_id >= 0) {
          forced_pass_receiver_id =
            static_cast<int>(world_model()->getMsg().game_analysis.pass_target_id);
          pass_receiver_id = forced_pass_receiver_id;
          auto receiver = world_model()->getOurRobot(pass_receiver_id.value());
          kick_skill.setParameter("target", receiver->pose.pos);
        }
        return true;
      } else {
        return false;
      }
    });

  // ----- ダブルタッチ防止の為、FORCED_PASS -> ENTRY_POINT の状態遷移は設けない ------- //

  addStateFunction(static_cast<int>(AttackerState::FORCED_PASS), [this]() -> Status {
    // パス
    command->disableBallAvoidance();
    command->setMaxVelocity("AttackerState::FORCED_PASS", 2.0);
    if (pass_receiver_id) {
      auto pass_receiver_pos = world_model()->getOurRobot(pass_receiver_id.value())->pose.pos;
      if (pass_receiver_pos.x() * world_model()->getAttackSideSign() > 0.) {
        // 自陣にいるときは強制的に攻撃対象ゴールを目標に設定
        kick_target = world_model()->getAttackGoalCenter();
      } else {
        // 敵陣に適当なロボットがいる場合はパス
        kick_target = pass_receiver_pos;
      }
    } else {
      kick_target = world_model()->getAttackGoalCenter();
    }
    kick_skill.setParameter("target", kick_target);
    Segment kick_line{world_model()->ball().pos, kick_target};
    configurePassKick(kick_target, kick_skill);
    kick_skill.run();

    return Status::RUNNING;
  });

  addTransition(
    static_cast<int>(AttackerState::ENTRY_POINT), static_cast<int>(AttackerState::RECEIVE),
    [this]() -> bool {
      // ボールが遠くにいて動いている場合にRECEIVEへ遷移する。
      // 以前の「ボール軌道の2.0m以内」チェックは削除した。
      // Attackerはボール取得が役割なので、転がるボールを積極的に追いかけるべきである。
      if (
        robot()->getDistance(world_model()->ball().pos) > BALL_CONTROL_DISTANCE &&
        world_model()->ball().isMoving(MOVING_BALL_VELOCITY) &&
        !world_model()->point_checker.isFriendPenaltyArea(world_model()->ball().pos, 0.15)) {
        // GameAnalysisの既存slackデータで敵との競合チェック（追加計算コストなし）
        const auto & ga = world_model()->getMsg().game_analysis;
        double my_min_slack = -100.0;
        for (const auto & s : ga.our_slack) {
          if (s.id == robot()->id) {
            my_min_slack = s.min.slack_time;
            break;
          }
        }
        double best_enemy_slack = -100.0;
        for (const auto & s : ga.their_slack) {
          best_enemy_slack = std::max(best_enemy_slack, static_cast<double>(s.min.slack_time));
        }
        if (best_enemy_slack > my_min_slack + ENEMY_SLACK_RECEIVE_THRESHOLD) {
          return false;
        }
        return true;
      } else {
        return false;
      }
    });

  addTransition(
    static_cast<int>(AttackerState::RECEIVE), static_cast<int>(AttackerState::ENTRY_POINT),
    [this]() -> bool {
      using std::chrono_literals::operator""s;
      if (world_model()->ball().isStopped(MOVING_BALL_VELOCITY)) {
        // ボールが止まっている
        return true;
      } else if ([&]() {
                   // ボール軌道の最近接点がボール現在位置より後方 = ボールが通り過ぎた
                   if (!world_model()->ball().isMoving(MOVING_BALL_VELOCITY)) return false;
                   auto result =
                     world_model()->ball().getClosestPointToTrajectory(robot()->pose.pos, 5.0);
                   Vector2 ball_dir = world_model()->ball().vel.normalized();
                   double proj = (result.closest_point - world_model()->ball().pos).dot(ball_dir);
                   return proj < -0.1;
                 }()) {
        // ボールが通り過ぎた（受取に失敗した）
        return true;
      } else if (robot()->ball_contact.getContactDuration() > 0.2s) {
        // 受取に成功してドリブラで触れている
        return true;
      } else {
        // 敵がボールに明らかに近い場合は諦めてENTRY_POINTへ（再割り当て待ち）
        auto enemies = world_model()->theirs().robotsWhere().available().get();
        auto nearest_enemy =
          world_model()->getNearestRobotWithDistanceFromPoint(world_model()->ball().pos, enemies);
        if (nearest_enemy.has_value()) {
          double my_dist = robot()->getDistance(world_model()->ball().pos);
          if (nearest_enemy->distance < my_dist * 0.7) {
            return true;
          }
        }
        return false;
      }
    });

  addStateFunction(static_cast<int>(AttackerState::RECEIVE), [this]() -> Status {
    auto redirect_target = [&]() -> Point {
      double angle = GoalKick::getBestAngleToShootFromPoint(
        10.0 * M_PI / 180., robot()->pose.pos, world_model(), visualizer);
      Segment shoot_line{robot()->pose.pos, robot()->pose.pos + getNormVec(angle) * 10.};
      Segment goal_line;
      goal_line.first << world_model()->getAttackGoalCenter().x(),
        -world_model()->fieldSize().y() * 0.5;
      goal_line.second << world_model()->getAttackGoalCenter().x(),
        world_model()->fieldSize().y() * 0.5;
      if (
        auto intersection_points = getIntersections(shoot_line, goal_line);
        intersection_points.empty()) {
        return world_model()->getAttackGoalCenter();
      } else {
        return intersection_points.front();
      }
    }();

    auto [best_angle, goal_angle_width] =
      world_model()->getLargestAttackGoalAngleRangeFromPoint(robot()->pose.pos);
    double angle_diff_deg =
      std::abs(getAngleDiff(getAngle(world_model()->ball().pos - robot()->pose.pos), best_angle)) *
      180.0 / M_PI;

    // ゴールが見えている && リダイレクト角度が45度以内
    bool redirect = goal_angle_width * 180.0 / M_PI > 10. && angle_diff_deg < 45.;

    if (redirect) {
      printTextOnRobot("RECEIVE::REDIRECT");
      receive_skill.setParameter("enable_redirect", true);
      receive_skill.setParameter("redirect_target", redirect_target);
      receive_skill.setParameter("policy", std::string("closest"));
      receive_skill.setParameter("redirect_kick_power", 0.4);
    } else {
      printTextOnRobot("RECEIVE::NORMAL");
      receive_skill.setParameter("enable_redirect", false);
      receive_skill.setParameter("policy", std::string("closest"));
      receive_skill.setParameter("dribble_power", 0.0);
      receive_skill.setParameter("enable_software_bumper", false);
    }
    return receive_skill.run();
  });

  addTransition(
    static_cast<int>(AttackerState::ENTRY_POINT), static_cast<int>(AttackerState::KICK),
    [this]() -> bool { return true; });

  addTransition(
    static_cast<int>(AttackerState::KICK), static_cast<int>(AttackerState::ENTRY_POINT),
    [this]() -> bool {
      // ボールが動いていても、ロボットがボールに近い場合はKICKを継続する。
      // ボールが動いているだけでKICKを抜けると、KICK→ENTRY_POINT→KICKの発振が起きるため。
      return world_model()->ball().isMoving(MOVING_BALL_VELOCITY) &&
             robot()->getDistance(world_model()->ball().pos) > BALL_CONTROL_DISTANCE;
    });

  // KICK状態でボールに触れずKICK_TIMEOUT_SEC以上経過した場合、ENTRY_POINTへ戻って再割当を待つ
  addTransition(
    static_cast<int>(AttackerState::KICK), static_cast<int>(AttackerState::ENTRY_POINT),
    [this]() -> bool {
      using std::chrono_literals::operator""s;
      if (!in_kick_state) return false;
      // 安価な条件を先に評価して早期リターン
      bool no_contact = robot()->ball_contact.getContactDuration() <
                        std::chrono::duration<double>(KICK_NO_CONTACT_THRESHOLD_SEC);
      if (!no_contact) return false;
      bool ball_stopped = world_model()->ball().isStopped(KICK_BALL_STOPPED_VEL);
      if (!ball_stopped) return false;
      auto elapsed = std::chrono::steady_clock::now() - kick_state_entry_time;
      return elapsed > std::chrono::duration<double>(KICK_TIMEOUT_SEC);
    });

  addStateFunction(static_cast<int>(AttackerState::KICK), [this]() -> Status {
    // KICK状態進入時にタイマー開始
    if (!in_kick_state) {
      kick_state_entry_time = std::chrono::steady_clock::now();
      in_kick_state = true;
    }

    // 味方ペナルティエリア内のボールには接近しない（GKに委ねる）
    if (world_model()->point_checker.isFriendPenaltyArea(world_model()->ball().pos, 0.15)) {
      command->lookAtBall();
      command->addPlanningFactor("attacker", "WAIT_BALL_EXIT_FRIEND_PA");
      return Status::RUNNING;
    }

    // 相手ペナルティエリア近辺での速度制限（ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA防止）
    if (world_model()->point_checker.isEnemyPenaltyArea(robot()->pose.pos, 0.5)) {
      command->setMaxVelocity("near_their_penalty_area", 1.5);
    }

    double goal_angle_width = evaluateGoalAngle(world_model()->ball().pos);

    // パスは pass_target_id がある場合のみ検討
    if (world_model()->getMsg().game_analysis.pass_target_id >= 0) {
      auto target_id = static_cast<uint8_t>(world_model()->getMsg().game_analysis.pass_target_id);
      if (target_id != robot()->id) {
        // pass_scores からスコアを参照して低品質パスを拒否
        const auto & pass_scores = world_model()->getMsg().game_analysis.pass_scores;
        double target_score = 0.0;
        for (const auto & s : pass_scores) {
          if (s.id == target_id) {
            target_score = s.value;
            break;
          }
        }
        if (target_score >= MIN_PASS_SCORE_ATTACKER) {
          pass_receiver_id = target_id;
          kick_target = world_model()->getOurRobot(target_id)->pose.pos;
        } else {
          pass_receiver_id = std::nullopt;
        }
      } else {
        pass_receiver_id = std::nullopt;
      }
    } else {
      // 未選択時はパスしない
      pass_receiver_id = std::nullopt;
    }

    double x_diff_with_their_goal =
      std::abs(world_model()->getAttackGoalCenter().x() - world_model()->ball().pos.x());

    using boost::math::constants::degree;
    if (goal_angle_width > GOAL_ANGLE_THRESHOLD_RAD) {
      // GOAL_KICK
      printTextOnRobot("KICK::GOAL_KICK");
      goal_kick_skill.setParameter("キック角度の最低要求精度[deg]", GOAL_ANGLE_THRESHOLD_DEG);
      goal_kick_skill.setParameter("use_target_kick_speed", true);
      goal_kick_skill.setParameter("target_kick_speed", 6.0);
      goal_kick_skill.setParameter("dribble_power", 0.2);
      return goal_kick_skill.run();
    }else if (pass_receiver_id.has_value()) {
      // STANDARD_PASS
      printTextOnRobot("KICK::STANDARD_PASS");
      kick_target = world_model()->getOurRobot(pass_receiver_id.value())->pose.pos;
      kick_skill.setParameter("target", kick_target);
      configurePassKick(kick_target, kick_skill);
      return kick_skill.run();
    }else if (goal_angle_width > LOW_CHANCE_GOAL_ANGLE_THRESHOLD_DEG * M_PI / 180.0) {
      // LOW_CHANCE_GOAL_KICK
      printTextOnRobot("KICK::LOW_CHANCE_GOAL_KICK");
      return goal_kick_skill.run();
    } else if (
      robot()->getDistance(world_model()->ball().pos) < BALL_CONTROL_DISTANCE &&
      x_diff_with_their_goal >= world_model()->fieldSize().x() * 0.5) {
      // MOVE_BALL_TO_OPPONENT_HALF
      printTextOnRobot("KICK::MOVE_BALL_TO_OPPONENT_HALF");
      kick_skill.setParameter("target", world_model()->getAttackGoalCenter());
      kick_skill.setParameter("chip_kick", true);
      kick_skill.setParameter("use_target_chip_distance", true);
      kick_skill.setParameter("target_chip_distance", CHIP_KICK_DISTANCE);
      command->disableBallAvoidance();
      return kick_skill.run();
    } else {
      // FINAL_GUARD: ゴール角度が不十分なのでチップキックで前方クリア
      printTextOnRobot("KICK::FINAL_GUARD");
      kick_skill.setParameter("target", world_model()->getAttackGoalCenter());
      kick_skill.setParameter("chip_kick", true);
      kick_skill.setParameter("use_target_chip_distance", true);
      kick_skill.setParameter("target_chip_distance", CHIP_KICK_DISTANCE);
      command->disableBallAvoidance();
      return kick_skill.run();
    }
  });
}

void Attacker::onPostUpdate()
{
  if (over_dribble.distance > OVER_DRIBBLE_DISTANCE_THRESHOLD) {
    RCLCPP_INFO(rclcpp::get_logger("Attacker"), "オーバードリブル[m]: %f", over_dribble.distance);
  } else {
    command->setOmegaLimit(10.0);
  }
}

void Attacker::configurePassKick(const Point & target, Kick & kick_skill)
{
  auto pass_analysis = getPassAnalysis(
    world_model()->ball().pos, target, world_model()->theirs().robotsWhere().available().get());

  if (pass_analysis.need_chip || shouldUseChipKick(target)) {
    kick_skill.setParameter("chip_kick", true);
    kick_skill.setParameter("with_dribble", true);
    kick_skill.setParameter("dribble_power", 0.7);
    if (pass_analysis.need_chip) {
      kick_skill.setParameter("use_target_chip_distance", true);
      kick_skill.setParameter("target_chip_distance", pass_analysis.required_chip_distance + 0.2);
    } else {
      kick_skill.setParameter("kick_power", 0.9);
    }
  } else {
    kick_skill.setParameter("chip_kick", false);
    kick_skill.setParameter("use_target_kick_speed", true);
    kick_skill.setParameter(
      "target_kick_speed", std::clamp((world_model()->ball().pos - target).norm(), 2.0, 4.0));
    kick_skill.setParameter("dribble_power", 0.0);
  }
}

bool Attacker::shouldUseChipKick(const Point & target)
{
  Segment kick_line{world_model()->ball().pos, target};
  if (
    auto nearest_enemy = world_model()->getNearestRobotWithDistanceFromSegment(
      kick_line, world_model()->theirs().robotsWhere().available().get());
    nearest_enemy.has_value()) {
    return nearest_enemy->distance < PASS_OBSTACLE_DISTANCE &&
           nearest_enemy->robot->getDistance(world_model()->ball().pos) < ENEMY_NEAR_BALL_DISTANCE;
  }
  return false;
}

double Attacker::evaluateGoalAngle(const Point & position)
{
  auto [best_angle, goal_angle_width] =
    world_model()->getLargestAttackGoalAngleRangeFromPoint(position);
  return goal_angle_width;
}

double Attacker::calculatePassScore(const Point & target)
{
  double score = 1.0;

  // パス距離の評価（1.5m〜4mが最適、1.0m以下は大幅減点）
  const double pass_distance = (target - world_model()->ball().pos).norm();
  if (pass_distance < 1.0) {
    // 1.0m以下の超近距離パスは大幅減点
    score *= pass_distance;  // 0.5mなら0.5倍、0.3mなら0.3倍
  } else {
    // 1.0m以上は距離に応じてボーナス（最大+2.0）
    score += std::clamp((pass_distance - 1.0) * 0.5, 0.0, 2.0);
  }

  // パス先のゴールチャンスが大きい場合はスコアを上げる(30度以上で最大0.5上昇)
  double goal_angle_width = evaluateGoalAngle(target);
  score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);

  // 自ゴールから遠いほうが良い（reverse_attack時は攻撃対象ゴールの反対側）
  auto defense_goal_posts = world_model()->isPracticeReverseAttack()
                              ? world_model()->getTheirGoalPosts()
                              : world_model()->getOurGoalPosts();
  auto [best_angle, own_goal_angle_width] =
    world_model()->getLargestGoalAngleRangeFromPoint(target, defense_goal_posts, {});
  score -= std::clamp(own_goal_angle_width / (M_PI / 12.), 0.0, 0.5);

  // 攻撃対象ゴールに近いときはスコアを上げる
  double normed_distance_to_their_goal = ((target - world_model()->getAttackGoalCenter()).norm() -
                                          (world_model()->fieldSize().x() * 0.5)) /
                                         (world_model()->fieldSize().x() * 0.5);
  score *= (1.0 - normed_distance_to_their_goal);

  // パスがブロックされているかチェックし、ブロックされていたら諦め、近くにいるときはスコアを下げる
  Segment ball_to_target{world_model()->ball().pos, target};
  const auto enemy_robots = world_model()->theirs().robotsWhere().available().get();
  if (
    auto nearest_enemy =
      world_model()->getNearestRobotWithDistanceFromSegment(ball_to_target, enemy_robots);
    nearest_enemy) {
    if (
      nearest_enemy->robot->getDistance(world_model()->ball().pos) > BALL_CONTROL_DISTANCE &&
      nearest_enemy->distance < PASS_OBSTACLE_DISTANCE) {
      return 0.0;  // パスがブロックされている
    }
    score *= std::clamp(nearest_enemy->distance / 2.0, 0.0, 1.0);
  }

  return score;
}
}  // namespace crane::skills
