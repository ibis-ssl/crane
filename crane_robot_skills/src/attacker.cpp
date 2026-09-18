// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/ddps.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/pass_plan.hpp>
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
constexpr double GOAL_ANGLE_THRESHOLD_RAD = deg2rad(GOAL_ANGLE_THRESHOLD_DEG);
constexpr double LOW_CHANCE_GOAL_ANGLE_THRESHOLD_DEG = 0.5;
constexpr double BALL_CONTROL_DISTANCE = 1.0;
constexpr double CHIP_KICK_DISTANCE = 2.0;
constexpr double MOVING_BALL_VELOCITY = 1.0;
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
        deg2rad(10.0), robot()->pose.pos, world_model(), visualizer);
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
    double angle_diff_deg = rad2deg(
      std::abs(getAngleDiff(getAngle(world_model()->ball().pos - robot()->pose.pos), best_angle)));

    // ゴールが見えている && リダイレクト角度が45度以内
    bool redirect = rad2deg(goal_angle_width) > 10. && angle_diff_deg < 45.;

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

    const auto & pass_plan = world_model()->getMsg().game_analysis.pass_plan;
    if (
      isUsablePassPlan(pass_plan, *world_model()) &&
      pass_plan.state == crane_msgs::msg::PassPlan::STATE_PLANNING &&
      pass_plan.kicker_id == robot()->id && pass_plan.score >= MIN_PASS_SCORE_ATTACKER) {
      pass_receiver_id = static_cast<uint8_t>(pass_plan.receiver_id);
      kick_target = Point(pass_plan.receive_point.x, pass_plan.receive_point.y);
    } else {
      // 未選択時はパスしない
      pass_receiver_id = std::nullopt;
    }

    double x_diff_with_their_goal =
      std::abs(world_model()->getAttackGoalCenter().x() - world_model()->ball().pos.x());

    using boost::math::constants::degree;
    // KICK 状態でどの分岐を選んだかは printTextOnRobot（可視化専用）にしか出ず、
    // ログからは追えない。「パス計画を持っていたのにシュートを選んだ」のか
    // 「そもそも計画が無かった」のかを後から切り分けられるようにする。
    //
    // 特に FINAL_GUARD はゴールが開いていなくても 6.0 m/s のストレートを撃つため、
    // 実測ではパスと見分けがつかない速いボールが飛ぶ。どちらが飛んだのかを
    // ログだけで判定できないと、シナリオ検証の失敗原因を取り違える。
    auto log_branch = [&](const char * branch) {
      static auto log_clock = rclcpp::Clock(RCL_STEADY_TIME);
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("Attacker"), log_clock, 1000,
        "KICK分岐: robot=%d %s ゴール可視角=%.2f°(シュート閾値 %.1f°) パス計画=%s",
        static_cast<int>(robot()->id), branch, goal_angle_width / degree<double>(),
        GOAL_ANGLE_THRESHOLD_DEG, pass_receiver_id.has_value() ? "あり" : "なし");
    };
    if (goal_angle_width > GOAL_ANGLE_THRESHOLD_RAD) {
      // GOAL_KICK
      log_branch("GOAL_KICK");
      printTextOnRobot("KICK::GOAL_KICK");
      goal_kick_skill.setParameter("キック角度の最低要求精度[deg]", GOAL_ANGLE_THRESHOLD_DEG);
      goal_kick_skill.setParameter("use_target_kick_speed", true);
      goal_kick_skill.setParameter("target_kick_speed", 6.0);
      goal_kick_skill.setParameter("dribble_power", 0.2);
      return goal_kick_skill.run();
    } else if (pass_receiver_id.has_value()) {
      // STANDARD_PASS
      log_branch("STANDARD_PASS");
      printTextOnRobot("KICK::STANDARD_PASS");
      kick_skill.setParameter("target", kick_target);
      kick_skill.setParameter("chip_kick", false);
      kick_skill.setParameter("use_target_chip_distance", false);
      kick_skill.setParameter("use_target_kick_speed", true);
      kick_skill.setParameter("target_kick_speed", static_cast<double>(pass_plan.kick_speed));
      kick_skill.setParameter("with_dribble", false);
      kick_skill.setParameter("dribble_power", 0.0);
      return kick_skill.run();
    } else if (goal_angle_width > deg2rad(LOW_CHANCE_GOAL_ANGLE_THRESHOLD_DEG)) {
      // LOW_CHANCE_GOAL_KICK
      log_branch("LOW_CHANCE_GOAL_KICK");
      printTextOnRobot("KICK::LOW_CHANCE_GOAL_KICK");
      return goal_kick_skill.run();
    } else if (
      robot()->getDistance(world_model()->ball().pos) < BALL_CONTROL_DISTANCE &&
      x_diff_with_their_goal >= world_model()->fieldSize().x() * 0.5) {
      // MOVE_BALL_TO_OPPONENT_HALF
      log_branch("MOVE_BALL_TO_OPPONENT_HALF");
      printTextOnRobot("KICK::MOVE_BALL_TO_OPPONENT_HALF");
      kick_skill.setParameter("target", world_model()->getAttackGoalCenter());
      kick_skill.setParameter("chip_kick", true);
      kick_skill.setParameter("use_target_chip_distance", true);
      kick_skill.setParameter("target_chip_distance", CHIP_KICK_DISTANCE);
      command->disableBallAvoidance();
      return kick_skill.run();
    } else {
      // FINAL_GUARD: ゴール角度が不十分でも強ストレートでクリア
      // チップキックはGK越えで直接ゴールに入るとファウルになるため使用不可
      log_branch("FINAL_GUARD");
      printTextOnRobot("KICK::FINAL_GUARD");
      kick_skill.setParameter("target", world_model()->getAttackGoalCenter());
      kick_skill.setParameter("chip_kick", false);
      kick_skill.setParameter("use_target_kick_speed", true);
      kick_skill.setParameter("target_kick_speed", 6.0);
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

double Attacker::evaluateGoalAngle(const Point & position)
{
  auto [best_angle, goal_angle_width] =
    world_model()->getLargestAttackGoalAngleRangeFromPoint(position);
  return goal_angle_width;
}
}  // namespace crane::skills
