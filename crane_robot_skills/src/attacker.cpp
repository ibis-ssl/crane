// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/ddps.hpp>
#include <crane_physics/pass.hpp>
#include <crane_robot_skills/attacker.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crane::skills
{
namespace
{
constexpr double GOAL_ANGLE_THRESHOLD_DEG = 5.0;
constexpr double GOAL_ANGLE_THRESHOLD_RAD = GOAL_ANGLE_THRESHOLD_DEG * M_PI / 180.0;
constexpr double LOW_CHANCE_GOAL_ANGLE_THRESHOLD_DEG = 2.0;
constexpr double PASS_OBSTACLE_DISTANCE = 0.4;
constexpr double BALL_CONTROL_DISTANCE = 1.0;
constexpr double CHIP_KICK_DISTANCE = 2.0;
constexpr double MOVING_BALL_VELOCITY = 1.0;
constexpr double ENEMY_NEAR_BALL_DISTANCE = 2.0;
}  // namespace
void Attacker::initialize()
{
  setParameter("moving_ball_velocity", MOVING_BALL_VELOCITY);

  receive_skill.setParameter("policy", std::string("closest"));
  addStateFunction(AttackerState::ENTRY_POINT, [this]() -> Status {
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

  setPostUpdateFunction([this]() {
    over_dribble.update(robot()->pose.pos, world_model()->ball().pos);
    if (over_dribble.distance > 0.5) {
      RCLCPP_INFO(rclcpp::get_logger("Attacker"), "オーバードリブル[m]: %f", over_dribble.distance);
      command->stopHere();
    } else {
      command->setOmegaLimit(10.0);
    }
  });

  // "ENTRY_POINT"のstate functionは実行されない（skill_base.hppのStateMachine::update参照）
  // ので自分への遷移関数で初期化処理を実装
  addTransition(AttackerState::ENTRY_POINT, AttackerState::ENTRY_POINT, [this]() -> bool {
    pass_receiver_id = std::nullopt;
    receive_skill.clearVisualizer();
    kick_skill.clearVisualizer();
    goal_kick_skill.clearVisualizer();
    return false;
  });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::FORCED_PASS, [this]() -> bool {
    // セットプレイのときは強制パス
    auto game_command = world_model()->getMsg().play_situation.command.value;
    // ボールの停止条件は、INPLAY切り替わりの遅延対策
    if (
      (game_command == crane_msgs::msg::PlaySituation::OUR_DIRECT_FREE ||
       game_command == crane_msgs::msg::PlaySituation::OUR_KICKOFF_START) &&
      world_model()->ball().isStopped()) {
      // 上位層のパス先が未選択なら強制パスに入らない
      if (world_model()->getMsg().game_analysis.pass_target_id < 0) {
        return false;
      }
      forced_pass_receiver_id =
        static_cast<int>(world_model()->getMsg().game_analysis.pass_target_id);
      pass_receiver_id = forced_pass_receiver_id;
      auto receiver = world_model()->getOurRobot(pass_receiver_id.value());
      kick_skill.setParameter("target", receiver->pose.pos);
      return true;
    } else {
      return false;
    }
  });

  // ----- ダブルタッチ防止の為、FORCED_PASS -> ENTRY_POINT の状態遷移は設けない ------- //

  addStateFunction(AttackerState::FORCED_PASS, [this]() -> Status {
    // パス
    command->disableBallAvoidance();
    command->setMaxVelocity("AttackerState::FORCED_PASS", 2.0);
    if (pass_receiver_id) {
      auto pass_receiver_pos = world_model()->getOurRobot(pass_receiver_id.value())->pose.pos;
      if (pass_receiver_pos.x() * world_model()->getOurSideSign() > 0.) {
        // 自陣にいるときは強制的に敵ゴールを目標に設定
        kick_target = world_model()->getTheirGoalCenter();
      } else {
        // 敵陣に適当なロボットがいる場合はパス
        kick_target = pass_receiver_pos;
      }
    } else {
      kick_target = world_model()->getTheirGoalCenter();
    }
    kick_skill.setParameter("target", kick_target);
    Segment kick_line{world_model()->ball().pos, kick_target};
    configurePassKick(kick_target, kick_skill);
    kick_skill.run();

    return Status::RUNNING;
  });

  addTransition(AttackerState::ENTRY_POINT, AttackerState::RECEIVE, [this]() -> bool {
    // ボールが遠くにいる/動いている/自分に向かってきている
    if (
      robot()->getDistance(world_model()->ball().pos) > BALL_CONTROL_DISTANCE &&
      world_model()->ball().isMoving(MOVING_BALL_VELOCITY) &&
      world_model()->ball().isMovingTowards(robot()->pose.pos)) {
      return true;
    } else {
      return false;
    }
  });

  addTransition(AttackerState::RECEIVE, AttackerState::ENTRY_POINT, [this]() -> bool {
    using std::chrono_literals::operator""s;
    if (world_model()->ball().isStopped(MOVING_BALL_VELOCITY)) {
      // ボールが止まっている
      return true;
    } else if (world_model()->ball().isMovingAwayFrom(robot()->pose.pos)) {
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
        -world_model()->fieldSize().y() * 0.5;
      goal_line.second << world_model()->getTheirGoalCenter().x(),
        world_model()->fieldSize().y() * 0.5;
      if (auto intersection_points = getIntersections(shoot_line, goal_line);
          intersection_points.empty()) {
        return world_model()->getTheirGoalCenter();
      } else {
        return intersection_points.front();
      }
    }();

    double goal_angle_width = evaluateGoalAngle(robot()->pose.pos);
    auto [best_angle, _] = world_model()->getLargestGoalAngleRangeFromPoint(robot()->pose.pos);
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

  addTransition(AttackerState::ENTRY_POINT, AttackerState::KICK, [this]() -> bool { return true; });

  addTransition(AttackerState::KICK, AttackerState::ENTRY_POINT, [this]() -> bool {
    return world_model()->ball().isMoving(MOVING_BALL_VELOCITY);
  });

  addStateFunction(AttackerState::KICK, [this]() -> Status {
    double goal_angle_width = evaluateGoalAngle(world_model()->ball().pos);

    // パスは pass_target_id がある場合のみ検討
    if (world_model()->getMsg().game_analysis.pass_target_id >= 0) {
      auto target_id = static_cast<uint8_t>(world_model()->getMsg().game_analysis.pass_target_id);
      if (target_id != robot()->id) {
        pass_receiver_id = target_id;
        kick_target = world_model()->getOurRobot(target_id)->pose.pos;
      } else {
        pass_receiver_id = std::nullopt;
      }
    } else {
      // 未選択時はパスしない
      pass_receiver_id = std::nullopt;
    }

    double x_diff_with_their_goal =
      std::abs(world_model()->getTheirGoalCenter().x() - world_model()->ball().pos.x());

    using boost::math::constants::degree;
    if (goal_angle_width > GOAL_ANGLE_THRESHOLD_RAD) {
      // GOAL_KICK
      printTextOnRobot("KICK::GOAL_KICK");
      goal_kick_skill.setParameter("キック角度の最低要求精度[deg]", GOAL_ANGLE_THRESHOLD_DEG);
      goal_kick_skill.setParameter("use_target_kick_speed", true);
      goal_kick_skill.setParameter("target_kick_speed", 6.0);
      goal_kick_skill.setParameter("dribble_power", 0.2);
      return goal_kick_skill.run();
    } else if (pass_receiver_id.has_value()) {
      // STANDARD_PASS
      printTextOnRobot("KICK::STANDARD_PASS");
      kick_target = world_model()->getOurRobot(pass_receiver_id.value())->pose.pos;
      kick_skill.setParameter("target", kick_target);
      configurePassKick(kick_target, kick_skill);
      return kick_skill.run();
    } else if (goal_angle_width > LOW_CHANCE_GOAL_ANGLE_THRESHOLD_DEG * M_PI / 180.0) {
      // LOW_CHANCE_GOAL_KICK
      printTextOnRobot("KICK::LOW_CHANCE_GOAL_KICK");
      return goal_kick_skill.run();
    } else if (
      robot()->getDistance(world_model()->ball().pos) < BALL_CONTROL_DISTANCE &&
      x_diff_with_their_goal >= world_model()->fieldSize().x() * 0.5) {
      // MOVE_BALL_TO_OPPONENT_HALF
      printTextOnRobot("KICK::MOVE_BALL_TO_OPPONENT_HALF");
      kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
      kick_skill.setParameter("chip_kick", true);
      kick_skill.setParameter("use_target_chip_distance", true);
      kick_skill.setParameter("target_chip_distance", CHIP_KICK_DISTANCE);
      command->disableBallAvoidance();
      return kick_skill.run();
    } else {
      // FINAL_GUARD
      printTextOnRobot("KICK::FINAL_GUARD");
      return goal_kick_skill.run();
    }
  });
}

auto Attacker::OverDribbleInfo::update(const Point & current_position, const Point & ball_position)
  -> void
{
  if ((current_position - ball_position).norm() < 0.12) {
    if (not detected) {
      distance = 0.0;
    } else {
      distance += (current_position - previous_position).norm();
    }
    detected = true;
    previous_position = current_position;
  } else {
    detected = false;
    distance = 0.0;
  }
}

void Attacker::configurePassKick(const Point & target, Kick & kick_skill)
{
  auto pass_analysis = getPassAnalysis(
    world_model()->ball().pos, target, world_model()->theirs().getAvailableRobots());

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
  if (auto nearest_enemy = world_model()->getNearestRobotWithDistanceFromSegment(
        kick_line, world_model()->theirs().getAvailableRobots());
      nearest_enemy.has_value()) {
    return nearest_enemy->distance < PASS_OBSTACLE_DISTANCE &&
           nearest_enemy->robot->getDistance(world_model()->ball().pos) < ENEMY_NEAR_BALL_DISTANCE;
  }
  return false;
}

double Attacker::evaluateGoalAngle(const Point & position)
{
  auto [best_angle, goal_angle_width] = world_model()->getLargestGoalAngleRangeFromPoint(position);
  return goal_angle_width;
}

bool Attacker::isPassBlocked(const Point & target)
{
  Segment ball_to_target{world_model()->ball().pos, target};
  const auto enemy_robots = world_model()->theirs().getAvailableRobots();

  if (auto nearest_enemy =
        world_model()->getNearestRobotWithDistanceFromSegment(ball_to_target, enemy_robots);
      nearest_enemy) {
    return nearest_enemy->robot->getDistance(world_model()->ball().pos) > BALL_CONTROL_DISTANCE &&
           nearest_enemy->distance < PASS_OBSTACLE_DISTANCE;
  }
  return false;
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

  // 自ゴールから遠いほうが良い
  auto [best_angle, own_goal_angle_width] =
    world_model()->getLargestGoalAngleRangeFromPoint(target, world_model()->getOurGoalPosts(), {});
  score -= std::clamp(own_goal_angle_width / (M_PI / 12.), 0.0, 0.5);

  // 敵ゴールに近いときはスコアを上げる
  double normed_distance_to_their_goal = ((target - world_model()->getTheirGoalCenter()).norm() -
                                          (world_model()->fieldSize().x() * 0.5)) /
                                         (world_model()->fieldSize().x() * 0.5);
  score *= (1.0 - normed_distance_to_their_goal);

  // パスがブロックされている場合は諦める
  if (isPassBlocked(target)) {
    return 0.0;
  }

  // パスラインに敵がいるときはスコアを下げる
  Segment ball_to_target{world_model()->ball().pos, target};
  const auto enemy_robots = world_model()->theirs().getAvailableRobots();
  if (auto nearest_enemy =
        world_model()->getNearestRobotWithDistanceFromSegment(ball_to_target, enemy_robots);
      nearest_enemy) {
    score *= std::clamp(nearest_enemy->distance / 2.0, 0.0, 1.0);
  }

  return score;
}
}  // namespace crane::skills
