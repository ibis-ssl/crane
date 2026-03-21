// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/robot_info.hpp>
#include <crane_physics/travel_time.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/functional/comparisons.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
namespace crane::skills
{

void Goalie::initialize()
{
  setParameter("run_inplay", true);
  setParameter("block_distance", 0.5);
  setParameter("emit_force_timeout", 3.0);
  setParameter("emit_enemy_distance_threshold", 0.5);
  setParameter("emit_ball_speed_threshold", 0.5);
}

Status Goalie::update()
{
  auto situation = world_model()->getMsg().play_situation.command.value;
  if (getParameter<bool>("run_inplay")) {
    situation = crane_msgs::msg::PlaySituation::INPLAY;
  }
  phase = "";
  command->withDribble(0.0);

  switch (situation) {
    case crane_msgs::msg::PlaySituation::HALT:
      phase = "HALT, stop here";
      command->stopHere();
      break;
    case crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION:
      [[fallthrough]];
    case crane_msgs::msg::PlaySituation::THEIR_PENALTY_START:
      phase = "ペナルティキック";
      inplay(false);
      break;
    default: {
      if (
        world_model()->getMsg().play_situation.referee_raw.command.value ==
        robocup_ssl_msgs::msg::RefereeCommand::STOP) {
        // STOPのときにはボールを排出しない
        inplay(false);
      } else {
        inplay(true);
      }
      break;
    }
  }

  visualizer->drawDebugLabel(robot()->pose.pos, phase);
  return Status::RUNNING;
}

void Goalie::emitBallFromPenaltyArea()
{
  Point ball = world_model()->ball().pos;
  // パスできるロボットのリストアップ
  auto passable_robot_list =
    world_model()->ours().robotsWhere().available().excludeId(command->getMsg().robot_id).get();
  std::erase_if(passable_robot_list, [&](const RobotInfo::SharedPtr & r) {
    if (
      std::abs(r->pose.pos.x() - world_model()->getOurGoalCenter().x()) <
      world_model()->getDefenseHeight()) {
      // ゴールラインに近いロボットは除外
      return true;
    } else if (r->getDistance(world_model()->ball().pos) < 0.5) {
      // ボールに近いロボットは除外
      return true;
    } else {
      return false;
    }
  });

  Point pass_target = world_model()->getTheirGoalCenter();

  visualizer->drawLine(ball, pass_target, "blue", 10);

  kick_skill.setParameter("target", pass_target);
  kick_skill.setParameter("kick_power", 1.0);
  kick_skill.setParameter("chip_kick", true);
  kick_skill.run();
  // 追加のコマンド
  command->disableGoalAreaAvoidance();
}

void Goalie::inplay(bool enable_emit)
{
  auto goals = world_model()->getOurGoalPosts();
  const auto goal_center = world_model()->getOurGoalCenter();
  const auto & ball = world_model()->ball();
  const double ball_vel_norm = ball.vel.norm();
  const double goal_min_y = std::min(goals.first.y(), goals.second.y());
  const double goal_max_y = std::max(goals.first.y(), goals.second.y());
  // シュートチェック
  Segment goal_line(goals.first, goals.second);
  Segment ball_line = ball.getTrajectorySegmentByDistance(10.0);
  auto intersections = getIntersections(ball_line, Segment{goals.first, goals.second});
  command->setTerminalVelocity(0.0).disableGoalAreaAvoidance().disableBallAvoidance();

  // 改善1: ニアミス判定（ボール軌道がゴールラインと交差しなくても近い場合はブロック）
  // intersections.empty()を先頭にして短絡評価でbg::distance計算を省略
  bool near_miss_shot =
    intersections.empty() && bg::distance(ball_line, goal_line) < 1.0 && ball_vel_norm > 0.5;

  if ((not intersections.empty() || near_miss_shot) && ball_vel_norm > 0.3) {
    // シュートブロック
    phase = "シュートブロック";
    prev_wait_point = std::nullopt;
    auto result = ball.getClosestPointToTrajectory(command->getRobot()->pose.pos);
    auto target = [&]() -> Point {
      if (not intersections.empty()) {
        if (not world_model()->point_checker.isFieldInside(result.closest_point)) {
          return clampXToGoalLine(intersections.front(), 0.15);
        } else {
          return result.closest_point;
        }
      } else {
        // ニアミス: ボール速度ベクトルからゴールラインのX位置でのy座標を予測
        double clamped_y;
        if (std::abs(ball.vel.x()) > 0.01f) {
          double t = (goal_center.x() - ball.pos.x()) / ball.vel.x();
          clamped_y = std::clamp(ball.pos.y() + t * ball.vel.y(), goal_min_y, goal_max_y);
        } else {
          clamped_y = std::clamp(static_cast<double>(ball.pos.y()), goal_min_y, goal_max_y);
        }
        return clampXToGoalLine(Point(goal_center.x(), clamped_y), 0.15);
      }
    }();

    command->setTargetPosition(target).lookAtBallFrom(target);
  } else {
    // ボールがフィールド外かつ自陣側（ゴール内）かどうか
    bool ball_in_our_goal = not world_model()->point_checker.isFieldInside(ball.pos) &&
                            std::signbit(ball.pos.x()) == std::signbit(goal_center.x());
    const double emit_ball_speed = getParameter<double>("emit_ball_speed_threshold");
    const double emit_enemy_dist = getParameter<double>("emit_enemy_distance_threshold");
    const double emit_timeout = getParameter<double>("emit_force_timeout");

    // 敵が近くにいる場合は待ち受け（タイムアウト後は強制排出）
    bool ball_in_penalty =
      world_model()->ball().isStopped(emit_ball_speed) && enable_emit &&
      (world_model()->point_checker.isFriendPenaltyArea(ball.pos) || ball_in_our_goal);
    bool should_emit = ball_in_penalty;

    if (ball_in_penalty && not ball_in_our_goal) {
      // タイマー管理（ゴール内ボールは敵チェックなし・タイムアウト不要のため対象外）
      auto now = std::chrono::steady_clock::now();
      if (!ball_in_penalty_since_) {
        ball_in_penalty_since_ = now;
      }

      auto enemies = world_model()->theirs().robotsWhere().available().get();
      bool enemy_near = std::any_of(enemies.begin(), enemies.end(), [&](const auto & e) {
        return e->getDistance(ball.pos) < emit_enemy_dist;
      });

      bool force_emit =
        std::chrono::duration<double>(now - *ball_in_penalty_since_).count() > emit_timeout;

      if (enemy_near && !force_emit) {
        should_emit = false;
        phase = "排出危険→待ち受け";
      } else if (force_emit) {
        phase = "排出強制(タイムアウト)";
      }
    } else {
      ball_in_penalty_since_ = std::nullopt;
    }

    if (should_emit) {
      // ボールが止まっていて，味方ペナルティエリア内にあり，敵が近くにいないときは排出
      if (phase.empty()) {
        phase = "ボール排出";
      }
      emitBallFromPenaltyArea();
    } else if (  // 改善2: 緊急接近ブロック（シュートブロック条件を満たさないゴール接近ボール）
      ball.isMovingTowards(goal_center, 60.0) && ball_vel_norm > 1.0 &&
      std::abs(goal_center.x() - ball.pos.x()) < 2.0) {
      phase = "緊急接近ブロック";
      prev_wait_point = std::nullopt;
      double target_y = std::clamp(static_cast<double>(ball.pos.y()), goal_min_y, goal_max_y);
      Point target = clampXToGoalLine(Point(goal_center.x(), target_y), 0.15);
      command->setTargetPosition(target).lookAtBallFrom(target);
    } else {
      const double BLOCK_DIST = getParameter<double>("block_distance");
      if (phase.empty()) {
        phase += "ボールを待ち受ける";
      }
      // デフォルト位置設定
      command->setTargetPosition(goal_center * 0.9).lookAt(Point(0, 0));
      if (std::signbit(world_model()->ball().pos.x()) == std::signbit(world_model()->goal().x())) {
        phase += " (自コート警戒モード)";
        Segment ball_prediction_4s = ball.getTrajectorySegmentByTime(4.0);
        auto available_enemies = world_model()->theirs().robotsWhere().available().get();
        auto candidates =
          available_enemies | ranges::views::filter([&](const auto & enemy) {
            Vector2 ball_to_enemy = (enemy->pose.pos - ball.pos).normalized();
            Vector2 ball_direction = ball.vel.normalized();
            // ボールの進行方向のロボットのみ反映
            return ball_to_enemy.dot(ball_direction) > 0.0;
          }) |
          ranges::views::transform([&](const auto & enemy) {
            return std::make_pair(enemy, bg::distance(enemy->pose.pos, ball_prediction_4s));
          }) |
          ranges::to<std::vector>();

        auto [next_their_attacker, distance] =
          [&]() -> std::pair<std::shared_ptr<RobotInfo>, double> {
          if (candidates.empty()) {
            return {nullptr, 1000000.0};
          }
          return *ranges::min_element(
            candidates, ranges::less{}, [](const auto & p) { return p.second; });
        }();

        if (not world_model()->point_checker.isFieldInside(ball.pos)) {
          // 改善A: フィールド外ボールはゴール幅内にクランプして直接ポジション
          phase += "(範囲外→ゴール直接守備)";
          double clamped_y = std::clamp(
            static_cast<double>(ball.pos.y()),
            static_cast<double>(std::min(goals.first.y(), goals.second.y())),
            static_cast<double>(std::max(goals.first.y(), goals.second.y())));
          Point wait_point = clampXToGoalLine(Point(goal_center.x(), clamped_y), 0.1);
          prev_wait_point = wait_point;
          command->setTargetPosition(wait_point).lookAtBallFrom(wait_point);
        } else {
          Point threat_point = world_model()->ball().pos;

          if (distance < 2.0) {
            phase += "(敵のパス先警戒モード)";
            auto result =
              getClosestPointAndDistance(ball_prediction_4s, next_their_attacker->pose.pos);

            // ボールが敵ロボットに届くまでに到達可能な最大限の前進守備を行う。
            // 前進ラインの始点をペナルティエリア境界に制限
            Point forward_start_point = result.closest_point;
            Point goalie_pos = command->getRobot()->pose.pos;
            Segment tentative_line(result.closest_point, goalie_pos);
            auto penalty_intersection =
              world_model()->getIntersectionOurPenaltyArea(tentative_line, 0.0, 0.0);
            if (
              penalty_intersection &&
              not world_model()->point_checker.isFriendPenaltyArea(result.closest_point)) {
              // ペナルティエリア外から内側へのラインの場合、交点を始点とする
              forward_start_point = *penalty_intersection;
            }

            // 前進するライン
            Segment forward_line(forward_start_point, goalie_pos);

            // ボールが敵ロボットに最も近い点に到達するまでの時間
            auto estimated_ball_reach_time =
              ball.getTimeToReachClosestPointFrom(result.closest_point);
            if (not estimated_ball_reach_time) {
              threat_point = clampXToGoalLine(result.closest_point, 0.1);
            } else {
              threat_point = result.closest_point;

              // 敵の予想されるロボット位置とゴールの間の直線を20点に分割
              std::vector<Point> forward_points = getSeparatedPoints(forward_line, 20);
              for (int i = forward_points.size() - 1; i >= 0; --i) {
                // goalieが前進守備位置に到達する時間
                double travel_time =
                  getTravelTimeTrapezoidal(this->robot(), forward_points[i], 0.5, 2.0);
                if (estimated_ball_reach_time > travel_time) {
                  threat_point = forward_points[i];
                  break;
                }
              }
            }
          } else {
            phase += "(とりあえず0.5s先を警戒モード)";
            threat_point = ball.getPredictedPosition(0.5);
          }

          auto [weak_point, dist] = [&]() {
            if (
              auto other_robots = world_model()
                                    ->ours()
                                    .robotsWhere()
                                    .available()
                                    .excludeId(world_model()->getOurGoalieId())
                                    .get();
              not other_robots.empty()) {
              auto goal = world_model()->getLargestGoalAngleRangeFromPoint(
                threat_point, world_model()->getOurGoalPosts(), other_robots);
              Segment expected_ball_line(
                threat_point, threat_point + getNormVec(goal.center_angle) * 10);
              Segment goal_line(goals.first, goals.second);
              auto intersect_to_goal_line = getIntersections(expected_ball_line, goal_line);
              if (intersect_to_goal_line.empty()) {
                return std::make_pair(goal_center, BLOCK_DIST);
              } else {
                auto intersect_to_penalty_area =
                  world_model()->getIntersectionOurPenaltyArea(expected_ball_line, -0.8, -0.8);
                auto ratio = world_model()->getForwardDefenseRatio(expected_ball_line);
                if (not intersect_to_penalty_area || not ratio) {
                  return std::make_pair(goal_center, BLOCK_DIST);
                }
                // ペナルティエリアライン上にボールがあるときにペナルティエリア上まで前進すると
                // シュートをずらして打たれて決められてしまう?
                double dist =
                  bg::distance(intersect_to_goal_line.front(), *intersect_to_penalty_area) *
                  (*ratio);
                visualizer->drawLine(
                  intersect_to_goal_line.front(), *intersect_to_penalty_area, "red", 1.0);
                phase += "(前進守備量可変)";
                command->addPlanningFactor("goalie", "dist:" + std::to_string(dist));
                return std::make_pair(intersect_to_goal_line.front(), dist);
              }
            } else {
              return std::make_pair(goal_center, BLOCK_DIST);
            }
          }();
          Point wait_point = weak_point + (threat_point - weak_point).normalized() * dist;

          // 改善D: wait_pointのY座標をゴールポスト幅内にクランプ
          wait_point.y() = std::clamp(wait_point.y(), goal_min_y, goal_max_y);

          // 改善3: 適応的ローパスフィルタ（通常は強平滑化、ボール接近時は高応答）
          double smoothing_alpha = 0.9;  // デフォルト: 振動抑制優先
          if (ball.isMovingTowards(goal_center) && ball_vel_norm > 1.0) {
            smoothing_alpha = 0.3;  // ボール接近中: 応答性優先
          }
          if (prev_wait_point) {
            wait_point = *prev_wait_point * smoothing_alpha + wait_point * (1.0 - smoothing_alpha);
          }
          prev_wait_point = wait_point;

          // ゴール侵入防止: ゴールラインより前方にあることを保証
          wait_point = clampXToGoalLine(wait_point, 0.1);

          command->setTargetPosition(wait_point).lookAtBallFrom(wait_point);
        }
      }
    }
  }
}

auto Goalie::clampXToGoalLine(const Point & position, double margin) const -> Point
{
  Point clamped_position = position;
  const Point goal_center = world_model()->getOurGoalCenter();

  if (goal_center.x() > 0) {
    // 正のx側のゴール: ゴールラインより前方（小さいx）にクランプ
    clamped_position.x() = std::min(clamped_position.x(), goal_center.x() - margin);
  } else {
    // 負のx側のゴール: ゴールラインより前方（大きいx）にクランプ
    clamped_position.x() = std::max(clamped_position.x(), goal_center.x() + margin);
  }
  return clamped_position;
}
}  // namespace crane::skills
