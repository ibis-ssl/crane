// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/robot_info.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <range/v3/all.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
namespace crane::skills
{

void Goalie::initialize()
{
  setParameter("run_inplay", true);
  setParameter("block_distance", 0.5);
  setPreUpdateFunction([&]() { command->clearSkillStates(); });
}

Status Goalie::update()
{
  auto situation = world_model()->getMsg().play_situation.command.value;
  if (getParameter<bool>("run_inplay")) {
    situation = crane_msgs::msg::PlaySituation::OUR_INPLAY;
  }
  phase = "";

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
  auto passable_robot_list = world_model()->ours().getAvailableRobots(command->getMsg().robot_id);
  std::erase_if(passable_robot_list, [&](const RobotInfo::SharedPtr & r) {
    if (
      std::abs(r->pose.pos.x() - world_model()->getOurGoalCenter().x()) <
      world_model()->getDefenseHeight()) {
      // ゴールラインに近いロボットは除外
      return true;
    } else if (world_model()->getDistanceFromRobotToBall(r->getID()) < 0.5) {
      // ボールに近いロボットは除外
      return true;
    } else {
      return false;
    }
  });

  // Point pass_target = [&]() {
  //   if (not passable_robot_list.empty()) {
  //     auto robots_with_score = passable_robot_list |
  //       ranges::views::transform([&](const std::shared_ptr<RobotInfo> robot) {
  //         double score = 0.0;
  //         // 3m +-2m
  //         score += std::clamp(
  //           2 - std::abs(3.0 - robot->getDistance(world_model()->getOurGoalCenter())), 0.0, 2.0);
  //         // 真ん中の4mより外はスコアが下がる
  //         score -= std::clamp(std::abs(robot->pose.pos.y()) - 2.0, 0.0, 2.0);
  //         return std::make_pair(robot, score);
  //       }) |
  //       ranges::to<std::vector>();
  //
  //     auto max_score = ranges::max_element(
  //       robots_with_score, [](const auto & a, const auto & b) { return a.second < b.second; });
  //     if (max_score != robots_with_score.end()) {
  //       if (max_score->second > 0.5) {
  //         return max_score->first->pose.pos;
  //       } else {
  //         return world_model()->getTheirGoalCenter();
  //       }
  //     } else {
  //       return world_model()->getTheirGoalCenter();
  //     }
  //   } else {
  //     return world_model()->getTheirGoalCenter();
  //   }
  // }();
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
  // シュートチェック
  Segment goal_line(goals.first, goals.second);
  Segment ball_line = ball.getTrajectorySegmentByDistance(10.0);
  auto intersections = getIntersections(ball_line, Segment{goals.first, goals.second});
  command->setTerminalVelocity(0.0).disableGoalAreaAvoidance().disableBallAvoidance();

  if (not intersections.empty() && world_model()->ball().vel.norm() > 0.3f) {
    // シュートブロック
    phase = "シュートブロック";
    auto result = ball.getClosestPointToTrajectory(command->getRobot()->pose.pos);
    auto target = [&]() -> Point {
      if (not world_model()->point_checker.isFieldInside(result.closest_point)) {
        // フィールド外（=ゴール内）でのセーブは避ける
        // ゴールライン上の交点からフィールド内方向に安全なマージンを取る
        Point goal_line_intersection = intersections.front();
        Vector2 goal_to_field_direction = (Point(0, 0) - goal_center).normalized();
        const double GOAL_MARGIN = 0.15;  // ゴールラインから15cm前方
        return goal_line_intersection + goal_to_field_direction * GOAL_MARGIN;
      } else {
        return result.closest_point;
      }
    }();

    command->setTargetPosition(target).lookAtBallFrom(target);
    if (command->getRobot()->getDistance(target) > 0.05) {
      // command->clearMaxVelocityFactors().clearMaxAccelerationFactors();
      // command->setTerminalVelocity(2.0)
      //   .setMaxAcceleration("なりふり構わず爆加速", 5.0)
      //   .setMaxVelocity("なりふり構わず爆加速", 5.0);
    }
  } else {
    if (
      world_model()->ball().isStopped(0.2) &&
      world_model()->point_checker.isFriendPenaltyArea(ball.pos) && enable_emit) {
      // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
      phase = "ボール排出";
      emitBallFromPenaltyArea();
    } else {
      const double BLOCK_DIST = getParameter<double>("block_distance");
      phase += "ボールを待ち受ける";
      // デフォルト位置設定
      command->setTargetPosition(goal_center * 0.9).lookAt(Point(0, 0));
      if (std::signbit(world_model()->ball().pos.x()) == std::signbit(world_model()->goal().x())) {
        phase += " (自コート警戒モード)";
        Segment ball_prediction_4s = ball.getTrajectorySegmentByTime(4.0);
        auto available_enemies = world_model()->theirs().getAvailableRobots();
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

        Point goal_center_adjusted = goal_center;
        goal_center_adjusted << goals.first.x() - std::clamp(goals.first.x(), -0.1, 0.1), 0.0f;

        if (not world_model()->point_checker.isFieldInside(ball.pos)) {
          // TODO(HansRobo): 一番近いフィールド内の点を警戒するようにする
          phase += "(範囲外なので正面に構える)";
          command->setTargetPosition(goal_center_adjusted, 0.1).lookAt(Point(0, 0));
        } else {
          Point threat_point = world_model()->ball().pos;

          if (distance < 2.0) {
            phase += "(敵のパス先警戒モード)";
            auto result =
              getClosestPointAndDistance(ball_prediction_4s, next_their_attacker->pose.pos);

            // ボールが敵ロボットに届くまでに到達可能な最大限の前進守備を行う。
            // 前進するライン
            auto forward_line = Segment(
              result.closest_point, world_model()
                                      ->ours()
                                      .getAvailableRobots(world_model()->getOurGoalieId())
                                      .front()
                                      ->pose.pos);

            // ボールが敵ロボットに最も近い点に到達するまでの時間
            auto estimated_ball_reach_time =
              ball.getTimeToReachClosestPointFrom(result.closest_point);
            if (not estimated_ball_reach_time) {
              threat_point = result.closest_point;
            } else {
              threat_point = result.closest_point;

              // 敵の予想されるロボット位置とゴールの間の直線を10点に分割
              std::vector<Point> forward_pooints = getSeparatedPoints(forward_line, 10);
              for (int i = forward_pooints.size() - 1; i >= 0; --i) {
                // goalieが前進守備位置に到達する時間
                double travel_time =
                  getTravelTimeTrapezoidal(this->robot(), forward_pooints[i], 0.5, 2.0);
                if (estimated_ball_reach_time > travel_time) {
                  threat_point = forward_pooints[i];
                  break;
                }
              }
            }
          } else {
            phase += "(とりあえず0.5s先を警戒モード)";
            threat_point = ball.getPredictedPosition(0.5);
          }

          auto [weak_point, dist] = [&]() {
            if (auto other_robots =
                  world_model()->ours().getAvailableRobots(world_model()->getOurGoalieId());
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
                command->addStateFactor("goalie", "dist:" + std::to_string(dist));
                return std::make_pair(intersect_to_goal_line.front(), dist);
              }
            } else {
              return std::make_pair(goal_center, BLOCK_DIST);
            }
          }();
          Point wait_point = weak_point + (threat_point - weak_point).normalized() * dist;

          // ゴール侵入防止: ゴールラインより前方にあることを保証
          const double GOAL_LINE_MARGIN = 0.1;  // ゴールラインから10cm前方
          double goal_line_x = goal_center.x();
          if (goal_center.x() > 0) {
            // 正のx側のゴール
            wait_point.x() = std::min(wait_point.x(), goal_line_x - GOAL_LINE_MARGIN);
          } else {
            // 負のx側のゴール
            wait_point.x() = std::max(wait_point.x(), goal_line_x + GOAL_LINE_MARGIN);
          }

          command->setTargetPosition(wait_point).lookAtBallFrom(wait_point);
          if (command->getRobot()->getDistance(wait_point) > 0.03) {
            // command->clearMaxVelocityFactors().clearMaxAccelerationFactors();
            // command->setTerminalVelocity(2.0)
            //   .setMaxAcceleration("なりふり構わず爆加速", 5.0)
            //   .setMaxVelocity("なりふり構わず爆加速", 5.0);
          }
          // }
        }
      }
    }
  }
}
}  // namespace crane::skills
