// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/goalie.hpp>

#include <crane_basics/robot_info.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/ball_model.hpp>

#include <robocup_ssl_msgs/msg/referee.hpp>
namespace crane::skills
{

void Goalie::initialize()
{
  setParameter("run_inplay", true);
  setParameter("block_distance", 0.5);
  setParameter("robot_acc_for_prediction", 2.0);
  setParameter("robot_max_vel_for_prediction", 5.0);
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
        world_model()->getMsg().play_situation.command_raw.value ==
        robocup_ssl_msgs::msg::Referee::COMMAND_STOP) {
        // STOPのときにはボールを排出しない
        inplay(false);
      } else {
        inplay(true);
      }
      break;
    }
  }

  visualizer->text()
    .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
    .text(phase)
    .fill("white")
    .fontSize(100)
    .build();
  return Status::RUNNING;
}

void Goalie::emitBallFromPenaltyArea()
{
  Point ball = world_model()->ball.pos;
  // パスできるロボットのリストアップ
  auto passable_robot_list = world_model()->ours.getAvailableRobots(command->getMsg().robot_id);
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
  //     auto robots_with_score =
  //       passable_robot_list | ranges::views::transform([&](const std::shared_ptr<RobotInfo> robot) {
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

  visualizer->line().start(ball).end(pass_target).stroke("blue").strokeWidth(10).build();

  Point intermediate_point = ball + (ball - pass_target).normalized() * 0.2f;
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
  const auto & ball = world_model()->ball;
  // シュートチェック
  Segment goal_line(goals.first, goals.second);
  Segment ball_line(ball.pos, ball.pos + ball.vel.normalized() * 20.f);
  auto intersections = getIntersections(ball_line, Segment{goals.first, goals.second});
  command->setTerminalVelocity(0.0).disableGoalAreaAvoidance().disableBallAvoidance();

  if (not intersections.empty() && world_model()->ball.vel.norm() > 0.3f) {
    // シュートブロック
    phase = "シュートブロック";
    auto result = getClosestPointAndDistance(ball_line, command->getRobot()->pose.pos);
    auto target = [&]() {
      if (not world_model()->point_checker.isFieldInside(result.closest_point)) {
        // フィールド外（=ゴール内）でのセーブは避ける
        return intersections.front();
      } else {
        return result.closest_point;
      }
    }();

    command->setTargetPosition(target).lookAtBallFrom(target);
    if (command->getRobot()->getDistance(target) > 0.05) {
      // なりふり構わず爆加速
      // command->setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
    }
  } else {
    if (
      world_model()->ball.isStopped(0.2) &&
      world_model()->point_checker.isFriendPenaltyArea(ball.pos) && enable_emit) {
      // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
      phase = "ボール排出";
      emitBallFromPenaltyArea();
    } else {
      const double BLOCK_DIST = getParameter<double>("block_distance");
      phase += "ボールを待ち受ける";
      // デフォルト位置設定
      command->setTargetPosition(world_model()->getOurGoalCenter() * 0.9).lookAt(Point(0, 0));
      if (std::signbit(world_model()->ball.pos.x()) == std::signbit(world_model()->goal.x())) {
        phase += " (自コート警戒モード)";
        Segment ball_prediction_4s(ball.pos, ball.pos + ball.vel * 4.0);
        auto [next_their_attacker, distance] = [&]() {
          std::shared_ptr<RobotInfo> nearest_enemy = nullptr;
          double min_distance = 1000000.0;
          for (const auto & enemy : world_model()->theirs.getAvailableRobots()) {
            double dist = bg::distance(enemy->pose.pos, ball_prediction_4s);
            if (dist < min_distance) {
              Vector2 ball_to_enemy = (enemy->pose.pos - ball.pos).normalized();
              Vector2 ball_direction = ball.vel.normalized();
              //  ボールの進行方向のロボットのみ反映
              if (ball_to_enemy.dot(ball_direction) > 0.0) {
                min_distance = dist;
                nearest_enemy = enemy;
              }
            }
          }
          return std::make_pair(nearest_enemy, min_distance);
        }();

        Point goal_center = world_model()->getOurGoalCenter();
        goal_center << goals.first.x() - std::clamp(goals.first.x(), -0.1, 0.1), 0.0f;

        if (not world_model()->point_checker.isFieldInside(ball.pos)) {
          // TODO(HansRobo): 一番近いフィールド内の点を警戒するようにする
          phase += "(範囲外なので正面に構える)";
          command->setTargetPosition(goal_center, 0.1).lookAt(Point(0, 0));
        } else {
          Point threat_point = world_model()->ball.pos;
          // bool penalty_area_pass_to_side = [&]() {
          //   Point penalty_base_1 = world_model()->getOurGoalCenter();
          //   Point penalty_base_2 = world_model()->getOurGoalCenter();
          //   penalty_base_1.y() = world_model()->penalty_area_size.y() * 0.5;
          //   penalty_base_2.y() = -world_model()->penalty_area_size.y() * 0.5;
          //   auto offset =
          //     Point(-world_model()->penalty_area_size.x() * world_model()->getOurSideSign(), 0.0);
          //   Segment goal_side1{penalty_base_1, penalty_base_1 + offset};
          //   Segment goal_side2{penalty_base_2, penalty_base_2 + offset};
          //
          //   auto result1 = getIntersections(ball_prediction_4s, goal_side1);
          //   auto result2 = getIntersections(ball_prediction_4s, goal_side2);
          //   if (result1.empty() && result2.empty()) {
          //     return false;
          //   } else if (not result1.empty() && not result2.empty()) {
          //     // 遠い方をthreat_pointにする
          //     double dist1 = bg::distance(ball.pos, result1.front());
          //     double dist2 = bg::distance(ball.pos, result2.front());
          //     if (dist1 < dist2) {
          //       threat_point = result2.front();
          //     } else {
          //       threat_point = result1.front();
          //     }
          //     return true;
          //   } else {
          //     if (not result1.empty()) {
          //       threat_point = result1.front();
          //     } else {
          //       threat_point = result2.front();
          //     }
          //     return true;
          //   }
          // }();

          // bool penalty_area_pass_to_front = [&]() {
          //   Point penalty_front_1;
          //   Point penalty_front_2;
          //   penalty_front_1.x() = penalty_front_2.x() =
          //     world_model()->getOurGoalCenter().x() - world_model()->penalty_area_size.x();
          //   penalty_front_1.y() = world_model()->penalty_area_size.y() * 0.5;
          //   penalty_front_2.y() = -world_model()->penalty_area_size.y() * 0.5;
          //   Segment goal_front_line(penalty_front_1, penalty_front_2);
          //
          //   if (auto result = getIntersections(ball_prediction_4s, goal_front_line);
          //       result.empty()) {
          //     return false;
          //   } else {
          //     threat_point = result.front();
          //     return true;
          //   }
          // }();

          // if (distance < 2.0 && (penalty_area_pass_to_front || penalty_area_pass_to_side)) {
          //   if (penalty_area_pass_to_front) {
          //     // TODO(HansRobo): 将来的には、パス経路を止めるのではなく適宜前進守備を行う
          //     // ペナルティーエリアの少し内側で待ち受ける
          //     Point wait_point = threat_point + (threat_point - ball.pos).normalized() * 0.2;
          //     command->setTargetPosition(wait_point).lookAtBall();
          //     phase += "(パスカットモードFRONT)";
          //   } else if (penalty_area_pass_to_side) {
          //     // ペナルティーエリアの少し内側で待ち受ける
          //     Point wait_point = threat_point + (threat_point - ball.pos).normalized() * 0.2;
          //     command->setTargetPosition(wait_point).lookAtBall();
          //     phase += "(パスカットモードSIDE)";
          //   }
          // } else {
          if (distance < 2.0) {
            phase += "(敵のパス先警戒モード)";
            auto result =
              getClosestPointAndDistance(ball_prediction_4s, next_their_attacker->pose.pos);
            
            // ボールが敵ロボットに届くまでに到達可能な最大限の前進守備を行う。
            // 前進するライン
            auto forward_line = Segment(
              result.closest_point,
              world_model()->ours.getAvailableRobots(world_model()->getOurGoalieId()).front()->pose.pos);
            
            // ボールが敵ロボットに届くまでの時間
            double ball_to_enemy_dist = bg::distance(ball.pos, result.closest_point);
            auto estimated_ball_reach_time = getBallReachTime(ball_to_enemy_dist, ball.vel.norm());
            if(not estimated_ball_reach_time){
              threat_point = result.closest_point;
            }
            else{
              threat_point = result.closest_point;
              
              // 敵の予想されるロボット位置とゴールの間の直線を10点に分割
              std::vector<Point> forward_pooints = getSeparatedPoints(forward_line, 10);
              for(int i = forward_pooints.size()-1; i >= 0; --i){
                // goalieが前進守備位置に到達する時間
                double travel_time = getTravelTimeTrapezoidal(this->robot(), forward_pooints[i], 0.5, 2.0);
                if( estimated_ball_reach_time > travel_time){
                  threat_point = forward_pooints[i];
                  break;
                }
              }
            }            
          } else {
            phase += "(とりあえず0.5s先を警戒モード)";
            threat_point = ball.pos + ball.vel * 0.5;
          }

          auto [weak_point, dist] = [&]() {
            if (auto other_robots =
                  world_model()->ours.getAvailableRobots(world_model()->getOurGoalieId());
                not other_robots.empty()) {
              auto goal =
                world_model()->getLargestOurGoalAngleRangeFromPoint(threat_point, other_robots);
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
                auto segment_goal_to_penalty_area =
                  Segment(intersect_to_goal_line.front(), *intersect_to_penalty_area);
                // ペナルティエリアライン上にボールがあるときにペナルティエリア上まで前進すると
                // シュートをずらして打たれて決められてしまう?
                double dist =
                  bg::distance(intersect_to_goal_line.front(), *intersect_to_penalty_area) *
                  (*ratio);
                visualizer->line()
                  .start(intersect_to_goal_line.front())  // 開始点
                  .end(*intersect_to_penalty_area)        // 終了点
                  .stroke("red")
                  .strokeWidth(1.0)
                  .build();
                phase += "(前進守備量可変)";
                command->addStateFactor("goalie", "dist:" + std::to_string(dist));
                return std::make_pair(intersect_to_goal_line.front(), dist);
              }
            } else {
              return std::make_pair(goal_center, BLOCK_DIST);
            }
          }();
          Point wait_point = weak_point + (threat_point - weak_point).normalized() * dist;

          command->setTargetPosition(wait_point).lookAtBallFrom(wait_point);
          if (command->getRobot()->getDistance(wait_point) > 0.03) {
            // なりふり構わず爆加速
            //              command->setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
          }
          // }
        }
      }
    }
  }
}
}  // namespace crane::skills
