// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/goalie_vel.hpp>

namespace crane::skills
{
GoalieVel::GoalieVel(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase<RobotCommandWrapperSimpleVelocity>("GoalieVel", base),
  phase(getContextReference<std::string>("phase")),
  kick_skill(base)
{
  setParameter("run_inplay", true);
  setParameter("block_distance", 1.0);
}

Status GoalieVel::update(const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  auto situation = world_model()->play_situation.getSituationCommandID();
  if (getParameter<bool>("run_inplay")) {
    situation = crane_msgs::msg::PlaySituation::OUR_INPLAY;
  }

  switch (situation) {
    case crane_msgs::msg::PlaySituation::HALT:
      phase = "HALT, stop here";
      command.stopHere();
      break;
    case crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION:
      [[fallthrough]];
    case crane_msgs::msg::PlaySituation::THEIR_PENALTY_START:
      phase = "ペナルティキック";
      inplay(false, visualizer);
      break;
    default:
      inplay(true, visualizer);
      break;
  }

  visualizer->addPoint(robot()->pose.pos.x(), robot()->pose.pos.y(), 0, "white", 1., phase);
  return Status::RUNNING;
}

void GoalieVel::emitBallFromPenaltyArea(const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  Point ball = world_model()->ball.pos;
  // パスできるロボットのリストアップ
  auto passable_robot_list = world_model()->ours.getAvailableRobots(command.getMsg().robot_id);
  passable_robot_list.erase(
    std::remove_if(
      passable_robot_list.begin(), passable_robot_list.end(),
      [&](const RobotInfo::SharedPtr & r) {
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
      }),
    passable_robot_list.end());

  Point pass_target = [&]() {
    if (not passable_robot_list.empty()) {
      // TODO(HansRobo): いい感じのロボットを選ぶようにする
      return passable_robot_list.front()->pose.pos;
    } else {
      return world_model()->getTheirGoalCenter();
    }
  }();

  visualizer->addLine(ball, pass_target, 1, "blue");

  Point intermediate_point = ball + (ball - pass_target).normalized() * 0.2f;
  kick_skill.setParameter("target", pass_target);
  kick_skill.setParameter("kick_power", 1.0);
  kick_skill.setParameter("chip_kick", true);
  kick_skill.run(visualizer);
  // 追加のコマンド
  command.disableGoalAreaAvoidance().disableRuleAreaAvoidance();
}

void GoalieVel::inplay(bool enable_emit, const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  auto goals = world_model()->getOurGoalPosts();
  const auto & ball = world_model()->ball;
  // シュートチェック
  Segment goal_line(goals.first, goals.second);
  Segment ball_line(ball.pos, ball.pos + ball.vel.normalized() * 20.f);
  auto intersections = getIntersections(ball_line, Segment{goals.first, goals.second});
  command.setTerminalVelocity(0.0)
    .disableGoalAreaAvoidance()
    .disableBallAvoidance()
    .disableRuleAreaAvoidance();

  if (not intersections.empty() && world_model()->ball.vel.norm() > 0.3f) {
    // シュートブロック
    phase = "シュートブロック";
    auto result = getClosestPointAndDistance(ball_line, command.getRobot()->pose.pos);
    auto target = [&]() {
      if (not world_model()->point_checker.isFieldInside(result.closest_point)) {
        // フィールド外（=ゴール内）でのセーブは避ける
        return intersections.front();
      } else {
        return result.closest_point;
      }
    }();

    command.setTargetPosition(target).lookAtBallFrom(target);
    if (command.getRobot()->getDistance(target) > 0.05) {
      // なりふり構わず爆加速
      command.setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
    }
  } else {
    if (
      world_model()->ball.isStopped(0.2) &&
      world_model()->point_checker.isFriendPenaltyArea(ball.pos) && enable_emit) {
      // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
      phase = "ボール排出";
      emitBallFromPenaltyArea(visualizer);
    } else {
      phase = "";
      const double BLOCK_DIST = getParameter<double>("block_distance");
      phase += "ボールを待ち受ける";
      // デフォルト位置設定
      command.setTargetPosition(world_model()->getOurGoalCenter()).lookAt(Point(0, 0));
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
          command.setTargetPosition(goal_center).lookAt(Point(0, 0));
        } else {
          Point threat_point = world_model()->ball.pos;
          bool penalty_area_pass_to_side = [&]() {
            Point penalty_base_1, penalty_base_2;
            penalty_base_1 = penalty_base_2 = world_model()->getOurGoalCenter();
            penalty_base_1.y() = world_model()->penalty_area_size.y() * 0.5;
            penalty_base_2.y() = -world_model()->penalty_area_size.y() * 0.5;
            auto offset =
              Point(-world_model()->penalty_area_size.x() * world_model()->getOurSideSign(), 0.0);
            Segment goal_side1{penalty_base_1, penalty_base_1 + offset};
            Segment goal_side2{penalty_base_2, penalty_base_2 + offset};
            std::vector<Point> result1, result2;
            bg::intersection(ball_prediction_4s, goal_side1, result1);
            bg::intersection(ball_prediction_4s, goal_side2, result2);
            if (result1.empty() && result2.empty()) {
              return false;
            } else if (not result1.empty() && not result2.empty()) {
              // 遠い方をthreat_pointにする
              double dist1 = bg::distance(ball.pos, result1.front());
              double dist2 = bg::distance(ball.pos, result2.front());
              if (dist1 < dist2) {
                threat_point = result2.front();
              } else {
                threat_point = result1.front();
              }
              return true;
            } else {
              if (not result1.empty()) {
                threat_point = result1.front();
              } else {
                threat_point = result2.front();
              }
              return true;
            }
          }();

          bool penalty_area_pass_to_front = [&]() {
            Point penalty_front_1, penalty_front_2;
            penalty_front_1.x() = penalty_front_2.x() =
              world_model()->getOurGoalCenter().x() - world_model()->penalty_area_size.x();
            penalty_front_1.y() = world_model()->penalty_area_size.y() * 0.5;
            penalty_front_2.y() = -world_model()->penalty_area_size.y() * 0.5;
            Segment goal_front_line(penalty_front_1, penalty_front_2);
            std::vector<Point> result;
            bg::intersection(ball_prediction_4s, goal_front_line, result);
            if (result.empty()) {
              return false;
            } else {
              threat_point = result.front();
              return true;
            }
          }();

          if (distance < 2.0 && (penalty_area_pass_to_front || penalty_area_pass_to_side)) {
            if (penalty_area_pass_to_front) {
              // TODO(HansRobo): 将来的には、パス経路を止めるのではなく適宜前進守備を行う
              // ペナルティーエリアの少し内側で待ち受ける
              Point wait_point = threat_point + (threat_point - ball.pos).normalized() * 0.2;
              command.setTargetPosition(wait_point).lookAtBallFrom(wait_point);
              if (command.getRobot()->getDistance(wait_point) > 0.03) {
                // なりふり構わず爆加速
                command.setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
              }
              phase += "(パスカットモードFRONT)";
            } else if (penalty_area_pass_to_side) {
              // ペナルティーエリアの少し内側で待ち受ける
              Point wait_point = threat_point + (threat_point - ball.pos).normalized() * 0.2;
              command.setTargetPosition(wait_point).lookAtBallFrom(wait_point);
              if (command.getRobot()->getDistance(wait_point) > 0.03) {
                // なりふり構わず爆加速
                command.setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
              }
              phase += "(パスカットモードSIDE)";
            }
          } else {
            if (distance < 2.0) {
              phase += "(敵のパス先警戒モード)";
              auto result =
                getClosestPointAndDistance(ball_prediction_4s, next_their_attacker->pose.pos);
              threat_point = result.closest_point;
            } else {
              phase += "(とりあえず0.5s先を警戒モード)";
              threat_point = ball.pos + ball.vel * 0.5;
            }
            Point weak_point = [&]() {
              auto [angle, interval] = world_model()->getLargestOurGoalAngleRangeFromPoint(
                threat_point,
                world_model()->ours.getAvailableRobots(world_model()->getOurGoalieId()));
              Segment expected_ball_line(threat_point, threat_point + getNormVec(angle) * 10);
              Segment goal_line(goals.first, goals.second);
              auto intersections = getIntersections(expected_ball_line, goal_line);
              if (intersections.empty()) {
                return goal_center;
              } else {
                return intersections.front();
              }
            }();

            Point wait_point = weak_point + (threat_point - weak_point).normalized() * BLOCK_DIST;

            command.setTargetPosition(wait_point).lookAtBallFrom(wait_point);
            if (command.getRobot()->getDistance(wait_point) > 0.03) {
              // なりふり構わず爆加速
              command.setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
            }
          }
        }
      }
    }
  }
}
}  // namespace crane::skills
