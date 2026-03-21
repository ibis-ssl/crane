// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/defender_session.hpp>

namespace crane
{
std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
DefenderSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {SessionBase::Status::RUNNING, {}};
  }

  const auto & ball = world_model->ball();
  Segment ball_line(ball.pos, ball.pos + ball.vel.normalized() * 20.f);
  {
    // シュート判定
    auto goal_posts = world_model->getOurGoalPosts();
    Segment goal_line(goal_posts.first, goal_posts.second);
    auto intersections = getIntersections(ball_line, goal_line);
    if (intersections.empty()) {
      // シュートがなければ通常の動き（TotalDefenseSessionと向きを統一）
      ball_line.first = ball.pos;
      ball_line.second = world_model->getOurGoalCenter();
    }
  }

  std::vector<Point> defense_points = [&]() {
    // フィールド横幅の半分よりボールが遠ければ円弧守備に移行
    if (
      (world_model->ball().pos - world_model->getOurGoalCenter()).norm() <
      world_model->fieldSize().y() * 0.5) {
      // ball_lineが防御ラインと交差しない場合のフォールバック
      // ボールがPA境界〜防御ライン間にいるとき、ball_line(ball→goal)が防御ラインの内側を向くため交差しない
      auto defense_parameter = getDefenseLinePointParameter(ball_line, world_model);
      Segment effective_ball_line = ball_line;
      if (not defense_parameter) {
        effective_ball_line = Segment{
          world_model->getOurGoalCenter(),
          ball.pos + (ball.pos - world_model->getOurGoalCenter()).normalized() * 2.0};
        defense_parameter = getDefenseLinePointParameter(effective_ball_line, world_model);
      }
      return getDefenseLinePoints(
        robots.size(), effective_ball_line, world_model, false, defense_parameter);
    } else {
      return getDefenseArcPoints(robots.size(), ball_line, world_model);
    }
  }();

  if (not defense_points.empty()) {
    auto robot_commands = assignRobotsToPoints(
      robots, defense_points, "defender_planner", ball.pos,
      [&](std::shared_ptr<PositionCommandWrapper> & command) {
        command->disableBasicAvoidances();
        if (
          world_model->getMsg().play_situation.command.value ==
          crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT) {
          command->disableAnyAreaAvoidance();
          command->enablePlacementAvoidance();
        } else {
          command->disableAnyAreaAvoidance();
          // INPLAY時のみPA回避を有効化
          // INPLAY: PA回避offset=0.1m < defense_offset=0.2m → 有効で安全
          // STOP/セットプレイ: PA回避offset=0.3m > defense_offset=0.2m → 競合するため無効
          if (
            world_model->getMsg().play_situation.command.value ==
            crane_msgs::msg::PlaySituation::INPLAY) {
            command->enableGoalAreaAvoidance();
          }
        }
      });
    return {SessionBase::Status::RUNNING, robot_commands};
  } else {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (const auto & robot_id : robots) {
      auto command = std::make_shared<crane::PositionCommandWrapper>(
        "defender_planner/stop", robot_id.id, world_model);
      command->stopHere();
      robot_commands.emplace_back(command->getMsg());
    }
    return {SessionBase::Status::RUNNING, robot_commands};
  }
}
}  // namespace crane
