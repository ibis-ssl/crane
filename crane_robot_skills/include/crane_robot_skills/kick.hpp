// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__KICK_HPP_
#define CRANE_ROBOT_SKILLS__KICK_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/receive.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
enum class KickState {
  ENTRY_POINT,
  CHASE_BALL,
  AROUND_BALL,
  KICK,
  REDIRECT_KICK,
  POSITIVE_REDIRECT_KICK,
};

class Kick : public SkillBaseWithState<KickState, RobotCommandWrapperPosition>
{
private:
  std::shared_ptr<Receive> receive_skill;

public:
  explicit Kick(RobotCommandWrapperBase::SharedPtr & base)
  : SkillBaseWithState<KickState>("Kick", base, KickState::ENTRY_POINT),
    receive_skill(std::make_shared<Receive>(base)),
    phase(getContextReference<std::string>("phase"))
  {
    setParameter("target", Point(0, 0));
    setParameter("kick_power", 0.5f);
    setParameter("chip_kick", false);
    setParameter("with_dribble", false);
    setParameter("dribble_power", 0.3f);
    setParameter("dot_threshold", 0.95f);
    setParameter("angle_threshold", 0.1f);
    setParameter("around_interval", 0.3f);
    setParameter("go_around_ball", true);
    setParameter("moving_speed_threshold", 0.2);
    setParameter("kicked_speed_threshold", 1.5);

    receive_skill->setParameter("dribble_power", 0.3);
    receive_skill->setParameter("enable_software_bumper", false);
    receive_skill->setParameter("policy", std::string("min_slack"));
    receive_skill->setParameter("enable_active_receive", true);
    receive_skill->setParameter("enable_redirect", true);
    receive_skill->setParameter("redirect_target", Point(0, 0));
    receive_skill->setParameter("redirect_kick_power", 0.3);

    addStateFunction(
      KickState::ENTRY_POINT, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::ENTRY_POINT");
        return Status::RUNNING;
      });
    addTransition(KickState::ENTRY_POINT, KickState::CHASE_BALL, [this]() {
      return world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
    });

    addTransition(KickState::ENTRY_POINT, KickState::AROUND_BALL, [this]() { return true; });

    addStateFunction(
      KickState::CHASE_BALL, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        std::stringstream state;
        state << "Kick::CHASE_BALL::";
        // メモ：ボールが近い時はボールから少しずらした位置を目指したほうがいいかも
        auto [min_slack_pos, max_slack_pos] = world_model()->getMinMaxSlackInterceptPoint(
          {robot()}, 5.0, 0.1, -0.1, command.getMsg().local_planner_config.max_acceleration,
          command.getMsg().local_planner_config.max_velocity);
        if (min_slack_pos) {
          state << "min_slack: " << min_slack_pos.value().x() << ", " << min_slack_pos.value().y();
          command.setTargetPosition(min_slack_pos.value()).lookAtBallFrom(min_slack_pos.value());
        } else {
          // ball_lineとフィールドラインの交点を目指す
          Point ball_exit_point = getBallExitPointFromField(0.3);
          command.setTargetPosition(ball_exit_point).lookAtBallFrom(ball_exit_point);
          state << "ball_exit: " << ball_exit_point.x() << ", " << ball_exit_point.y();
        }
        command.enableBallAvoidance();
        visualizer->addPoint(robot()->pose.pos, 0, "", 1., state.str());
        return Status::RUNNING;
      });

    addTransition(KickState::CHASE_BALL, KickState::AROUND_BALL, [this]() {
      // ボールが止まったら回り込みへ
      command.disableBallAvoidance();
      return not world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
    });

    addTransition(KickState::CHASE_BALL, KickState::REDIRECT_KICK, [this]() {
      // ボールライン上に乗ったらリダイレクトキックへ
      command.disableBallAvoidance();
      return world_model()->ball.isMovingTowards(robot()->pose.pos, 10.0);
    });

    addTransition(KickState::CHASE_BALL, KickState::POSITIVE_REDIRECT_KICK, [this]() {
      return world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 10.0) &&
             world_model()->ball.isMovingTowards(getParameter<Point>("target"), 30.0);
    });

    addStateFunction(
      KickState::POSITIVE_REDIRECT_KICK,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::POSITIVE_REDIRECT_KICK");
        // ボールラインに沿って追いかけつつ、角度はtargetへ向ける
        const auto & ball_pos = world_model()->ball.pos;
        command.lookAtFrom(getParameter<Point>("target"), ball_pos);

        const auto & ball_vel_normed = world_model()->ball.vel.normalized();
        Segment ball_line{ball_pos - ball_vel_normed * 10, ball_pos + ball_vel_normed * 10};
        auto [distance, closest_point] = getClosestPointAndDistance(ball_pos, ball_line);
        auto target_pos = [&]() -> Point {
          if (distance < 0.1) {
            return ball_pos + ball_vel_normed;
          } else {
            return closest_point + ball_vel_normed * distance;
          }
        }();
        command.setDribblerTargetPosition(target_pos);
        command.kickStraight(0.3);
        command.disableBallAvoidance();
        return Status::RUNNING;
      });

    addTransition(KickState::POSITIVE_REDIRECT_KICK, KickState::ENTRY_POINT, [this]() {
      return !world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 10.0) or
             !world_model()->ball.isMovingTowards(getParameter<Point>("target"), 30.0);
    });

    addStateFunction(
      KickState::REDIRECT_KICK, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::REDIRECT_KICK");
        receive_skill->setParameter("target", getParameter<Point>("target"));
        if (robot()->getDistance(world_model()->ball.pos) < 0.5) {
          receive_skill->setParameter("policy", std::string("closest"));
        } else {
          receive_skill->setParameter("policy", std::string("min_slack"));
        }
        return receive_skill->update(visualizer);
      });

    addTransition(KickState::REDIRECT_KICK, KickState::AROUND_BALL, [this]() {
      // ボールが止まったら回り込みへ
      return not world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
    });

    addTransition(KickState::REDIRECT_KICK, KickState::ENTRY_POINT, [this]() {
      // 素早く遠ざかっていったら終了
      return world_model()->ball.isMoving(getParameter<double>("kicked_speed_threshold")) &&
             world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 30.);
    });

    addStateFunction(
      KickState::AROUND_BALL, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::AROUND_BALL");
        auto target = getParameter<Point>("target");
        Point ball_pos = world_model()->ball.pos;

        // 経由ポイント
        Point intermediate_point =
          ball_pos + (ball_pos - target).normalized() * getParameter<double>("around_interval");
        command.setTargetPosition(intermediate_point)
          .lookAtBallFrom(intermediate_point)
          .enableCollisionAvoidance()
          .enableBallAvoidance();

        // ボールを避けて回り込む
        if (
          ((robot()->pose.pos - ball_pos).normalized()).dot((target - ball_pos).normalized()) >
          0.1) {
          Point around_point = [&]() {
            Vector2 vertical_vec = getVerticalVec((target - ball_pos).normalized()) *
                                   getParameter<double>("around_interval");
            Point around_point1 = ball_pos + vertical_vec;
            Point around_point2 = ball_pos - vertical_vec;
            if (robot()->getDistance(around_point1) < robot()->getDistance(around_point2)) {
              return around_point1;
            } else {
              return around_point2;
            }
          }();
          command.setTargetPosition(around_point).lookAtBallFrom(around_point);
        } else {
          // 経由ポイントへGO
          command.setTargetPosition(intermediate_point)
            .lookAtBallFrom(intermediate_point)
            .enableCollisionAvoidance()
            .enableBallAvoidance();
        }
        return Status::RUNNING;
      });

    addTransition(KickState::AROUND_BALL, KickState::KICK, [this]() {
      // 中間地点に到達したらキックへ
      Point intermediate_point =
        world_model()->ball.pos +
        (world_model()->ball.pos - getParameter<Point>("target")).normalized() *
          getParameter<double>("around_interval");
      return robot()->getDistance(intermediate_point) < 0.05 && robot()->vel.linear.norm() < 0.1;
    });

    addStateFunction(
      KickState::KICK, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::KICK");
        auto target = getParameter<Point>("target");
        Point ball_pos = world_model()->ball.pos;
        command.setTargetPosition(ball_pos + (target - ball_pos).normalized() * 0.3)
          .disableCollisionAvoidance()
          .disableBallAvoidance();
        if (getParameter<bool>("chip_kick")) {
          command.kickWithChip(getParameter<double>("kick_power"));
        } else {
          command.kickStraight(getParameter<double>("kick_power"));
        }
        if (getParameter<bool>("with_dribble")) {
          command.dribble(getParameter<double>("dribble_power"));
        } else {
          // ドリブラーを止める
          command.withDribble(0.0);
        }
        return Status::RUNNING;
      });

    addTransition(KickState::KICK, KickState::ENTRY_POINT, [this]() {
      // 素早く遠ざかっていったら終了
      return world_model()->ball.isMoving(getParameter<double>("kicked_speed_threshold")) &&
             world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 30.);
    });

    addTransition(KickState::KICK, KickState::ENTRY_POINT, [this]() -> bool {
      // 素早く遠ざかっていったら終了
      auto target = getParameter<Point>("target");
      Point ball_pos = world_model()->ball.pos;
      Point p = ball_pos + (target - ball_pos).normalized() * 0.3;
      return robot()->getDistance(p) < 0.1;
    });
  }

  /**
   * @brief ボールがフィールドから出る位置を取得
   * @param offset 出る位置を内側にずらすオフセット
   * @return ボールが出る位置
   */
  auto getBallExitPointFromField(const double offset = 0.3) -> Point
  {
    Segment ball_line{
      world_model()->ball.pos,
      world_model()->ball.pos + world_model()->ball.vel.normalized() * 10.0};

    const double X = world_model()->field_size.x() / 2.0 - offset;
    const double Y = world_model()->field_size.y() / 2.0 - offset;

    std::vector<Segment> segments;
    segments.emplace_back(Point(X, Y), Point(X, -Y));
    segments.emplace_back(Point(-X, Y), Point(-X, -Y));
    segments.emplace_back(Point(X, Y), Point(-X, Y));
    segments.emplace_back(Point(X, -Y), Point(-X, -Y));

    for (const auto & seg : segments) {
      if (auto intersections = getIntersections(ball_line, seg); not intersections.empty()) {
        return intersections.front();
      }
    }
    return world_model()->ball.pos;
  }

  void print(std::ostream & os) const override { os << "[Kick]"; }

  std::string & phase;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICK_HPP_
