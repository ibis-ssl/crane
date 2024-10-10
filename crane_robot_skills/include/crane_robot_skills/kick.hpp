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
    receive_skill->setParameter("policy", std::string("closest"));
    receive_skill->setParameter("enable_active_receive", true);
    receive_skill->setParameter("enable_redirect", true);
    receive_skill->setParameter("redirect_target", Point(0, 0));
    receive_skill->setParameter("redirect_kick_power", 0.3);

    addStateFunction(
      KickState::ENTRY_POINT,
      []([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        return Status::RUNNING;
      });
    addTransition(KickState::ENTRY_POINT, KickState::CHASE_BALL, [this]() {
      return world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
    });

    addTransition(KickState::ENTRY_POINT, KickState::AROUND_BALL, [this]() { return true; });

    addStateFunction(
      KickState::CHASE_BALL,
      [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        // メモ：ボールが近い時はボールから少しずらした位置を目指したほうがいいかも
        auto [min_slack_pos, max_slack_pos] = world_model()->getMinMaxSlackInterceptPoint(
          {robot()}, 5.0, 0.1, -1.0, command.getMsg().local_planner_config.max_acceleration,
          command.getMsg().local_planner_config.max_velocity);
        if (min_slack_pos) {
          command.setTargetPosition(min_slack_pos.value()).lookAtBallFrom(min_slack_pos.value());
        } else {
          // ball_lineとフィールドラインの交点を目指す
          Point ball_exit_point = getBallExitPointFromField(0.3);
          command.setTargetPosition(ball_exit_point).lookAtBallFrom(ball_exit_point);
        }
        return Status::RUNNING;
      });

    addTransition(KickState::CHASE_BALL, KickState::AROUND_BALL, [this]() {
      // ボールが止まったら回り込みへ
      return not world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
    });

    addTransition(KickState::CHASE_BALL, KickState::REDIRECT_KICK, [this]() {
      // ボールライン上に乗ったらリダイレクトキックへ
      return world_model()->ball.isMovingTowards(robot()->pose.pos, 10.0);
    });

    addStateFunction(
      KickState::REDIRECT_KICK, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        receive_skill->setParameter("target", getParameter<Point>("target"));
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
      KickState::AROUND_BALL,
      [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
        auto target = getParameter<Point>("target");
        Point ball_pos = world_model()->ball.pos;

        // 経由ポイント
        Point intermediate_point =
          ball_pos + (ball_pos - target).normalized() * getParameter<double>("around_interval");
        command.setTargetPosition(intermediate_point)
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
          command.setTargetPosition(around_point).setTargetTheta(getAngle(target - ball_pos));
        } else {
          // 経由ポイントへGO
          command.setTargetPosition(intermediate_point)
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
      return robot()->getDistance(intermediate_point) < 0.1;
    });

    addStateFunction(
      KickState::KICK,
      [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) {
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
  }

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  {
    const Point target = getParameter<Point>("target");
    const Point ball_pos = world_model()->ball.pos;

    // ボールを避けて回り込む
    if (
      getParameter<bool>("go_around_ball") &&
      ((robot()->pose.pos - ball_pos).normalized()).dot((target - ball_pos).normalized()) > 0.1) {
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
      command.setTargetPosition(around_point).setTargetTheta(getAngle(target - ball_pos));

    } else {  // この部分はいずれ回り込み＆キックのスキルとして一般化したい
              // (その時は動くボールへの回り込みを含めて)
      // パラメータ候補：キックパワー・dotしきい値・角度しきい値・経由ポイント距離・突撃速度
      phase = "キック";

      double dot =
        (robot()->pose.pos - ball_pos).normalized().dot((ball_pos - target).normalized());
      double angle_diff =
        std::abs(getAngleDiff(getAngle(target - world_model()->ball.pos), robot()->pose.theta));
      if (
        (dot > getParameter<double>("dot_threshold") ||
         (robot()->pose.pos - ball_pos).norm() < 0.1) &&
        angle_diff < getParameter<double>("angle_threshold")) {
        // キック
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
        // TODO(HansRobo): 終了判定
      } else {
        // 経由ポイントへGO
        phase = "経由ポイントへGO";
        Point intermediate_point =
          ball_pos + (ball_pos - target).normalized() * getParameter<double>("around_interval");
        command.setTargetPosition(intermediate_point)
          .enableCollisionAvoidance()
          .enableBallAvoidance();
      }
      // 共通コマンド
      command.liftUpDribbler().setTargetTheta(getAngle(target - world_model()->ball.pos));
    }

    return Status::RUNNING;
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

    Segment seg1{Point(X, Y), Point(X, -Y)};
    Segment seg2{Point(-X, Y), Point(-X, -Y)};
    Segment seg3{Point(Y, X), Point(-Y, X)};
    Segment seg4{Point(Y, -X), Point(-Y, -X)};
    std::vector<Point> intersections;
    if (bg::intersection(ball_line, seg1, intersections); not intersections.empty()) {
      return intersections.front();
    } else if (bg::intersection(ball_line, seg2, intersections); not intersections.empty()) {
      return intersections.front();
    } else if (bg::intersection(ball_line, seg3, intersections); not intersections.empty()) {
      return intersections.front();
    } else if (bg::intersection(ball_line, seg4, intersections); not intersections.empty()) {
      return intersections.front();
    } else {
      return ball_line.second;
    }
  }

  void print(std::ostream & os) const override { os << "[Kick]"; }

  std::string & phase;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__KICK_HPP_
