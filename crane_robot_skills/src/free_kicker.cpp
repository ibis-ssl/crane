// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/pass.hpp>
#include <crane_robot_skills/free_kicker.hpp>
#include <crane_robot_skills/goal_kick.hpp>
#include <magic_enum/magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crane::skills
{
namespace
{
constexpr double FK_KICK_DETECT_VEL = 0.7;
constexpr double FK_KICK_DIRECTION_COS_THRESHOLD = 0.7;
constexpr double FK_APPROACH_PHASE_TIMEOUT = 6.0;
constexpr double FK_ALIGN_PHASE_TIMEOUT = 4.0;
constexpr double FK_KICK_TIMEOUT_SEC = 3.0;
constexpr double FK_MIN_PASS_ACCEPT_SCORE = 0.4;
constexpr double FK_PASS_HYSTERESIS_RATIO = 0.85;
constexpr double FK_APPROACH_ANGLE_TOLERANCE_RAD = 10.0 * M_PI / 180.0;
constexpr double FK_APPROACH_SPEED_TOLERANCE = 0.3;
}  // namespace

std::string FreeKicker::getStateName(int s)
{
  return std::string(magic_enum::enum_name(static_cast<FreeKickerState>(s)));
}

void FreeKicker::resetInternalState()
{
  kick_started_ = false;
  target_locked_ = false;
  use_chip_ = false;
  chip_distance_ = getParameter<double>("target_chip_distance");
  last_chose_shoot_ = false;
  last_pass_receiver_id_ = std::nullopt;
  align_stable_count_ = 0;
  approach_entry_time_ = std::nullopt;
  align_entry_time_ = std::nullopt;
  align_target_locked_ = std::nullopt;
}

void FreeKicker::initialize()
{
  setParameter("approach_max_velocity", 1.5);
  setParameter("align_max_velocity", 0.6);
  setParameter("kick_max_velocity", 1.2);
  setParameter("approach_distance", 0.25);
  setParameter("approach_position_tolerance", 0.10);
  setParameter("align_position_tolerance", 0.07);
  setParameter("align_speed_tolerance", 0.10);
  setParameter("align_omega_tolerance", 0.20);
  setParameter("align_stable_frames", 3);
  setParameter("target_kick_speed", 5.0);
  setParameter("target_chip_distance", 2.5);
  setParameter("shoot_min_angle_rad", 6.0 * M_PI / 180.0);
  setParameter("pass_obstacle_distance", 0.2);
  setParameter("pass_min_distance", 1.5);
  setParameter("pass_max_distance", 6.0);
  setParameter("enemy_slack_threshold", 0.3);
  setParameter("ball_nearby_radius", 1.0);
  setParameter("ball_nearby_speed", 0.5);
  setParameter("pass_defensive_half_penalty", 0.1);
  setParameter("pass_chip_bypass_distance", 1.5);

  using S = FreeKickerState;
  auto s = [](S st) { return static_cast<int>(st); };

  // ENTRY_POINT: 自己遷移で内部状態リセット後、必ず APPROACH へ
  addTransition(s(S::ENTRY_POINT), s(S::ENTRY_POINT), [this]() -> bool {
    resetInternalState();
    return false;
  });
  addTransition(s(S::ENTRY_POINT), s(S::APPROACH), []() -> bool { return true; });

  addStateFunction(s(S::APPROACH), [this]() -> Status {
    if (!approach_entry_time_.has_value()) {
      approach_entry_time_ = std::chrono::steady_clock::now();
    }
    if (!target_locked_) {
      kick_target_ = selectKickTarget();
    }

    const Point ball_pos = world_model()->ball().pos;
    const double approach_dist = getParameter<double>("approach_distance");
    standoff_ = computeAroundBallApproachTargetDynamic(
      ball_pos, kick_target_, robot()->pose.pos, approach_dist, approach_dist * 1.5);

    double max_vel = getParameter<double>("approach_max_velocity");
    if ((robot()->pose.pos - ball_pos).norm() < getParameter<double>("ball_nearby_radius")) {
      max_vel = std::min(max_vel, getParameter<double>("ball_nearby_speed"));
    }
    command->setTargetPosition(standoff_)
      .lookAtFrom(kick_target_, ball_pos)
      .setMaxVelocity("FreeKicker::APPROACH", max_vel);

    return Status::RUNNING;
  });
  addTransition(s(S::APPROACH), s(S::ALIGN), [this]() -> bool {
    const Point ball_pos = world_model()->ball().pos;
    double pos_error = (robot()->pose.pos - standoff_).norm();
    double angle_error =
      std::abs(normalizeAngle(robot()->pose.theta - getAngle(kick_target_ - ball_pos)));
    double speed = robot()->vel.linear.norm();

    return pos_error < getParameter<double>("approach_position_tolerance") &&
           angle_error < FK_APPROACH_ANGLE_TOLERANCE_RAD && speed < FK_APPROACH_SPEED_TOLERANCE;
  });
  addTransition(s(S::APPROACH), s(S::ENTRY_POINT), [this]() -> bool {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::steady_clock;
    return approach_entry_time_.has_value() &&
           duration_cast<duration<double>>(steady_clock::now() - *approach_entry_time_).count() >
             FK_APPROACH_PHASE_TIMEOUT;
  });

  addStateFunction(s(S::ALIGN), [this]() -> Status {
    if (!align_entry_time_.has_value()) {
      target_locked_ = true;
      align_entry_time_ = std::chrono::steady_clock::now();
      align_stable_count_ = 0;
      // APPROACH 最終地点と整合させるため、進入時のボール位置を基準に1回だけ計算してロック
      const Point ball_pos = world_model()->ball().pos;
      align_target_locked_ = ball_pos - (kick_target_ - ball_pos).normalized() *
                                          getParameter<double>("approach_distance");
    }

    command->setTargetPosition(*align_target_locked_)
      .lookAtFrom(kick_target_, world_model()->ball().pos)
      .setMaxVelocity("FreeKicker::ALIGN", getParameter<double>("align_max_velocity"))
      .disableBallAvoidance();

    double pos_error = (robot()->pose.pos - *align_target_locked_).norm();
    double speed = robot()->vel.linear.norm();
    double omega = std::abs(robot()->vel.omega);

    bool stable = pos_error < getParameter<double>("align_position_tolerance") &&
                  speed < getParameter<double>("align_speed_tolerance") &&
                  omega < getParameter<double>("align_omega_tolerance");

    if (stable) {
      align_stable_count_++;
    } else {
      align_stable_count_ = 0;
    }

    return Status::RUNNING;
  });
  addTransition(s(S::ALIGN), s(S::KICK), [this]() -> bool {
    return align_stable_count_ >= getParameter<int>("align_stable_frames");
  });
  addTransition(s(S::ALIGN), s(S::ENTRY_POINT), [this]() -> bool {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::steady_clock;
    return align_entry_time_.has_value() &&
           duration_cast<duration<double>>(steady_clock::now() - *align_entry_time_).count() >
             FK_ALIGN_PHASE_TIMEOUT;
  });

  addStateFunction(s(S::KICK), [this]() -> Status {
    if (!kick_started_) {
      kick_started_ = true;
      kick_started_time_ = std::chrono::steady_clock::now();
    }

    command
      ->setTargetPosition(
        world_model()->ball().pos + (kick_target_ - world_model()->ball().pos).normalized() * 0.1)
      .lookAtFrom(kick_target_, robot()->pose.pos)
      .setMaxVelocity("FreeKicker::KICK", getParameter<double>("kick_max_velocity"))
      .disableBallAvoidance();
    // 衝突回避は有効のまま（KickOld と異なる、フィールド外押し出し防止の肝）

    if (use_chip_) {
      command->setKickWithChipTargetDistance(chip_distance_);
    } else {
      command->setKickStraightTargetSpeed(getParameter<double>("target_kick_speed"));
    }

    return Status::RUNNING;
  });
  // KICK → ENTRY_POINT のリトライ遷移は意図的に設けない（ダブルタッチ防止）
  addTransition(s(S::KICK), s(S::FINISH), [this]() -> bool {
    if (!kick_started_) return false;

    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::steady_clock;
    double elapsed =
      duration_cast<duration<double>>(steady_clock::now() - kick_started_time_).count();
    if (elapsed > FK_KICK_TIMEOUT_SEC) return true;

    double ball_speed = world_model()->ball().vel.norm();
    if (ball_speed > FK_KICK_DETECT_VEL) {
      Vector2 ball_vel_dir = world_model()->ball().vel.normalized();
      Vector2 intended_dir = (kick_target_ - world_model()->ball().pos).normalized();
      return ball_vel_dir.dot(intended_dir) > FK_KICK_DIRECTION_COS_THRESHOLD;
    }
    return false;
  });

  addStateFunction(s(S::FINISH), [this]() -> Status {
    command->stopHere();
    return Status::SUCCESS;
  });
}

Point FreeKicker::selectKickTarget()
{
  use_chip_ = false;

  Point shoot_target;
  if (tryShoot(shoot_target)) {
    return shoot_target;
  }

  if (auto pass_target = selectPassTarget(); pass_target.has_value()) {
    return pass_target.value();
  }

  return computeFallbackTarget();
}

bool FreeKicker::tryShoot(Point & out_target)
{
  const Point ball_pos = world_model()->ball().pos;
  auto [best_angle, goal_angle_width] =
    world_model()->getLargestAttackGoalAngleRangeFromPoint(ball_pos);

  // ヒステリシス: 前回シュートを選んだ場合は閾値を少し下げる
  double threshold = getParameter<double>("shoot_min_angle_rad");
  if (last_chose_shoot_) {
    threshold -= 0.5 * M_PI / 180.0;
  }

  if (goal_angle_width > threshold) {
    last_chose_shoot_ = true;
    double shoot_angle =
      GoalKick::getBestAngleToShootFromPoint(3.0, ball_pos, world_model(), visualizer);
    out_target = ball_pos + Vector2(std::cos(shoot_angle), std::sin(shoot_angle)) * 10.0;
    return true;
  }

  last_chose_shoot_ = false;
  return false;
}

std::optional<Point> FreeKicker::selectPassTarget()
{
  const auto & wm = world_model();
  const Point ball_pos = wm->ball().pos;
  const auto enemies = wm->theirs().robotsWhere().available().get();

  double best_enemy_slack = -100.0;
  for (const auto & s : wm->getMsg().game_analysis.their_slack) {
    best_enemy_slack = std::max(best_enemy_slack, static_cast<double>(s.min.slack_time));
  }

  auto teammates = wm->ours().robotsWhere().available().excludeGoalie().get();

  // 自陣ロボ（敵半面にいないロボット）はパス先候補から除外
  const double our_sign = wm->getOurSideSign();
  teammates.erase(
    std::remove_if(
      teammates.begin(), teammates.end(),
      [&](const auto & r) { return r->id == robot()->id || r->pose.pos.x() * our_sign > 0.0; }),
    teammates.end());

  double best_score = FK_MIN_PASS_ACCEPT_SCORE;
  std::optional<uint8_t> best_id;
  Point best_pos = Point::Zero();

  for (const auto & teammate : teammates) {
    double score = calculatePassScore(teammate->pose.pos, enemies, best_enemy_slack);

    if (last_pass_receiver_id_ && teammate->id == last_pass_receiver_id_.value()) {
      score /= FK_PASS_HYSTERESIS_RATIO;
    }

    if (score > best_score) {
      best_score = score;
      best_id = teammate->id;
      best_pos = teammate->pose.pos;
    }
  }

  if (best_id) {
    last_pass_receiver_id_ = best_id;
    auto pass_analysis =
      getPassAnalysis(ball_pos, best_pos, enemies, getParameter<double>("pass_obstacle_distance"));
    use_chip_ = pass_analysis.need_chip;
    if (use_chip_) {
      chip_distance_ = pass_analysis.required_chip_distance + 0.2;
    }
    return best_pos;
  }

  return std::nullopt;
}

double FreeKicker::calculatePassScore(
  const Point & target, const std::vector<std::shared_ptr<RobotInfo>> & enemies,
  double best_enemy_slack) const
{
  const auto & wm = world_model();
  const Point ball_pos = wm->ball().pos;
  const double pass_distance = (target - ball_pos).norm();
  const double pass_min = getParameter<double>("pass_min_distance");
  const double pass_max = getParameter<double>("pass_max_distance");

  if (pass_distance < pass_min || pass_distance > pass_max) return 0.0;

  double score = 1.0;

  if (target.x() * wm->getAttackSideSign() > 0.0) {
    score *= 1.5;
  } else {
    score *= getParameter<double>("pass_defensive_half_penalty");
  }

  // ボール近傍の敵はチップキックで飛び越せるためブロッカーから除外
  const double chip_bypass_dist = getParameter<double>("pass_chip_bypass_distance");
  std::vector<std::shared_ptr<RobotInfo>> far_enemies;
  for (const auto & e : enemies) {
    if ((e->pose.pos - ball_pos).norm() >= chip_bypass_dist) {
      far_enemies.push_back(e);
    }
  }
  Segment pass_line{ball_pos, target};
  if (
    auto nearest_enemy = wm->getNearestRobotWithDistanceFromSegment(pass_line, far_enemies);
    nearest_enemy.has_value()) {
    if (nearest_enemy->distance < getParameter<double>("pass_obstacle_distance")) return 0.0;
    score *= std::clamp(nearest_enemy->distance / 2.0, 0.0, 1.5);
  }

  auto [best_angle, recv_goal_w] = wm->getLargestAttackGoalAngleRangeFromPoint(target);
  score += std::clamp(recv_goal_w / (M_PI / 12.0), 0.0, 0.5);

  const Point attack_goal = wm->getAttackGoalCenter();
  double dist_ball_to_goal = (ball_pos - attack_goal).norm();
  double dist_target_to_goal = (target - attack_goal).norm();
  double normed = (dist_target_to_goal - wm->fieldSize().x() * 0.5) / (wm->fieldSize().x() * 0.5);
  score *= std::clamp(1.0 - normed, 0.3, 1.5);
  // パスによりボールが敵ゴールから遠ざかる場合はペナルティ
  if (dist_target_to_goal > dist_ball_to_goal) {
    score *= std::clamp(dist_ball_to_goal / dist_target_to_goal, 0.3, 1.0);
  }

  // 敵の到達が受け手より enemy_slack_threshold 秒以上速い場合は減点
  // (受け手への到達時間の代理値として pass_distance / 2.0 を使用)
  if (best_enemy_slack > -(pass_distance / 2.0) + getParameter<double>("enemy_slack_threshold")) {
    score *= 0.3;
  }

  return score;
}

Point FreeKicker::computeFallbackTarget()
{
  const Point ball_pos = world_model()->ball().pos;
  double sign = world_model()->getAttackSideSign();
  double field_half_x = world_model()->fieldSize().x() * 0.5;
  double field_half_y = world_model()->fieldSize().y() * 0.5;

  Point fallback;
  fallback.x() = sign * field_half_x * 0.4;
  fallback.y() = std::clamp(ball_pos.y(), -field_half_y * 0.3, field_half_y * 0.3);

  use_chip_ = true;
  chip_distance_ =
    std::clamp((fallback - ball_pos).norm(), 1.0, getParameter<double>("pass_max_distance"));
  return fallback;
}
}  // namespace crane::skills
