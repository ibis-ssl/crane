// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/pass.hpp>
#include <crane_robot_skills/free_kicker.hpp>
#include <crane_robot_skills/goal_kick.hpp>
#include <magic_enum/magic_enum.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace crane::skills
{
namespace
{
constexpr double FK_KICK_DETECT_VEL = 0.7;
constexpr double FK_KICK_DIRECTION_COS_THRESHOLD = 0.7;
constexpr double FK_APPROACH_PHASE_TIMEOUT = 6.0;
constexpr double FK_ALIGN_WAIT_SEC = 1.0;
constexpr double FK_KICK_TIMEOUT_SEC = 3.0;
constexpr double FK_MIN_PASS_ACCEPT_SCORE = 0.4;
constexpr double FK_PASS_HYSTERESIS_RATIO = 0.85;
// フィールド余裕の計算に使う実機半径。RobotInfo::geometry() は 0.06 で実機より小さい
constexpr double FK_ROBOT_RADIUS = 0.09;
// SSL 公式球の半径（直径 43mm）
constexpr double FK_BALL_RADIUS = 0.0215;
// APPROACH 目標を最終 standoff に固定してよい条件: 自機から最終 standoff への直線がボール中心から
// これ以上離れていること。ボール回避を無効にしているので、直線上でボールに触れない保証が要る。
// 角度（ボール後方 ±45°）で判定すると、ライン際でフィールド内に収めた周回目標からは満たせず、
// 6 秒タイムアウトで APPROACH をやり直し続ける
constexpr double FK_APPROACH_BALL_CLEARANCE = FK_ROBOT_RADIUS + FK_BALL_RADIUS + 0.02;

auto segmentClearsBall(const Point & from, const Point & to, const Point & ball) -> bool
{
  return getClosestPointAndDistance(ball, Segment(from, to)).distance >= FK_APPROACH_BALL_CLEARANCE;
}
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
  approach_entry_time_ = std::nullopt;
  align_entry_time_ = std::nullopt;
  align_target_locked_ = Point::Zero();
  kick_actually_launched_ = false;
  approach_final_latched_ = false;
  latched_ball_pos_ = Point::Zero();
  latched_standoff_ = Point::Zero();
}

void FreeKicker::initialize()
{
  setParameter("approach_max_velocity", 1.5);
  setParameter("align_max_velocity", 0.3);
  setParameter("kick_max_velocity", 0.5);
  setParameter("approach_distance", 0.15);
  setParameter("approach_position_tolerance", 0.05);
  // ALIGN へ進んでよい速度上限。0.8 m/s で突入すると位置制御器が 0.09 m 過走してボールを小突く
  // （2026-09-20 bag）。0.3 m/s なら減速度 3 m/s^2 で過走 0.015 m に収まる
  setParameter("approach_exit_speed", 0.3);
  // 最終 standoff からこの距離以内で、そこへの直線がボールに触れなければ目標を固定する。
  // 周回半径 0.30 の上でボール後方 80° にいても届くよう 0.40（機体〜standoff 約 0.31）
  setParameter("approach_final_latch_distance", 0.40);
  // ラッチ後にボールがこれ以上動いたら解除して再判定する（固定 standoff がボールを横切らないため）
  setParameter("approach_relatch_ball_move", 0.05);
  // フィールドラインからロボット中心までに残す余裕（ロボット半径に加算）
  setParameter("field_margin", 0.05);
  setParameter("target_kick_speed", 5.0);
  setParameter("target_chip_distance", 2.5);
  setParameter("shoot_min_angle_rad", deg2rad(6.0));
  setParameter("pass_obstacle_distance", 0.2);
  setParameter("pass_min_distance", 1.5);
  setParameter("pass_max_distance", 6.0);
  setParameter("enemy_slack_threshold", 0.3);
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
      // standoff_ を安定させるため APPROACH 開始の 1 フレーム目でロック
      kick_target_ = selectKickTarget();
      target_locked_ = true;
    }

    const Point ball_pos = world_model()->ball().pos;
    const double interval = getParameter<double>("approach_distance");
    const Vector2 kick_dir = kickDirection(ball_pos);
    // ALIGN の align_target_locked_ と同じ式。ラッチ後はここへ終端速度 0 で減速させる
    const Point final_standoff = ball_pos - kick_dir * interval;

    if (
      approach_final_latched_ &&
      (ball_pos - latched_ball_pos_).norm() > getParameter<double>("approach_relatch_ball_move")) {
      // ボールが動いた。固定 standoff へ直進すると新しいボール位置を横切りかねないので再判定
      approach_final_latched_ = false;
    }
    if (!approach_final_latched_) {
      const Point robot_pos = robot()->pose.pos;
      const bool near_final =
        (robot_pos - final_standoff).norm() < getParameter<double>("approach_final_latch_distance");
      // 自機位置に依存して動く周回目標の追いかけ回しを止める。最終 standoff への直線が
      // ボールに触れないことを幾何で確かめる（APPROACH はボール回避を無効にしている）
      if (near_final && segmentClearsBall(robot_pos, final_standoff, ball_pos)) {
        approach_final_latched_ = true;
        latched_ball_pos_ = ball_pos;
        latched_standoff_ = final_standoff;
      }
    }

    if (approach_final_latched_) {
      standoff_ = latched_standoff_;
    } else {
      // 周回半径はフィールド境界までの余裕で上限を切り、standoff をフィールド内に保つ。
      // 以前は disableFieldBoundary() でプランナのクランプを外していたが、コーナーでは
      // 周回目標がタッチライン外へ出てロボットが境界へ向かう原因になっていた。
      standoff_ = keepStandoffInField(
        ball_pos, computeAroundBallApproachTargetDynamic(
                    ball_pos, kick_target_, robot()->pose.pos, interval,
                    orbitRadiusLimit(ball_pos, interval)));
    }

    command->setMaxVelocity("FreeKicker::APPROACH", getParameter<double>("approach_max_velocity"))
      .setTargetPosition(standoff_, 0.0)
      .lookAtFrom(kick_target_, ball_pos)
      .disableBallAvoidance()
      .disablePlacementAvoidance()
      .dribble(0.0)
      .setOmegaLimit(10.0);
    command->addPlanningFactor("FreeKickerApproachLatched", approach_final_latched_ ? "1" : "0");

    return Status::RUNNING;
  });
  // ラッチ済み（目標が固定）かつ到達かつ減速済みのときだけ ALIGN へ進む。
  // #1360 で振動対策として ALIGN 側の条件を外した経緯があるため、ALIGN から出る辺は増やさない
  addTransition(s(S::APPROACH), s(S::ALIGN), [this]() -> bool {
    return approach_final_latched_ &&
           command->getTargetDistance() < getParameter<double>("approach_position_tolerance") &&
           robot()->vel.linear.norm() < getParameter<double>("approach_exit_speed");
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
      // APPROACH 最終地点と整合させるため、進入時のボール位置を基準に1回だけ計算してロック
      const Point ball_pos = world_model()->ball().pos;
      align_target_locked_ =
        ball_pos - kickDirection(ball_pos) * getParameter<double>("approach_distance");
    }

    command->setTargetPosition(align_target_locked_)
      .lookAtFrom(kick_target_, world_model()->ball().pos)
      .setMaxVelocity("FreeKicker::ALIGN", getParameter<double>("align_max_velocity"))
      .disableBallAvoidance();

    return Status::RUNNING;
  });
  addTransition(s(S::ALIGN), s(S::KICK), [this]() -> bool {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::steady_clock;
    return align_entry_time_.has_value() &&
           duration_cast<duration<double>>(steady_clock::now() - *align_entry_time_).count() >=
             FK_ALIGN_WAIT_SEC;
  });

  addStateFunction(s(S::KICK), [this]() -> Status {
    if (!kick_started_) {
      kick_started_ = true;
      kick_started_time_ = std::chrono::steady_clock::now();
    }

    command
      ->setTargetPosition(
        world_model()->ball().pos + kickDirection(world_model()->ball().pos) * 0.1)
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
  // 同じ FINISH 遷移でも、ボールが意図方向に飛んだ場合は kick_actually_launched_=true、
  // タイムアウトで諦めた場合は false にして外部 (Session) から判別できるようにする。
  addTransition(s(S::KICK), s(S::FINISH), [this]() -> bool {
    if (!kick_started_) return false;

    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::steady_clock;
    double elapsed =
      duration_cast<duration<double>>(steady_clock::now() - kick_started_time_).count();
    if (elapsed > FK_KICK_TIMEOUT_SEC) {
      kick_actually_launched_ = false;
      return true;
    }

    double ball_speed = world_model()->ball().vel.norm();
    if (ball_speed > FK_KICK_DETECT_VEL) {
      Vector2 ball_vel_dir = world_model()->ball().vel.normalized();
      Vector2 intended_dir = kickDirection(world_model()->ball().pos);
      if (ball_vel_dir.dot(intended_dir) > FK_KICK_DIRECTION_COS_THRESHOLD) {
        kick_actually_launched_ = true;
        return true;
      }
    }
    return false;
  });

  addStateFunction(s(S::FINISH), [this]() -> Status {
    command->stopHere();
    return Status::SUCCESS;
  });
}

auto FreeKicker::fieldBoxWithMargin() const -> Box
{
  const double margin = FK_ROBOT_RADIUS + getParameter<double>("field_margin");
  const Point half = world_model()->fieldSize() * 0.5;
  return Box(
    Point(-half.x() + margin, -half.y() + margin), Point(half.x() - margin, half.y() - margin));
}

auto FreeKicker::orbitRadiusLimit(const Point & ball, double interval) const -> double
{
  // ボールから余裕付き境界までの距離を周回半径の上限にする。半径 <= room なら周回点は必ず箱の中。
  // 下限 interval*2 は、追従遅れで目標がボール側へ寄ってもロボットがボールに触れないための余裕で、
  // それを超えた分は keepStandoffInField のクランプで吸収する。
  const Box box = fieldBoxWithMargin();
  const double room = std::min(
    {ball.x() - box.min_corner().x(), box.max_corner().x() - ball.x(),
     ball.y() - box.min_corner().y(), box.max_corner().y() - ball.y()});
  return std::clamp(room, interval * 2.0, interval * 4.0);
}

auto FreeKicker::kickDirection(const Point & ball) const -> Vector2
{
  const Vector2 kick_dir = kick_target_ - ball;
  if (kick_dir.squaredNorm() < 1e-12) {
    // kick_target_ がボールと一致する退行ケース。normalized() の NaN を避けて攻撃方向に倒す
    return Vector2(world_model()->getAttackSideSign(), 0.0);
  }
  return kick_dir.normalized();
}

auto FreeKicker::keepStandoffInField(const Point & ball, const Point & standoff) const -> Point
{
  // 箱へ単純にクランプすると、ライン際のボール（配置位置は 0.2 m でロボット余裕 0.14 m より
  // 内側に 0.06 m しかない）に対して standoff がボールへ寄り、ボール回避無効の APPROACH で
  // ボールを押してしまう。ボールからの距離を保ったまま、周回円上で箱に入る最寄りの点へ滑らせる。
  // 周回円が箱と交わらない（ボール自体が箱の外）ときは箱へクランプし、プランナ側の境界クランプに任せる
  return slideOntoCircleInsideBox(ball, standoff, fieldBoxWithMargin());
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
    threshold -= deg2rad(0.5);
  }

  if (goal_angle_width > threshold) {
    last_chose_shoot_ = true;
    double shoot_angle =
      GoalKick::getBestAngleToShootFromPoint(3.0, ball_pos, world_model(), visualizer);
    out_target = ball_pos + getNormVec(shoot_angle) * 10.0;
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
    use_chip_ = true;
    if (use_chip_) {
      if (pass_analysis.required_chip_distance > 0.5) {
        chip_distance_ = pass_analysis.required_chip_distance + 0.2;
      } else {
        chip_distance_ = 1.0;
      }
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
