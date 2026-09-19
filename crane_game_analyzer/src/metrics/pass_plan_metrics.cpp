// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/pass_plan_metrics.hpp"

#include <algorithm>
#include <chrono>
#include <crane_geometry/ddps.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/pass_plan.hpp>
#include <crane_msg_wrappers/pass_rating.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <crane_physics/travel_time.hpp>
#include <format>
#include <vector>

#include "crane_game_analyzer/metrics/pass_origin.hpp"

namespace crane::metrics
{
namespace
{
/// 計画の出し手がボールの保持を続けているか。
/// 他の味方が明確に（kKickerHoldMargin 以上）近づいたら手放したとみなす。
/// 計画の受け手以外の味方が、経路上のどこかでボールに先着できるか。
///
/// 計画は「この受け手が受け取る」という契約なので、他の味方が先に触れる点は
/// 採用しない。実測では、キック較正を直したあとの失敗の最大要因がこれだった
/// （15試行中4件）。ボールは計画どおりの地点に届く（受領点誤差 0.03〜0.75m）のに、
/// そこへ来たのが計画の受け手ではない、という形で契約が破れる。
/// 受領点そのものの競合だけでなく、経路を横切って途中で触ってしまう場合も含むので、
/// 敵の迎撃評価と同じ「経路全体で先着できるか」を味方にも適用する。
///
/// 経路のどこまでを「途中で横切る」と見なすかの、受領点手前の余裕 [m]。
///
/// 終端そのものは別の基準で見る（下の receiver_travel_time 比較）。ここで
/// 終端まで含めてボール到達時刻と比べると、受領点の近くに味方が立っている
/// だけで候補が消える。実測では 563 候補中 427 がこれで落ち、計画が成立
/// しないまま Attacker がクリアを蹴る試行が並んだ。値はロボット直径 0.18m に
/// 制御半径ぶんを足した程度。
constexpr double kReceivePointClearance = 0.6;

/// 計画の受け手より先に、別の味方がボールへ届いてしまう受領点かどうか。
///
/// 見る観点は2つある。
///
/// 1. **経路を途中で横切る味方**。ボールより先にパスラインへ到達できるなら、
///    受領点まで届く前に触られる。
/// 2. **受領点で先に待っている味方**。ここはボール到達時刻ではなく
///    「計画の受け手より先に着くか」で見る。ボールと比べると、受け手自身が
///    間に合う地点でも近くの味方が居るだけで落ちてしまい、候補が枯れる。
///    受け手より近い味方が居るなら、その点はその味方に割り当てられるべきで、
///    実際その味方を受け手とする候補として別途評価される。
///
/// 候補点ごとに全味方を評価すると重いので、経路までの距離で先に切る。
/// 飛行時間内に届き得ない味方は最初から見ない。
auto friendlyWouldSteal(
  const WorldModelWrapper & wm, const Point & origin, const Point & target,
  const StraightPassFlight & flight, int kicker_id, int receiver_id, uint8_t goalie_id,
  const ReceiveFeasibilityParams & params, double receiver_travel_time) -> bool
{
  const double pass_distance = (target - origin).norm();
  const double ball_time =
    rollingTravelTime(pass_distance, flight.initial_speed, flight.deceleration);
  if (!std::isfinite(ball_time) || ball_time <= 0.0) {
    return false;
  }
  // 停止して到達する台形プロファイルで、この時間に覆える最大距離。
  // 実際の到達判定より甘めに見積もる（甘い側で切り捨てても取りこぼさない）。
  const double max_reach = 0.25 * params.receiver_max_acceleration * ball_time * ball_time;
  const Segment pass_line{origin, target};
  for (const auto & robot : wm.ours().robotsWhere().available().get()) {
    const int id = static_cast<int>(robot->id);
    if (id == kicker_id || id == receiver_id || robot->id == goalie_id) {
      continue;
    }
    if (getClosestPointAndDistance(robot->pose.pos, pass_line).distance > max_reach) {
      continue;
    }
    if (
      straightPassInterceptionSlack(
        origin, target, flight, robot->pose.pos, robot->vel.linear,
        params.receiver_max_acceleration, params.receiver_max_velocity,
        PassPathRange{0.0, pass_distance - kReceivePointClearance}) <= 0.0) {
      return true;
    }
    // 受領点に受け手より先（同着を含む）に着ける味方が居れば、そこは渡らない。
    const double friend_time = getTravelTimeTrapezoidal(
      robot->pose.pos, robot->vel.linear, target, params.receiver_max_acceleration,
      params.receiver_max_velocity);
    if (std::isfinite(friend_time) && friend_time <= receiver_travel_time) {
      return true;
    }
  }
  return false;
}

auto kickerKeepsBall(const WorldModelWrapper & wm, int kicker_id) -> bool
{
  constexpr double kKickerHoldMargin = 0.5;
  if (kicker_id < 0) {
    return false;
  }
  const auto kicker = wm.getOurRobot(static_cast<uint8_t>(kicker_id));
  if (!kicker) {
    return false;
  }
  const double kicker_distance = kicker->getDistance(wm.ball().pos);
  for (const auto & robot : wm.ours().robotsWhere().available().excludeGoalie().get()) {
    if (static_cast<int>(robot->id) == kicker_id) {
      continue;
    }
    if (robot->getDistance(wm.ball().pos) + kKickerHoldMargin < kicker_distance) {
      return false;
    }
  }
  return true;
}
}  // namespace

namespace
{
/// 計画の出し手がボールを保持しているとみなす距離 [m]。
/// ロボット半径 0.09 + ドリブラー前方の余裕。これより近ければ、
/// ボールが動いていても「運んでいる」と判断して計画を保持する。
constexpr double kCarryDistance = 0.25;
}  // namespace

PassPlanMetric::PassPlanMetric() : MetricBase(MetricId::PASS_PLAN, "PassPlan") {}

auto PassPlanMetric::compute(MetricContext & ctx) -> void
{
  // デシメーション: 重い再計算は recompute_interval_sec_ 間隔でのみ実施
  const auto now = ctx.clock->now();
  if (selection_clock_ != ctx.clock) {
    selection_clock_ = ctx.clock;
    receiver_hysteresis_ = SelectionHysteresis<int>(
      {.min_hold_duration_sec = 0.5, .min_improvement_ratio = 0.2}, ctx.clock);
  }
  if (last_recompute_time_ && now < *last_recompute_time_) {
    last_recompute_time_.reset();
    flight_started_at_.reset();
    writeInactivePlan(-1);
  }
  auto & wm = *ctx.world_model;
  using Plan = crane_msgs::msg::PassPlan;
  const bool inplay =
    wm.getMsg().play_situation.command.value == crane_msgs::msg::PlaySituation::INPLAY;
  const bool usable = isUsablePassPlan(cached_plan_, wm);
  const Vector2 pass_direction =
    Point(cached_plan_.receive_point.x, cached_plan_.receive_point.y) - planned_origin_;
  const bool aligned_motion =
    usable && wm.ball().vel.norm() > 0.5 && pass_direction.norm() > 0.1 &&
    pass_direction.normalized().dot(wm.ball().vel.normalized()) > std::cos(0.35);
  bool matching_kick = false;
  if (usable && !ctx.analysis.ongoing_kick.empty()) {
    const auto & kick = ctx.analysis.ongoing_kick.front();
    matching_kick = kick.is_kicker_friend && kick.kicker_id == cached_plan_.kicker_id &&
                    (Point(kick.origin_x, kick.origin_y) - planned_origin_).norm() < 0.8;
  }
  // キック検出器の履歴確定まで短い猶予を設ける。初検出時のkick.directionは未確定。
  const bool detection_grace =
    usable && ctx.analysis.ongoing_kick.empty() &&
    (flight_started_at_ ? (now - *flight_started_at_).seconds() < 0.3
                        : (wm.ball().pos - planned_origin_).norm() < 0.8 &&
                            wm.getOurRobot(static_cast<uint8_t>(cached_plan_.kicker_id))
                                ->getDistance(planned_origin_) < 0.5);
  if (wm.ball().detected && aligned_motion && (matching_kick || detection_grace)) {
    if (!flight_started_at_) {
      flight_started_at_ = now;
    }
    if ((now - *flight_started_at_).seconds() < cached_plan_.ball_travel_time + 1.0) {
      cached_plan_.state = Plan::STATE_BALL_IN_FLIGHT;
      ctx.analysis.pass_plan = cached_plan_;
      ctx.analysis.recommended_pass_receiver_id = cached_plan_.receiver_id;
      return;
    }
  }
  // ボールが動いていても、計画の出し手が運んでいる間は計画を解除しない。
  //
  // Attacker はキック前にボールを運んで体勢を整えるので、その間ボール速度は
  // 0.5 m/s をすぐ超える。ここで無条件に解除すると、出し手が蹴る判断をする
  // まさにその瞬間に計画が消え、パスではなくクリアが選ばれる。
  // 実測では、この解除のせいで計画の有効時間が INPLAY の約半分に留まり、
  // 9試行中5試行が「計画なしで蹴る」結果になっていた。
  //
  // 相手のキックやこぼれ球は出し手から離れるので、距離条件で区別できる。
  const bool carried_by_kicker = [&]() -> bool {
    if (!usable || cached_plan_.kicker_id < 0) {
      return false;
    }
    // 計画の出し手以外のキックが進行中なら、ボールが近くにあっても
    // 「運んでいる」ではない。相手に蹴られた場合はここで計画を手放す。
    if (!ctx.analysis.ongoing_kick.empty()) {
      const auto & kick = ctx.analysis.ongoing_kick.front();
      if (!kick.is_kicker_friend || kick.kicker_id != cached_plan_.kicker_id) {
        return false;
      }
    }
    const auto kicker = wm.getOurRobot(static_cast<uint8_t>(cached_plan_.kicker_id));
    return kicker && kicker->getDistance(wm.ball().pos) < kCarryDistance;
  }();
  if (!inplay || !wm.ball().detected || (wm.ball().isMoving(0.5) && !carried_by_kicker)) {
    writeInactivePlan(-1);
    last_recompute_time_.reset();
    ctx.analysis.pass_plan = cached_plan_;
    return;
  }
  // 出し手の推薦が変わっても、計画を保持している間は追従しない。
  // AttackerMetric のスコアは `10.0 / 到達距離` に二値条件の乗算が掛かる構造で、
  // 実測では推薦が 1.0〜1.6 秒ごとに入れ替わる。そのたびに再計算して
  // kicker_id を差し替えると plan_id が変わり、受領点が跳ぶ
  // （実測: 1.49 秒の PLANNING 継続中に受け手が 1→10、受領点が 3.0m 移動）。
  // 受け手は先回りする先を決められず、パスが成立しない。
  // 計画自体が出し手の割当を固定する（AttackerSkillSession は計画の kicker を
  // 最優先で選ぶ）ので、ここで追従しないことが役割の安定にもつながる。
  // ボールを手放したとき（他の味方が明確に近い）は追従する。
  const bool follow_attacker_change =
    ctx.analysis.recommended_attacker_id >= 0 &&
    cached_plan_.kicker_id != ctx.analysis.recommended_attacker_id &&
    (!usable || !kickerKeepsBall(wm, cached_plan_.kicker_id));
  const bool due = !last_recompute_time_.has_value() ||
                   (cached_plan_.state != Plan::STATE_INACTIVE && !usable) || flight_started_at_ ||
                   follow_attacker_change ||
                   (now - last_recompute_time_.value()).seconds() >= recompute_interval_sec_;
  if (due) {
    last_recompute_time_ = now;
    const auto t0 = std::chrono::steady_clock::now();
    recomputePlan(ctx);
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // 再計算1回あたりの所要時間と評価候補数を間引きログに記録
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("PassPlanMetric"), *ctx.clock, 5000,
      "recompute %.2f ms, 評価候補 %d (feasibility棄却 %d, 味方横取り棄却 %d, スコア棄却 %d, "
      "最良スコア %.2f / 下限 %.2f), "
      "最良点(%.2f,%.2f) 内訳[距離 %.2f, ゴール角+%.2f, 自ゴール-%.2f, 敵ゴール %.2f, 迎撃 %.2f, "
      "遮蔽 %.2f], state=%u",
      elapsed_ms, last_evaluated_, last_rejected_infeasible_, last_rejected_friendly_,
      last_rejected_low_score_, last_best_score_, min_pass_score_, last_best_point_.x(),
      last_best_point_.y(), last_best_rating_.distance_factor, last_best_rating_.goal_angle_bonus,
      last_best_rating_.own_goal_penalty, last_best_rating_.their_goal_factor,
      last_best_rating_.intercept_score, last_best_rating_.shadow_score,
      static_cast<unsigned>(cached_plan_.state));
  }
  // 間引き有無に関わらず、キャッシュ済みプランを毎フレーム完全書込
  // （未書込フィールドの int 既定値0による幻ロボット参照を防ぐ）
  ctx.analysis.pass_plan = cached_plan_;
  if (cached_plan_.state == Plan::STATE_PLANNING) {
    ctx.analysis.recommended_pass_receiver_id = cached_plan_.receiver_id;
  }
}

auto PassPlanMetric::writeInactivePlan(int kicker_id, bool keep_selection) -> void
{
  if (!keep_selection) {
    receiver_hysteresis_.reset();
    held_receiver_id_ = -1;
    held_receive_point_.reset();
  }
  flight_started_at_.reset();
  crane_msgs::msg::PassPlan plan;  // 既定構築（数値0）
  plan.plan_id = plan_seq_;
  plan.state = crane_msgs::msg::PassPlan::STATE_INACTIVE;
  plan.kicker_id = kicker_id;  // -1 のこともある
  plan.receiver_id = -1;       // 幻ロボット0を避けるため明示的に -1
  plan.is_chip = false;
  cached_plan_ = plan;
}

auto PassPlanMetric::recomputePlan(MetricContext & ctx) -> void
{
  auto & wm = *ctx.world_model;
  flight_started_at_.reset();
  feasibility_.ball_deceleration = wm.ball().getPhysicsModel()->getDeceleration();
  const Point pass_origin = computePassOrigin(ctx);
  planned_origin_ = pass_origin;

  // 出し手: 計画を保持している間はその出し手を維持する。推薦が揺れても
  // 契約を差し替えない（compute() の follow_attacker_change と同じ理由）。
  const bool keep_kicker = cached_plan_.state == crane_msgs::msg::PassPlan::STATE_PLANNING &&
                           isUsablePassPlan(cached_plan_, wm) &&
                           kickerKeepsBall(wm, cached_plan_.kicker_id);
  // それ以外は推奨アタッカー、無ければボール最近傍味方にフォールバック
  int kicker_id = keep_kicker ? cached_plan_.kicker_id : ctx.analysis.recommended_attacker_id;
  if (kicker_id < 0) {
    // GKの排出は専用の判断経路に任せる
    const auto nearest = wm.getNearestRobotWithDistanceFromPoint(
      wm.ball().pos, wm.ours().robotsWhere().available().excludeGoalie().get());
    kicker_id = nearest ? static_cast<int>(nearest->robot->id) : -1;
  }

  // スコア下限のヒステリシス。計画を保持している間は下限を緩め、
  // 閾値ぎわで計画が明滅するのを防ぐ（min_pass_score_release_ratio_ 参照）。
  const bool holding_plan = cached_plan_.state == crane_msgs::msg::PassPlan::STATE_PLANNING &&
                            isUsablePassPlan(cached_plan_, wm);
  const double accept_score =
    holding_plan ? min_pass_score_ * min_pass_score_release_ratio_ : min_pass_score_;

  const double side = wm.getOurSideSign();
  const auto goalie_id = wm.getOurGoalieId();
  const PassRatingConfig rating_cfg{
    .slack_scale = slack_scale_, .enemy_slack = enemy_slack_config_};
  const auto rate_point = [&](const Point & point, const ReceiveFeasibility & feasibility) {
    auto config = rating_cfg;
    config.straight_flight =
      StraightPassFlight{feasibility.kick_speed, feasibility_.ball_deceleration};
    return ratePassCandidate(&wm, pass_origin, point, config);
  };
  const auto legal_point = [&](const Point & point) {
    // isFieldInside の offset は正で外側へ広げるので、負を渡して内側へ狭める。
    return point.x() * side < 0.0 && !wm.point_checker.isPenaltyArea(point) &&
           wm.point_checker.isFieldInside(point, -receive_point_field_margin_);
  };

  // 受け手ごとの最良候補（点・スコア）を集める
  struct ReceiverBest
  {
    int id = -1;
    Point point = Point::Zero();
    double score = 0.0;
    bool valid = false;
  };
  std::vector<ReceiverBest> receiver_bests;
  int evaluated = 0;
  int rejected_infeasible = 0;
  int rejected_low_score = 0;
  int rejected_friendly = 0;
  double best_score_seen = 0.0;
  PassRating best_rating_seen{};
  Point best_point_seen = Point::Zero();
  bool capped = false;

  auto receivers = wm.ours().robotsWhere().available().excludeGoalie().get();
  std::erase_if(receivers, [&](const auto & receiver) {
    return receiver->id == goalie_id || static_cast<int>(receiver->id) == kicker_id;
  });
  for (const auto & receiver : receivers) {
    ReceiverBest best;
    best.id = static_cast<int>(receiver->id);
    receiver_bests.push_back(best);
  }

  // 全受け手の現在点を先に評価し、その後は近いリングから交互に探索する。
  // 予算を一人目のグリッドで使い切らず、遠い角度一方向への偏りも避ける。
  auto offsets = getDPPSPoints(Point::Zero(), dpps_r_resolution_, dpps_r_max_, dpps_theta_div_);
  std::stable_sort(offsets.begin(), offsets.end(), [](const Point & a, const Point & b) {
    return a.squaredNorm() < b.squaredNorm();
  });
  offsets.insert(offsets.begin(), Point::Zero());
  for (const auto & offset : offsets) {
    for (size_t i = 0; i < receivers.size(); ++i) {
      const auto & receiver = receivers[i];
      auto & best = receiver_bests[i];
      const Point point = receiver->pose.pos + offset;
      // フィルタ: 受領点が攻撃ハーフ・非PA・フィールド内
      if (!legal_point(point)) {
        continue;
      }
      if (evaluated >= max_candidates_) {
        capped = true;
        break;
      }
      ++evaluated;
      // feasibility ゲート（安価な閉形式で早期棄却）
      const auto feas = feasibleReceivePoint(
        pass_origin, point, receiver->pose.pos, receiver->vel.linear, feasibility_);
      if (!feas.feasible) {
        ++rejected_infeasible;
        continue;
      }
      // 味方の横取りゲート（採点より安価なので先に置く）
      if (
        friendlyWouldSteal(
          wm, pass_origin, point,
          StraightPassFlight{feas.kick_speed, feasibility_.ball_deceleration}, kicker_id,
          static_cast<int>(receiver->id), goalie_id, feasibility_, feas.receiver_travel_time)) {
        ++rejected_friendly;
        continue;
      }
      // 採点（重い: 敵 slack・遮蔽の評価を含む）
      const auto rating = rate_point(point, feas);
      const double score = rating.score;
      if (std::isfinite(score) && score > best_score_seen) {
        best_score_seen = score;
        best_rating_seen = rating;
        best_point_seen = point;
      }
      if (!std::isfinite(score) || score < accept_score) {
        ++rejected_low_score;
        continue;
      }
      if (!best.valid || score > best.score) {
        best.valid = true;
        best.point = point;
        best.score = score;
      }
    }
    if (capped) {
      break;
    }
  }
  std::erase_if(receiver_bests, [](const auto & best) { return !best.valid; });
  last_evaluated_ = evaluated;
  last_rejected_infeasible_ = rejected_infeasible;
  last_rejected_low_score_ = rejected_low_score;
  last_rejected_friendly_ = rejected_friendly;
  last_best_score_ = best_score_seen;
  last_best_rating_ = best_rating_seen;
  last_best_point_ = best_point_seen;

  if (capped) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("PassPlanMetric"), *ctx.clock, 2000,
      "候補評価数が上限 %d に達したため打ち切りました（受領点の一部が未評価）", max_candidates_);
  }

  // 有効候補なし → 非アクティブ
  if (receiver_bests.empty()) {
    writeInactivePlan(kicker_id, /*keep_selection=*/true);
    return;
  }

  // 全体最良の受け手
  const ReceiverBest * overall_best = &receiver_bests.front();
  for (const auto & rb : receiver_bests) {
    if (rb.score > overall_best->score) {
      overall_best = &rb;
    }
  }
  // スコアゲート
  if (overall_best->score < accept_score) {
    writeInactivePlan(kicker_id, /*keep_selection=*/true);
    return;
  }

  // 受け手選定（第1レベルヒステリシス）: 前回受け手の現行スコアと比較
  double prev_score = 0.0;
  if (const auto prev_id = receiver_hysteresis_.currentId(); prev_id.has_value()) {
    for (const auto & rb : receiver_bests) {
      if (rb.id == prev_id.value()) {
        prev_score = rb.score;
        break;
      }
    }
  }
  receiver_hysteresis_.shouldSwitch(overall_best->id, overall_best->score, prev_score);
  const int held_or_best_id = receiver_hysteresis_.currentId().value_or(overall_best->id);

  // ヒステリシスが保持する受け手の今フレーム候補を探す。保持受け手が今フレーム有効候補を
  // 持たなければ overall_best にフォールバックし、以降は sel->id を唯一の権威 id とする。
  // これを怠ると plan.receiver_id（保持ID）と receive_point（別ロボの点）が食い違う。
  const ReceiverBest * sel = overall_best;
  for (const auto & rb : receiver_bests) {
    if (rb.id == held_or_best_id) {
      sel = &rb;
      break;
    }
  }
  const int chosen_id = sel->id;
  if (chosen_id != held_or_best_id) {
    receiver_hysteresis_.forceSwitch(chosen_id);
  }

  // 受領点の第2レベル保持（同一受け手内で改善が閾値未満なら点を動かさない）
  // 下の全分岐で必ず代入するため、ここでは初期化しない（冗長代入 redundantAssignment を避ける）
  Point receive_point;
  if (held_receiver_id_ != chosen_id || !held_receive_point_.has_value()) {
    held_receiver_id_ = chosen_id;
    held_receive_point_ = sel->point;
    receive_point = sel->point;
  } else {
    // 保持点の現在の feasibility・スコアを再評価し、改善が閾値未満なら点を動かさない
    const auto held_receiver = wm.getOurRobot(static_cast<uint8_t>(chosen_id));
    const auto held_feas = feasibleReceivePoint(
      pass_origin, held_receive_point_.value(), held_receiver->pose.pos, held_receiver->vel.linear,
      feasibility_);
    const double held_score = held_feas.feasible && legal_point(*held_receive_point_)
                                ? rate_point(*held_receive_point_, held_feas).score
                                : 0.0;
    if (
      !held_feas.feasible || !legal_point(*held_receive_point_) || !std::isfinite(held_score) ||
      held_score < accept_score || sel->score >= held_score * (1.0 + receive_point_improvement_)) {
      held_receive_point_ = sel->point;
      receive_point = sel->point;
    } else {
      receive_point = held_receive_point_.value();
    }
  }

  // 選定プランの最終評価（msg 用に一貫した値を採取）
  const auto receiver = wm.getOurRobot(static_cast<uint8_t>(chosen_id));
  const auto feas = feasibleReceivePoint(
    pass_origin, receive_point, receiver->pose.pos, receiver->vel.linear, feasibility_);
  const auto rating = rate_point(receive_point, feas);

  // 世代: 受け手が変わった or 直近が非アクティブなら plan_id を進める
  if (
    cached_plan_.state != crane_msgs::msg::PassPlan::STATE_PLANNING ||
    cached_plan_.kicker_id != kicker_id || cached_plan_.receiver_id != chosen_id) {
    ++plan_seq_;
  }

  crane_msgs::msg::PassPlan plan;
  plan.plan_id = plan_seq_;
  plan.state = crane_msgs::msg::PassPlan::STATE_PLANNING;
  plan.kicker_id = kicker_id;
  plan.receiver_id = chosen_id;
  plan.receive_point.x = receive_point.x();
  plan.receive_point.y = receive_point.y();
  plan.receive_point.z = 0.0;
  plan.kick_speed = feas.kick_speed;
  plan.is_chip = false;  // この計画は直進パスのみを評価する
  plan.chip_distance = 0.0;
  plan.ball_travel_time = feas.ball_travel_time;
  plan.receiver_travel_time = feas.receiver_travel_time;
  plan.score = rating.score;
  plan.distance_factor = rating.distance_factor;
  plan.goal_angle_bonus = rating.goal_angle_bonus;
  plan.own_goal_penalty = rating.own_goal_penalty;
  plan.their_goal_factor = rating.their_goal_factor;
  plan.intercept_score = rating.intercept_score;
  plan.shadow_score = rating.shadow_score;
  cached_plan_ = plan;
}

auto PassPlanMetric::visualize(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & /*shared*/) -> void
{
  // 自前サブレイヤ analyzer/pass_plan に描画し自己 flush（共有 analyzer とは別レイヤ）
  const auto & plan = cached_plan_;
  if (isUsablePassPlan(plan, *ctx.world_model)) {
    const Point origin = computePassOrigin(ctx);
    const Point receive_point(plan.receive_point.x, plan.receive_point.y);

    // 新: PassPlan の受領点（リードパス可）
    viz_->drawLine(origin, receive_point, "cyan", 30, 0.9);
    viz_->drawStyledCircle(receive_point, 0.15, "none", 1.0, "cyan", 1.0, 20);
    viz_->drawText(
      receive_point + Point(0.15, 0.15),
      std::format("PLAN r{} v{:.1f}", plan.receiver_id, plan.kick_speed), "cyan", 90);

    // 旧: pass_target（受け手現在位置）を同時描画して新旧比較
    if (ctx.analysis.pass_target_id >= 0) {
      const auto old_receiver =
        ctx.world_model->getOurRobot(static_cast<uint8_t>(ctx.analysis.pass_target_id));
      viz_->drawStyledCircle(old_receiver->pose.pos, 0.18, "none", 1.0, "orange", 0.9, 15);
      viz_->drawText(
        old_receiver->pose.pos + Point(0.2, -0.2),
        std::format("target r{}", ctx.analysis.pass_target_id), "orange", 80);
    }
  }
  viz_->flush();
}

}  // namespace crane::metrics
