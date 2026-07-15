// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/pass_plan_metrics.hpp"

#include <algorithm>
#include <chrono>
#include <crane_geometry/ddps.hpp>
#include <crane_msg_wrappers/pass_rating.hpp>
#include <format>
#include <vector>

#include "crane_game_analyzer/metrics/pass_origin.hpp"

namespace crane::metrics
{

PassPlanMetric::PassPlanMetric() : MetricBase(MetricId::PASS_PLAN, "PassPlan") {}

auto PassPlanMetric::compute(MetricContext & ctx) -> void
{
  // デシメーション: 重い再計算は recompute_interval_sec_ 間隔でのみ実施
  const auto now = ctx.clock->now();
  const bool due = !last_recompute_time_.has_value() ||
                   (now - last_recompute_time_.value()).seconds() >= recompute_interval_sec_;
  if (due) {
    last_recompute_time_ = now;
    const auto t0 = std::chrono::steady_clock::now();
    recomputePlan(ctx);
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // コスト可視化: 再計算1回あたりの所要時間と評価候補数を間引きログ（M3 前の実測用）
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("PassPlanMetric"), *ctx.clock, 5000,
      "recompute %.2f ms, 評価候補 %d, state=%u", elapsed_ms, last_evaluated_,
      static_cast<unsigned>(cached_plan_.state));
  }
  // 間引き有無に関わらず、キャッシュ済みプランを毎フレーム完全書込
  // （未書込フィールドの int 既定値0による幻ロボット参照を防ぐ）
  ctx.analysis.pass_plan = cached_plan_;
}

auto PassPlanMetric::writeInactivePlan(int kicker_id) -> void
{
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
  const Point pass_origin = computePassOrigin(ctx);

  // 出し手: 推奨アタッカー、無ければボール最近傍味方にフォールバック
  int kicker_id = ctx.analysis.recommended_attacker_id;
  if (kicker_id < 0) {
    // フォールバックでも goalie は出し手候補から除外（シャドー可視化を素直に保つ）
    const auto nearest = wm.getNearestRobotWithDistanceFromPoint(
      wm.ball().pos, wm.ours().robotsWhere().available().excludeGoalie().get());
    kicker_id = nearest ? static_cast<int>(nearest->robot->id) : -1;
  }

  const double side = wm.getOurSideSign();
  const auto goalie_id = wm.getOurGoalieId();
  const PassRatingConfig rating_cfg{
    .slack_scale = slack_scale_, .enemy_slack = enemy_slack_config_};

  // 受け手ごとの最良候補（点・スコア）を集める
  struct ReceiverBest
  {
    int id = -1;
    Point point;
    double score = 0.0;
    bool valid = false;
  };
  std::vector<ReceiverBest> receiver_bests;
  int evaluated = 0;
  bool capped = false;

  const auto receivers = wm.ours().robotsWhere().available().excludeGoalie().get();
  for (const auto & receiver : receivers) {
    if (receiver->id == goalie_id || static_cast<int>(receiver->id) == kicker_id) {
      continue;
    }
    ReceiverBest best;
    best.id = static_cast<int>(receiver->id);

    // 候補受領点: 現在位置 + 周辺グリッド（走り込み先＝スペースへのパスを許容）
    std::vector<Point> points;
    points.push_back(receiver->pose.pos);
    for (const auto & p :
         getDPPSPoints(receiver->pose.pos, dpps_r_resolution_, dpps_r_max_, dpps_theta_div_)) {
      points.push_back(p);
    }

    for (const auto & point : points) {
      // フィルタ: 受領点が攻撃ハーフ・非PA・フィールド内
      if (point.x() * side >= 0.0) {
        continue;
      }
      if (wm.point_checker.isPenaltyArea(point) || !wm.point_checker.isFieldInside(point)) {
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
        continue;
      }
      // 採点（重い: 敵 slack・遮蔽の評価を含む）
      const double score = ratePassCandidate(&wm, pass_origin, point, rating_cfg).score;
      if (!best.valid || score > best.score) {
        best.valid = true;
        best.point = point;
        best.score = score;
      }
    }
    if (best.valid) {
      receiver_bests.push_back(best);
    }
    if (capped) {
      break;
    }
  }
  last_evaluated_ = evaluated;

  if (capped) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("PassPlanMetric"), *ctx.clock, 2000,
      "候補評価数が上限 %d に達したため打ち切りました（受領点の一部が未評価）", max_candidates_);
  }

  // 有効候補なし → 非アクティブ
  if (receiver_bests.empty()) {
    receiver_hysteresis_.reset();
    held_receiver_id_ = -1;
    held_receive_point_.reset();
    writeInactivePlan(kicker_id);
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
  if (overall_best->score < min_pass_score_) {
    receiver_hysteresis_.reset();
    held_receiver_id_ = -1;
    held_receive_point_.reset();
    writeInactivePlan(kicker_id);
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
    const double held_score =
      held_feas.feasible
        ? ratePassCandidate(&wm, pass_origin, held_receive_point_.value(), rating_cfg).score
        : 0.0;
    if (!held_feas.feasible || sel->score >= held_score * (1.0 + receive_point_improvement_)) {
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
  const auto rating = ratePassCandidate(&wm, pass_origin, receive_point, rating_cfg);

  // 世代: 受け手が変わった or 直近が非アクティブなら plan_id を進める
  if (
    cached_plan_.state == crane_msgs::msg::PassPlan::STATE_INACTIVE ||
    cached_plan_.receiver_id != chosen_id) {
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
  plan.is_chip = false;  // M2 はチップ計画を行わない（M3 以降）
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
  if (plan.state == crane_msgs::msg::PassPlan::STATE_PLANNING && plan.receiver_id >= 0) {
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
