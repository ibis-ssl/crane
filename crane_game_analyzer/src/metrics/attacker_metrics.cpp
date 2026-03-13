// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/attacker_metrics.hpp"

#include <algorithm>

namespace crane::metrics
{

// AttackerCandidateMetric実装

AttackerCandidateMetric::AttackerCandidateMetric()
: MetricBase(MetricId::ATTACKER_CANDIDATE, "AttackerCandidate")
{
}

auto AttackerCandidateMetric::compute(MetricContext & ctx) -> void
{
  const auto & ball_pos = ctx.world_model->ball().pos;
  const auto available_robots =
    ctx.world_model->ours().robotsWhere().available().excludeGoalie().get();

  if (available_robots.empty()) {
    ctx.analysis.recommended_attacker_id = -1;
    ctx.analysis.attacker_suitability_score = 0.0;
    ctx.analysis.recommended_pass_receiver_id = -1;
    return;
  }

  // 各ロボットの適性スコアを計算
  // スコア = ボール距離の逆数 × Slack時間の逆数
  struct RobotScore
  {
    uint8_t id;
    double score;
  };

  // 敵チームの中で最も有利なslack（ループ外で1回だけ計算）
  constexpr double ENEMY_SLACK_SCORE_THRESHOLD = 0.2;  // スコア減算用: 敵がこの秒数以上早いと不利
  double best_enemy_slack = -100.0;
  for (const auto & ts : ctx.analysis.their_slack) {
    if (ts.min.slack_time > best_enemy_slack) {
      best_enemy_slack = ts.min.slack_time;
    }
  }

  std::vector<RobotScore> robot_scores;

  for (const auto & robot : available_robots) {
    double distance = (robot->pose.pos - ball_pos).norm();

    // Slack情報を取得
    double metric_distance = distance;
    bool has_valid_intercept = false;
    double my_slack = -100.0;

    for (const auto & slack : ctx.analysis.our_slack) {
      if (slack.id == robot->id) {
        // min.slack_time > 0 なら有効なインターセプト地点が存在する
        if (slack.min.slack_time > 0.001) {
          Point intercept_pos(slack.min.x, slack.min.y);
          // インターセプト地点までの距離を指標とする
          metric_distance = (robot->pose.pos - intercept_pos).norm();
          has_valid_intercept = true;
        }
        my_slack = slack.min.slack_time;
        break;
      }
    }

    // スコア計算
    // 到達距離が短いほど高スコア
    double score = 10.0 / std::max(metric_distance, 0.1);

    // インターセプト計算ができなかった（ボールに追いつけない等）場合はスコアを大幅に下げる
    if (!has_valid_intercept) {
      score *= 0.3;  // 0.5 -> 0.3に強化（インターセプト不可能なロボットの優先度を下げる）
    }

    // 敵slackとの比較: 敵が先にボールに到達できる場合はスコアを大幅に下げる
    if (best_enemy_slack > my_slack + ENEMY_SLACK_SCORE_THRESHOLD) {
      score *= 0.2;
    }

    double total_score = score;

    // EMAでスムージング
    auto it = ema_scores_.find(robot->id);
    double smoothed_score;
    if (it == ema_scores_.end()) {
      // 初回は生スコアをそのまま使用
      ema_scores_[robot->id] = total_score;
      smoothed_score = total_score;
    } else {
      // EMA更新: smoothed = α * new + (1-α) * old
      smoothed_score = EMA_ALPHA * total_score + (1.0 - EMA_ALPHA) * it->second;
      ema_scores_[robot->id] = smoothed_score;
    }

    robot_scores.push_back({robot->id, smoothed_score});

    // デバッグログ: 各ロボットのスコア
    RCLCPP_DEBUG(
      rclcpp::get_logger("AttackerMetric"),
      "Robot %d: raw_score=%.2f, smoothed=%.2f, has_intercept=%d, distance=%.2f", robot->id, score,
      smoothed_score, has_valid_intercept, metric_distance);
  }

  // スコアでソート（降順）
  std::sort(robot_scores.begin(), robot_scores.end(), [](const auto & a, const auto & b) {
    return a.score > b.score;
  });

  int best_id = robot_scores[0].id;
  double best_score = robot_scores[0].score;

  // 現在選択中ロボットのスコアを取得するラムダ
  auto find_current_score = [&](int id) -> double {
    for (const auto & rs : robot_scores) {
      if (static_cast<int>(rs.id) == id) return rs.score;
    }
    return 0.0;
  };

  // ヒステリシス処理
  const auto prev_id = attacker_hysteresis_.currentId();
  const double current_score = find_current_score(prev_id.value_or(-1));
  const double time_before_switch = attacker_hysteresis_.timeSinceSwitch();
  const bool switched = attacker_hysteresis_.shouldSwitch(best_id, best_score, current_score);

  if (switched) {
    if (prev_id.has_value()) {
      double improvement_ratio = (best_score - current_score) / std::max(current_score, 0.1);
      RCLCPP_INFO(
        rclcpp::get_logger("AttackerMetric"),
        "SWITCH: %d -> %d (old_score=%.2f, new_score=%.2f, improvement=%.1f%%, time=%.2fs)",
        *prev_id, best_id, current_score, best_score, improvement_ratio * 100, time_before_switch);
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("AttackerMetric"), "INITIAL: selected robot %d (score=%.2f)", best_id,
        best_score);
    }
  } else if (prev_id.has_value() && best_id != *prev_id) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("AttackerMetric"),
      "HOLD: robot %d (best=%d, best_score=%.2f, current=%.2f, time=%.2fs)", *prev_id, best_id,
      best_score, current_score, attacker_hysteresis_.timeSinceSwitch());
  }

  ctx.analysis.recommended_attacker_id = attacker_hysteresis_.currentId().value_or(-1);
  ctx.analysis.attacker_suitability_score = best_score;

  // recommended_pass_receiver_id: Attacker以外で最もスコアの高いロボットを推薦
  ctx.analysis.recommended_pass_receiver_id = -1;
  {
    int selected_attacker = ctx.analysis.recommended_attacker_id;
    for (const auto & rs : robot_scores) {
      if (static_cast<int>(rs.id) != selected_attacker) {
        ctx.analysis.recommended_pass_receiver_id = rs.id;
        break;
      }
    }
  }
}

}  // namespace crane::metrics
