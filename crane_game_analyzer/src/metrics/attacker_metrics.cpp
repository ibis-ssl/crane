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
  const auto available_robots = ctx.world_model->ours().getAvailableRobots(255, true);

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

  std::vector<RobotScore> robot_scores;

  for (const auto & robot : available_robots) {
    double distance = (robot->pose.pos - ball_pos).norm();

    // Slack情報を取得
    double metric_distance = distance;
    bool has_valid_intercept = false;

    for (const auto & slack : ctx.analysis.our_slack) {
      if (slack.id == robot->id) {
        // min.slack_time > 0 なら有効なインターセプト地点が存在する
        if (slack.min.slack_time > 0.001) {
          Point intercept_pos(slack.min.x, slack.min.y);
          // インターセプト地点までの距離を指標とする
          metric_distance = (robot->pose.pos - intercept_pos).norm();
          has_valid_intercept = true;
        }
        break;
      }
    }

    // スコア計算
    // 到達距離が短いほど高スコア
    double score = 10.0 / std::max(metric_distance, 0.1);

    // インターセプト計算ができなかった（ボールに追いつけない等）場合はスコアを下げる
    if (!has_valid_intercept) {
      score *= 0.5;
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
  }

  // スコアでソート（降順）
  std::sort(robot_scores.begin(), robot_scores.end(), [](const auto & a, const auto & b) {
    return a.score > b.score;
  });

  int best_id = robot_scores[0].id;
  double best_score = robot_scores[0].score;

  // ヒステリシス処理
  auto current_time = ros_clock_.now();
  bool should_switch = false;

  if (last_attacker_id_ < 0) {
    // 初回は即座にスイッチ
    should_switch = true;
  } else if (best_id == last_attacker_id_) {
    // 同じロボットなら継続
    should_switch = false;
  } else {
    // 異なるロボットの場合
    double time_since_switch = (current_time - last_switch_time_).seconds();

    // 現在のロボットのスコアを取得
    double current_score = 0.0;
    for (const auto & rs : robot_scores) {
      if (static_cast<int>(rs.id) == last_attacker_id_) {
        current_score = rs.score;
        break;
      }
    }

    // 緊急切り替え: 新しいロボットが2倍以上良い場合は保持時間を無視して即座に切り替え
    if (best_score >= current_score * EMERGENCY_SWITCH_RATIO) {
      should_switch = true;
    } else if (time_since_switch >= MIN_HOLD_DURATION_SEC) {
      // 通常切り替え: 保持時間経過後、相対的に50%以上改善がある場合のみ切り替え
      double improvement_ratio = (best_score - current_score) / std::max(current_score, 0.1);
      if (improvement_ratio >= MIN_IMPROVEMENT_RATIO) {
        should_switch = true;
      }
    }
  }

  if (should_switch) {
    last_attacker_id_ = best_id;
    last_switch_time_ = current_time;
  }

  ctx.analysis.recommended_attacker_id = last_attacker_id_;
  ctx.analysis.attacker_suitability_score = best_score;

  // recommended_pass_receiver_idは未使用（-1固定）
  ctx.analysis.recommended_pass_receiver_id = -1;
}

}  // namespace crane::metrics
