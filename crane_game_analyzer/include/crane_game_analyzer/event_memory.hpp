// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__EVENT_MEMORY_HPP_
#define CRANE_GAME_ANALYZER__EVENT_MEMORY_HPP_

#include <crane_msgs/msg/ronar_event.hpp>
#include <deque>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace crane
{
/// イベントの重要度スコア計算用設定
struct EventImportanceConfig
{
  // イベントタイプごとの基本重要度
  double goal_importance = 1.0;
  double fast_shot_importance = 0.8;
  double shot_importance = 0.6;
  double possession_change_importance = 0.3;
  double ball_out_importance = 0.4;
  double set_play_importance = 0.5;

  // Play Switcher イベントの重要度
  double halt_importance = 0.3;
  double stop_importance = 0.3;
  double inplay_start_importance = 0.6;
  double timeout_importance = 0.7;
  double half_time_importance = 0.8;
  double game_end_importance = 1.0;

  // Autoref イベントの重要度
  double foul_importance = 0.7;

  // 時間減衰係数（秒あたり）
  double time_decay_rate = 0.1;
};

/// イベントとメタデータを保持する構造体
struct EventRecord
{
  crane_msgs::msg::RonarEvent event;
  rclcpp::Time received_time;
  double importance_score;

  /// 時間経過による重要度減衰を適用
  [[nodiscard]] auto getDecayedImportance(
    const rclcpp::Time & current_time, double decay_rate) const -> double
  {
    double elapsed = (current_time - received_time).seconds();
    return importance_score * std::exp(-decay_rate * elapsed);
  }
};

/// WorkingMemory: 短期記憶（30秒のリングバッファ）
/// 最近のイベントを保持し、文脈理解に使用
class WorkingMemory
{
public:
  explicit WorkingMemory(double retention_duration_sec = 30.0)
  : retention_duration_(rclcpp::Duration::from_seconds(retention_duration_sec))
  {
  }

  /// イベントを追加
  auto addEvent(const EventRecord & record) -> void { events_.push_front(record); }

  /// 古いイベントを削除
  auto cleanup(const rclcpp::Time & current_time) -> void
  {
    std::erase_if(events_, [&](const EventRecord & record) {
      return (current_time - record.received_time) > retention_duration_;
    });
  }

  /// 最近のイベントを取得（最新順）
  [[nodiscard]] auto getRecentEvents(size_t max_count = 10) const -> std::vector<EventRecord>
  {
    std::vector<EventRecord> result;
    size_t count = std::min(max_count, events_.size());
    for (size_t i = 0; i < count; ++i) {
      result.push_back(events_[i]);
    }
    return result;
  }

  /// 特定タイプのイベントを取得
  [[nodiscard]] auto getEventsByType(uint8_t event_type) const -> std::vector<EventRecord>
  {
    std::vector<EventRecord> result;
    for (const auto & record : events_) {
      if (record.event.event_type == event_type) {
        result.push_back(record);
      }
    }
    return result;
  }

  /// 現在のイベント数
  [[nodiscard]] auto size() const -> size_t { return events_.size(); }

  /// イベントをクリア
  auto clear() -> void { events_.clear(); }

private:
  std::deque<EventRecord> events_;
  rclcpp::Duration retention_duration_;
};

/// HighlightBuffer: 重要イベントのバッファ
/// 試合のハイライト生成や分析用に重要なイベントを保持
class HighlightBuffer
{
public:
  explicit HighlightBuffer(size_t max_size = 100) : max_size_(max_size) {}

  /// 重要度が閾値以上のイベントを追加
  auto addIfImportant(const EventRecord & record, double threshold = 0.5) -> bool
  {
    if (record.importance_score >= threshold) {
      highlights_.push_front(record);
      if (highlights_.size() > max_size_) {
        highlights_.pop_back();
      }
      return true;
    }
    return false;
  }

  /// ハイライトを取得（重要度順）
  [[nodiscard]] auto getHighlights(size_t max_count = 10) const -> std::vector<EventRecord>
  {
    std::vector<EventRecord> result;
    size_t count = std::min(max_count, highlights_.size());
    for (size_t i = 0; i < count; ++i) {
      result.push_back(highlights_[i]);
    }
    // 重要度でソート
    std::sort(result.begin(), result.end(), [](const EventRecord & a, const EventRecord & b) {
      return a.importance_score > b.importance_score;
    });
    return result;
  }

  /// 最新のハイライトを取得
  [[nodiscard]] auto getLatestHighlight() const -> std::optional<EventRecord>
  {
    if (highlights_.empty()) {
      return std::nullopt;
    }
    return highlights_.front();
  }

  /// 現在のハイライト数
  [[nodiscard]] auto size() const -> size_t { return highlights_.size(); }

  /// ハイライトをクリア
  auto clear() -> void { highlights_.clear(); }

private:
  std::deque<EventRecord> highlights_;
  size_t max_size_;
};

/// EventMemory: イベントメモリシステム
/// WorkingMemoryとHighlightBufferを統合管理
class EventMemory
{
public:
  explicit EventMemory(rclcpp::Clock::SharedPtr clock) : clock_(clock) {}

  /// 新しいイベントを処理
  auto processEvent(const crane_msgs::msg::RonarEvent & event) -> void
  {
    EventRecord record;
    record.event = event;
    record.received_time = clock_->now();
    record.importance_score = calculateImportance(event);

    // WorkingMemoryに追加
    working_memory_.addEvent(record);

    // 重要度が高ければHighlightBufferにも追加
    highlight_buffer_.addIfImportant(record, config_.goal_importance * 0.5);

    // 古いイベントをクリーンアップ
    working_memory_.cleanup(clock_->now());
  }

  /// 重要度を計算
  [[nodiscard]] auto calculateImportance(const crane_msgs::msg::RonarEvent & event) const -> double
  {
    double base_importance = 0.0;

    switch (event.event_type) {
      case crane_msgs::msg::RonarEvent::EVENT_GOAL:
        base_importance = config_.goal_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_FAST_SHOT:
        base_importance = config_.fast_shot_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_SHOT:
        base_importance = config_.shot_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_POSSESSION_CHANGE:
        base_importance = config_.possession_change_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_BALL_OUT:
        base_importance = config_.ball_out_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_SET_PLAY:
        base_importance = config_.set_play_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_HALT:
        base_importance = config_.halt_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_STOP:
        base_importance = config_.stop_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_INPLAY_START:
        base_importance = config_.inplay_start_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_TIMEOUT:
        base_importance = config_.timeout_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_HALF_TIME:
        base_importance = config_.half_time_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_GAME_END:
        base_importance = config_.game_end_importance;
        break;
      case crane_msgs::msg::RonarEvent::EVENT_FOUL:
        base_importance = config_.foul_importance;
        break;
      default:
        base_importance = 0.2;
        break;
    }

    // ボール速度による補正（シュート系イベント）
    if (
      event.event_type == crane_msgs::msg::RonarEvent::EVENT_SHOT ||
      event.event_type == crane_msgs::msg::RonarEvent::EVENT_FAST_SHOT) {
      // 速度が速いほど重要度を上げる（最大1.5倍）
      double speed_factor = 1.0 + std::min(event.ball_speed / 10.0, 0.5);
      base_importance *= speed_factor;
    }

    // 信頼度による補正
    base_importance *= event.confidence;

    return std::clamp(base_importance, 0.0, 1.0);
  }

  /// 最近のイベントを取得
  [[nodiscard]] auto getRecentEvents(size_t max_count = 10) const -> std::vector<EventRecord>
  {
    return working_memory_.getRecentEvents(max_count);
  }

  /// ハイライトを取得
  [[nodiscard]] auto getHighlights(size_t max_count = 10) const -> std::vector<EventRecord>
  {
    return highlight_buffer_.getHighlights(max_count);
  }

  /// 特定タイプのイベントを取得
  [[nodiscard]] auto getEventsByType(uint8_t event_type) const -> std::vector<EventRecord>
  {
    return working_memory_.getEventsByType(event_type);
  }

  /// 設定を更新
  auto setConfig(const EventImportanceConfig & config) -> void { config_ = config; }

  /// WorkingMemoryへのアクセス
  [[nodiscard]] auto workingMemory() const -> const WorkingMemory & { return working_memory_; }

  /// HighlightBufferへのアクセス
  [[nodiscard]] auto highlightBuffer() const -> const HighlightBuffer & { return highlight_buffer_; }

  /// メモリをクリア
  auto clear() -> void
  {
    working_memory_.clear();
    highlight_buffer_.clear();
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  EventImportanceConfig config_;
  WorkingMemory working_memory_;
  HighlightBuffer highlight_buffer_;
};

}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__EVENT_MEMORY_HPP_
