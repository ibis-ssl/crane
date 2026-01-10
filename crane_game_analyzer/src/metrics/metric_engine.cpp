// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/metric_engine.hpp"

#include <queue>
#include <sstream>
#include <unordered_map>

namespace crane::metrics
{

auto MetricEngine::registerMetric(MetricBase::Ptr metric) -> void
{
  metrics_[metric->getId()] = metric;
}

auto MetricEngine::initialize() -> bool { return buildExecutionOrder(); }

auto MetricEngine::computeAll(MetricContext & ctx) -> void
{
  for (const auto & id : execution_order_) {
    auto it = metrics_.find(id);
    if (it != metrics_.end()) {
      it->second->compute(ctx);
    }
  }
}

auto MetricEngine::visualizeAll(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
{
  for (const auto & id : execution_order_) {
    auto it = metrics_.find(id);
    if (it != metrics_.end()) {
      it->second->visualize(ctx, visualizer);
    }
  }
}

auto MetricEngine::buildExecutionOrder() -> bool
{
  // 入次数と隣接リストを構築
  std::unordered_map<MetricId, int> in_degree;
  std::unordered_map<MetricId, std::vector<MetricId>> adj;

  // 全メトリクスの入次数を0で初期化
  for (const auto & [id, metric] : metrics_) {
    in_degree[id] = 0;
  }

  // グラフ構築: 依存関係を辺として表現
  // A depends on B => edge B -> A
  for (const auto & [id, metric] : metrics_) {
    for (const auto & dep : metric->getDependencies()) {
      // 依存先が登録されているか確認
      if (metrics_.find(dep) == metrics_.end()) {
        RCLCPP_ERROR(
          logger_, "Metric '%s' depends on unregistered metric (ID: %d)", metric->getName().c_str(),
          static_cast<int>(dep));
        return false;
      }
      // 依存関係を追加: dep -> id
      adj[dep].push_back(id);
      in_degree[id]++;
    }
  }

  // Kahnのアルゴリズム: トポロジカルソート
  std::queue<MetricId> queue;

  // 入次数0のノードをキューに追加
  for (const auto & [id, degree] : in_degree) {
    if (degree == 0) {
      queue.push(id);
    }
  }

  execution_order_.clear();

  while (!queue.empty()) {
    auto current = queue.front();
    queue.pop();
    execution_order_.push_back(current);

    // currentから出ている辺を削除
    for (const auto & next : adj[current]) {
      if (--in_degree[next] == 0) {
        queue.push(next);
      }
    }
  }

  // 循環依存検出: 全ノードを処理できなかった場合
  if (execution_order_.size() != metrics_.size()) {
    RCLCPP_ERROR(logger_, "Circular dependency detected in metrics!");

    // 処理できなかったメトリクスをログ出力
    std::stringstream ss;
    ss << "Unresolved metrics: ";
    for (const auto & [id, metric] : metrics_) {
      if (
        std::find(execution_order_.begin(), execution_order_.end(), id) == execution_order_.end()) {
        ss << metric->getName() << " ";
      }
    }
    RCLCPP_ERROR(logger_, "%s", ss.str().c_str());

    return false;
  }

  // 実行順序をログ出力
  std::stringstream ss;
  ss << "Metric execution order: ";
  for (size_t i = 0; i < execution_order_.size(); ++i) {
    if (i > 0) ss << " -> ";
    ss << metrics_[execution_order_[i]]->getName();
  }
  RCLCPP_INFO(logger_, "%s", ss.str().c_str());

  return true;
}

}  // namespace crane::metrics
