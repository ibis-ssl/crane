// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_controller/global_robot_allocator.hpp"

#include <algorithm>
#include <crane_physics/position_assignments.hpp>
#include <crane_utils/stream.hpp>
#include <limits>
#include <sstream>

namespace crane
{

auto GlobalRobotAllocator::allocate(
  const std::vector<TacticRequirement> & requirements,
  const std::vector<uint8_t> & available_robots, WorldModelWrapper::SharedPtr & world_model,
  const AllocationState & prev_state, const AllocationCostConfig & config)
  -> std::unordered_map<std::string, std::vector<uint8_t>>
{
  std::unordered_map<std::string, std::vector<uint8_t>> result;

  RCLCPP_INFO(
    logger_, "[GlobalRobotAllocator] 割当開始: %lu Tactics, %lu ロボット", requirements.size(),
    available_robots.size());

  if (available_robots.empty()) {
    RCLCPP_WARN(logger_, "[GlobalRobotAllocator] 利用可能なロボットがありません");
    return result;
  }

  // ハード制約とソフト制約に分類
  std::vector<TacticRequirement> hard_requirements;
  std::vector<TacticRequirement> soft_requirements;

  for (const auto & req : requirements) {
    if (req.is_hard_constraint) {
      hard_requirements.push_back(req);
    } else {
      soft_requirements.push_back(req);
    }
  }

  // ハード制約を優先度順にソート
  std::ranges::sort(
    hard_requirements, [](const auto & a, const auto & b) { return a.priority < b.priority; });

  // ソフト制約も優先度順にソート
  std::ranges::sort(
    soft_requirements, [](const auto & a, const auto & b) { return a.priority < b.priority; });

  auto remaining_robots = available_robots;

  RCLCPP_INFO(
    logger_, "[GlobalRobotAllocator] ハード制約: %lu, ソフト制約: %lu", hard_requirements.size(),
    soft_requirements.size());

  // Phase 1: ハード制約Tacticを先に処理
  allocateHardConstraints(
    hard_requirements, remaining_robots, world_model, prev_state, config, result);

  RCLCPP_INFO(
    logger_, "[GlobalRobotAllocator] ハード制約処理後の残りロボット: %lu", remaining_robots.size());

  // Phase 2: 残りのロボットでソフト制約Tacticをハンガリアン法で処理
  allocateSoftConstraints(
    soft_requirements, remaining_robots, world_model, prev_state, config, result);

  RCLCPP_INFO(logger_, "[GlobalRobotAllocator] 割当完了: %lu Tactics", result.size());

  return result;
}

auto GlobalRobotAllocator::allocateHardConstraints(
  const std::vector<TacticRequirement> & hard_requirements, std::vector<uint8_t> & remaining_robots,
  WorldModelWrapper::SharedPtr & world_model, const AllocationState & prev_state,
  const AllocationCostConfig & config,
  std::unordered_map<std::string, std::vector<uint8_t>> & result) -> void
{
  for (const auto & req : hard_requirements) {
    if (remaining_robots.empty()) {
      RCLCPP_WARN(
        logger_, "ハード制約Tactic「%s」に割り当てるロボットが不足しています", req.name.c_str());
      break;
    }

    // 適性評価でロボットをソート
    std::vector<std::pair<uint8_t, double>> robot_scores;
    for (const auto & robot_id : remaining_robots) {
      auto robot = world_model->getOurRobot(robot_id);
      double score = req.suitability_func(robot);
      robot_scores.emplace_back(robot_id, score);
    }

    std::ranges::sort(robot_scores, [](const auto & a, const auto & b) {
      return a.second < b.second;  // コストが小さい順
    });

    // 必要数だけロボットを割り当て
    int num_to_allocate = std::min(req.max_robots, static_cast<int>(remaining_robots.size()));
    num_to_allocate = std::max(req.min_robots, num_to_allocate);

    std::vector<uint8_t> assigned_robots;
    for (int i = 0; i < num_to_allocate && i < robot_scores.size(); ++i) {
      assigned_robots.push_back(robot_scores[i].first);
    }

    result[req.name] = assigned_robots;

    // 割り当てたロボットをremaining_robotsから削除
    for (const auto & robot_id : assigned_robots) {
      auto it = std::find(remaining_robots.begin(), remaining_robots.end(), robot_id);
      if (it != remaining_robots.end()) {
        remaining_robots.erase(it);
      }
    }

    RCLCPP_DEBUG(
      logger_, "ハード制約Tactic「%s」に%luロボットを割り当て: %s", req.name.c_str(),
      assigned_robots.size(),
      [&]() {
        std::stringstream ss;
        for (const auto & id : assigned_robots) {
          ss << static_cast<int>(id) << " ";
        }
        return ss.str();
      }()
        .c_str());
  }
}

auto GlobalRobotAllocator::allocateSoftConstraints(
  const std::vector<TacticRequirement> & soft_requirements,
  const std::vector<uint8_t> & remaining_robots, WorldModelWrapper::SharedPtr & world_model,
  const AllocationState & prev_state, const AllocationCostConfig & config,
  std::unordered_map<std::string, std::vector<uint8_t>> & result) -> void
{
  if (remaining_robots.empty() || soft_requirements.empty()) {
    return;
  }

  // 仮想ターゲットを生成（各Tacticが必要とするロボット数分）
  std::vector<VirtualTarget> virtual_targets;
  for (const auto & req : soft_requirements) {
    int num_targets = std::min(req.max_robots, static_cast<int>(remaining_robots.size()));
    // 適性関数をshared_ptrでラップ（コピーコストを削減しライフタイムを管理）
    auto func_ptr = std::make_shared<std::function<double(const std::shared_ptr<RobotInfo> &)>>(
      req.suitability_func);
    for (int i = 0; i < num_targets; ++i) {
      virtual_targets.push_back({req.name, req.priority, static_cast<size_t>(i), func_ptr});
    }
  }

  if (virtual_targets.empty()) {
    return;
  }

  // ロボット数 ≤ ターゲット数 を保証
  if (remaining_robots.size() > virtual_targets.size()) {
    RCLCPP_WARN(
      logger_, "[allocateSoftConstraints] ロボット数(%lu)がターゲット数(%lu)を超えています",
      remaining_robots.size(), virtual_targets.size());
    // ハンガリアン法は robots <= targets を要求するため、
    // ダミーターゲットを追加する必要があるが、ここでは単純に警告を出す
  }

  // コスト関数を構築
  auto cost_func =
    buildCostMatrix(remaining_robots, virtual_targets, world_model, prev_state, config);

  // Hungarian実装は U >= V を要求するため、robots < targets の場合は転置する
  bool needs_transpose = remaining_robots.size() < virtual_targets.size();
  size_t matrix_rows = needs_transpose ? virtual_targets.size() : remaining_robots.size();
  size_t matrix_cols = needs_transpose ? remaining_robots.size() : virtual_targets.size();

  std::vector<std::vector<double>> cost_matrix(matrix_rows, std::vector<double>(matrix_cols));

  if (needs_transpose) {
    // 転置: cost_matrix[target][robot] = cost(robot, target)
    for (size_t i = 0; i < remaining_robots.size(); ++i) {
      for (size_t j = 0; j < virtual_targets.size(); ++j) {
        cost_matrix[j][i] = cost_func(i, j);
      }
    }
  } else {
    // 通常: cost_matrix[robot][target] = cost(robot, target)
    for (size_t i = 0; i < remaining_robots.size(); ++i) {
      for (size_t j = 0; j < virtual_targets.size(); ++j) {
        cost_matrix[i][j] = cost_func(i, j);
      }
    }
  }

  // コスト行列の検証と正規化
  double min_cost = std::numeric_limits<double>::max();
  double max_cost = std::numeric_limits<double>::lowest();
  bool has_invalid = false;

  for (size_t i = 0; i < matrix_rows; ++i) {
    for (size_t j = 0; j < matrix_cols; ++j) {
      if (std::isnan(cost_matrix[i][j]) || std::isinf(cost_matrix[i][j])) {
        RCLCPP_ERROR(
          logger_, "[allocateSoftConstraints] 不正なコスト値: cost[%lu][%lu]=%f", i, j,
          cost_matrix[i][j]);
        has_invalid = true;
      } else {
        min_cost = std::min(min_cost, cost_matrix[i][j]);
        max_cost = std::max(max_cost, cost_matrix[i][j]);
      }
    }
  }

  if (has_invalid) {
    RCLCPP_ERROR(logger_, "[allocateSoftConstraints] コスト行列に不正な値が含まれています");
    return;
  }

  // Hungarian法は非負のコスト行列を要求するため、負の値がある場合はオフセットを追加
  if (min_cost < 0.0) {
    RCLCPP_WARN(
      logger_, "[allocateSoftConstraints] 負のコスト値を検出: %f。オフセットを追加します",
      min_cost);
    double offset = -min_cost + 1.0;
    for (size_t i = 0; i < matrix_rows; ++i) {
      for (size_t j = 0; j < matrix_cols; ++j) {
        cost_matrix[i][j] += offset;
      }
    }
  }

  // ハンガリアン法で最適割当を計算
  std::vector<int> assignment;

  try {
    math::Hungarian<double> hungarian_solver(cost_matrix);
    auto [cost, solution_index] = hungarian_solver.solve();
    assignment = std::move(solution_index);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "[allocateSoftConstraints] ハンガリアン法で例外発生: %s", e.what());
    return;
  }

  // 割当結果を集計
  std::unordered_map<std::string, std::vector<uint8_t>> tactic_assignments;

  if (needs_transpose) {
    // 転置モード: assignment[target_idx] = robot_idx
    for (size_t target_idx = 0; target_idx < assignment.size(); ++target_idx) {
      int robot_idx = assignment[target_idx];
      if (robot_idx < 0 || robot_idx >= remaining_robots.size()) {
        continue;
      }

      const auto & target = virtual_targets[target_idx];
      uint8_t robot_id = remaining_robots[robot_idx];
      tactic_assignments[target.tactic_name].push_back(robot_id);
    }
  } else {
    // 通常モード: assignment[robot_idx] = target_idx
    for (size_t robot_idx = 0; robot_idx < assignment.size(); ++robot_idx) {
      int target_idx = assignment[robot_idx];
      if (target_idx < 0 || target_idx >= virtual_targets.size()) {
        continue;
      }

      const auto & target = virtual_targets[target_idx];
      uint8_t robot_id = remaining_robots[robot_idx];
      tactic_assignments[target.tactic_name].push_back(robot_id);
    }
  }

  // min_robotsの制約を満たしているか確認
  for (const auto & req : soft_requirements) {
    auto it = tactic_assignments.find(req.name);
    if (it != tactic_assignments.end()) {
      if (it->second.size() < req.min_robots) {
        RCLCPP_WARN(
          logger_, "Tactic「%s」の最小ロボット数(%d)を満たせませんでした（実際: %lu）",
          req.name.c_str(), req.min_robots, it->second.size());
      }
    } else if (req.min_robots > 0) {
      RCLCPP_WARN(
        logger_, "Tactic「%s」にロボットを割り当てられませんでした（最小要求: %d）",
        req.name.c_str(), req.min_robots);
    }
  }

  // 結果をマージ
  for (const auto & [tactic_name, robots] : tactic_assignments) {
    result[tactic_name] = robots;
    RCLCPP_DEBUG(
      logger_, "ソフト制約Tactic「%s」に%luロボットを割り当て: %s", tactic_name.c_str(),
      robots.size(),
      [&]() {
        std::stringstream ss;
        for (const auto & id : robots) {
          ss << static_cast<int>(id) << " ";
        }
        return ss.str();
      }()
        .c_str());
  }
}

auto GlobalRobotAllocator::buildCostMatrix(
  const std::vector<uint8_t> & robots, const std::vector<VirtualTarget> & targets,
  WorldModelWrapper::SharedPtr & world_model, const AllocationState & prev_state,
  const AllocationCostConfig & config) -> std::function<double(size_t, size_t)>
{
  // 必要な情報を値キャプチャしてライフタイムを保証
  // ただし、VirtualTargetは軽量なので直接コピー
  auto robots_copy = robots;
  auto targets_copy = targets;
  auto wm = world_model;
  auto state_copy = prev_state;
  auto config_copy = config;

  return [robots_copy, targets_copy, wm, state_copy, config_copy](
           size_t robot_idx, size_t target_idx) -> double {
    if (robot_idx >= robots_copy.size() || target_idx >= targets_copy.size()) {
      return std::numeric_limits<double>::max();
    }

    uint8_t robot_id = robots_copy[robot_idx];
    const auto & target = targets_copy[target_idx];
    auto robot = wm->getOurRobot(robot_id);

    // 基本コスト: shared_ptr経由で適性関数を呼び出す
    double base_cost = 0.0;
    if (target.suitability_func) {
      base_cost = (*target.suitability_func)(robot);
    }

    // ヒステリシスコンテキストを構築
    AssignmentContext context;
    context.was_assigned_to_same_tactic = state_copy.wasAssignedTo(robot_id, target.tactic_name);
    context.tactic_priority = target.tactic_priority;

    // 総コスト計算
    double priority_cost = context.tactic_priority * config_copy.priority_cost_multiplier;
    double hysteresis_bonus =
      context.was_assigned_to_same_tactic ? config_copy.hysteresis_bonus : 0.0;
    double velocity_hysteresis = 0.0;
    if (context.was_assigned_to_same_tactic) {
      velocity_hysteresis = robot->vel.linear.norm() * config_copy.velocity_hysteresis_factor;
    }

    return base_cost + priority_cost - hysteresis_bonus + velocity_hysteresis;
  };
}

}  // namespace crane
