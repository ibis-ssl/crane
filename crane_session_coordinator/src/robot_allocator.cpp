// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_coordinator/robot_allocator.hpp"

#include <algorithm>
#include <crane_utils/stream.hpp>
#include <range/v3/action/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <sstream>

namespace crane
{

RobotAllocator::RobotAllocator(
  std::shared_ptr<ConfigurationManager> config_manager,
  std::shared_ptr<SessionRegistry> tactic_registry, rclcpp::Logger logger)
: config_manager_(config_manager),
  session_registry_(tactic_registry),
  logger_(logger),
  global_allocator_(std::make_unique<GlobalRobotAllocator>(logger))
{
}

auto RobotAllocator::allocate(
  const std::string & session_name, std::vector<uint8_t> selectable_robot_ids,
  WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> crane_msgs::msg::RobotSelectResults
{
  auto session_capacities_opt = config_manager_->getSessionCapacitiesForSituation(session_name);
  if (!session_capacities_opt.has_value()) {
    RCLCPP_ERROR(
      logger_,
      "\t「%"
      "s」というSituationに対してロボット割当リクエストが発行されましたが，見つかりませんでした",
      session_name.c_str());
    return crane_msgs::msg::RobotSelectResults{};
  }

  const auto & session_capacities = session_capacities_opt.value();

  // 前回のプランナーリストを保存し、新しいリストをクリア
  auto prev_available_planners = session_registry_->getAllPlanners();
  session_registry_->clear();

  // SessionRequirementリストを構築
  std::vector<SessionRequirement> requirements;
  int priority = 0;
  for (const auto & session_capacity : session_capacities) {
    if (session_capacity.max_robots <= 0) {
      continue;
    }

    // プランナー生成
    auto session = session_registry_->getOrCreatePlanner(
      session_capacity.session_name, world_model, node, prev_available_planners,
      session_capacity.params);

    // 適性関数を取得
    auto suitability_func = session->getRobotSuitabilityFunc();

    // 動的ロボット数を取得してクランプ
    int desired =
      session->getDesiredRobotNumber(session_capacity.min_robots, session_capacity.max_robots);
    int effective_max =
      std::clamp(desired, session_capacity.min_robots, session_capacity.max_robots);

    requirements.emplace_back(
      session_capacity.session_name, priority++,
      session_capacity.min_robots,  // min_robots
      effective_max,                // max_robots（動的に調整）
      suitability_func);
  }

  // GlobalRobotAllocatorで割当を実行
  auto allocation = global_allocator_->allocate(
    requirements, selectable_robot_ids, world_model, allocation_state_, allocation_cost_config_);

  // 割当結果を適用
  crane_msgs::msg::RobotSelectResults results;
  for (const auto & [allocated_name, robot_ids] : allocation) {
    // session_capacitiesを1回だけ検索（フォールバック生成とmin/max取得の両方で使い回す）
    auto capacity_it = std::find_if(
      session_capacities.begin(), session_capacities.end(),
      [&allocated_name](const auto & s) { return s.session_name == allocated_name; });

    // Sessionを取得または再生成
    auto session_it = std::find_if(
      session_registry_->getAllPlanners().begin(), session_registry_->getAllPlanners().end(),
      [&allocated_name](const auto & t) { return t->name == allocated_name; });

    SessionBase::SharedPtr session;
    if (session_it != session_registry_->getAllPlanners().end()) {
      session = *session_it;
    } else if (capacity_it != session_capacities.end()) {
      // 見つからない場合は新規生成（通常はここには来ない）
      session = session_registry_->getOrCreatePlanner(
        allocated_name, world_model, node, prev_available_planners, capacity_it->params);
    }

    if (session) {
      // ロボット割当をSessionに反映
      // GlobalRobotAllocatorが選択したロボットを直接設定（getSelectedRobotsをバイパス）
      session->setAllocatedRobots(robot_ids);

      // レジストリに追加
      if (session_it == session_registry_->getAllPlanners().end()) {
        session_registry_->addPlanner(session);
      }

      // AllocationStateを更新（ターゲット位置は現時点では不明なのでロボット位置を使用）
      for (auto id : robot_ids) {
        auto robot = world_model->getOurRobot(id);
        allocation_state_.updateAssignment(id, allocated_name, robot->pose.pos);
        prev_robot_roles_.insert_or_assign(id, RobotRole{allocated_name, ""});
      }

      // RobotSelectResult を構築
      crane_msgs::msg::RobotSelectResult result;
      result.name = allocated_name;

      if (capacity_it != session_capacities.end()) {
        result.min_robots_num = static_cast<uint8_t>(capacity_it->min_robots);
        result.max_robots_num = static_cast<uint8_t>(capacity_it->max_robots);
      }

      result.selectable_robots_num = static_cast<uint8_t>(selectable_robot_ids.size());
      result.selectable_robots = selectable_robot_ids;
      result.selected_robots = robot_ids;
      results.results.push_back(result);
    }
  }

  return results;
}

auto RobotAllocator::detectRobotChange(const std::vector<uint8_t> & observed_robot_ids) const
  -> bool
{
  auto assigned_robot_ids = getAssignedRobotIds();
  auto sorted_observed_ids = observed_robot_ids;
  std::sort(sorted_observed_ids.begin(), sorted_observed_ids.end());

  if (assigned_robot_ids.size() != sorted_observed_ids.size()) {
    RCLCPP_DEBUG_STREAM(
      logger_, "ロボットの数が変動しています｜割当数：" << assigned_robot_ids.size() << ", 観測数："
                                                        << sorted_observed_ids.size());
    return true;
  } else if (assigned_robot_ids != sorted_observed_ids) {
    RCLCPP_DEBUG_STREAM(
      logger_, "ロボットの数は変わっていないですが、ラインナップが変動しています\n"
                 << "\tbefore: " << assigned_robot_ids << "\tafter : " << sorted_observed_ids);
    return true;
  }

  return false;
}

auto RobotAllocator::getAssignedRobotIds() const -> std::vector<uint8_t>
{
  auto assigned_robot_ids =
    session_registry_->getAllPlanners() |
    ranges::views::transform([](const auto & session) { return session->getRobots(); }) |
    ranges::views::join | ranges::views::transform([](const auto & robot) { return robot.id; }) |
    ranges::to<std::vector>() | ranges::actions::sort;
  return assigned_robot_ids;
}

auto RobotAllocator::buildAssignmentLog() const -> std::string
{
  std::stringstream assignment_log;
  bool first = true;
  for (const auto & session : session_registry_->getAllPlanners()) {
    if (!first) {
      assignment_log << ", ";
    }
    first = false;
    assignment_log << session->name << ":[";
    const auto & robots = session->getRobots();
    for (size_t i = 0; i < robots.size(); ++i) {
      if (i > 0) {
        assignment_log << ",";
      }
      assignment_log << static_cast<int>(robots[i].id);
    }
    assignment_log << "]";
  }
  return assignment_log.str();
}

auto RobotAllocator::logAssignmentIfChanged(const std::string & current_assignment) -> void
{
  if (current_assignment != prev_assignment_log_) {
    if (current_assignment.empty()) {
      RCLCPP_DEBUG(logger_, "ロボット割当: なし");
    } else {
      RCLCPP_DEBUG(logger_, "ロボット割当: %s", current_assignment.c_str());
    }
    prev_assignment_log_ = current_assignment;
  }
}

}  // namespace crane
