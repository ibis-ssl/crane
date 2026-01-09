// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_controller/robot_allocator.hpp"

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
  std::shared_ptr<PlannerRegistry> planner_registry, rclcpp::Logger logger)
: config_manager_(config_manager), planner_registry_(planner_registry), logger_(logger)
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
  auto prev_available_planners = planner_registry_->getAllPlanners();
  planner_registry_->clear();
  prev_robot_roles_.clear();

  crane_msgs::msg::RobotSelectResults results;

  // 優先順位が高いPlannerから順にロボットを割り当てる
  for (const auto & session_capacity : session_capacities) {
    if (!tryAssignRobotToPlanner(
          session_capacity, selectable_robot_ids, prev_available_planners, world_model, node,
          results)) {
      // エラーが発生した場合はループを抜ける
      break;
    }
  }

  // 割り当てられなかったロボットを待機状態にする
  if (not selectable_robot_ids.empty()) {
    SessionCapacity waiter_session{"waiter", static_cast<int>(selectable_robot_ids.size()), {}};
    tryAssignRobotToPlanner(
      waiter_session, selectable_robot_ids, prev_available_planners, world_model, node, results);
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
    planner_registry_->getAllPlanners() |
    ranges::views::transform([](const auto & planner) { return planner->getRobots(); }) |
    ranges::views::join | ranges::views::transform([](const auto & robot) { return robot.id; }) |
    ranges::to<std::vector>() | ranges::actions::sort;
  return assigned_robot_ids;
}

auto RobotAllocator::buildAssignmentLog() const -> std::string
{
  std::stringstream assignment_log;
  bool first = true;
  for (const auto & planner : planner_registry_->getAllPlanners()) {
    if (!first) {
      assignment_log << ", ";
    }
    first = false;
    assignment_log << planner->name << ":[";
    const auto & robots = planner->getRobots();
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

auto RobotAllocator::tryAssignRobotToPlanner(
  const SessionCapacity & session_capacity, std::vector<uint8_t> & selectable_robot_ids,
  const std::vector<PlannerBase::SharedPtr> & prev_available_planners,
  WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node,
  crane_msgs::msg::RobotSelectResults & results) -> bool
{
  // 割り当て可能なロボットがない場合はスキップ
  if (session_capacity.selectable_robot_num <= 0 || selectable_robot_ids.empty()) {
    return true;
  }

  crane_msgs::msg::RobotSelectResult result;
  result.name = session_capacity.session_name;
  result.selectable_robots_num = session_capacity.selectable_robot_num;
  std::ranges::copy(selectable_robot_ids, std::back_inserter(result.selectable_robots));

  try {
    // プランナー生成とロボット選択
    // PlannerRegistryを使ってプランナーを取得または生成
    auto planner = planner_registry_->getOrCreatePlanner(
      session_capacity.session_name, world_model, node, prev_available_planners,
      session_capacity.params);

    auto selected_robots = planner->selectRobots(
      selectable_robot_ids, session_capacity.selectable_robot_num, prev_robot_roles_);
    results.results.push_back(result);

    // プランナーをレジストリに登録
    planner_registry_->addPlanner(planner);

    if (not selectable_robot_ids.empty()) {
      RCLCPP_DEBUG_STREAM(
        logger_, "\tセッション「" << session_capacity.session_name << "」のロボット選択："
                                  << selectable_robot_ids << " -> " << selected_robots);
    }

    // 割当依頼結果の反映
    for (auto selected_robot_id : selected_robots) {
      // 割当されたロボットを利用可能ロボットリストから削除
      selectable_robot_ids.erase(
        remove(selectable_robot_ids.begin(), selectable_robot_ids.end(), selected_robot_id),
        selectable_robot_ids.end());
      // 割当されたロボットをロールマップに追加(この情報は他のプランナにも共有される)
      prev_robot_roles_.insert_or_assign(
        selected_robot_id, RobotRole{session_capacity.session_name, ""});
    }

    return true;
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      logger_, "\t「%s」というプランナを呼び出した時に例外が発生しました : %s",
      session_capacity.session_name.c_str(), e.what());
    return false;
  }
}

}  // namespace crane
