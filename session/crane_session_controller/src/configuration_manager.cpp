// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_controller/configuration_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace crane
{
ConfigurationManager::ConfigurationManager(
  const std::string & package_share_directory, const std::string & config_file_name,
  rclcpp::Logger logger)
: logger_(logger)
{
  using std::filesystem::path;

  // 統合設定ファイルの読み込み
  auto config_path = path(package_share_directory) / "config" / config_file_name;
  loadUnifiedConfig(config_path);
}

auto ConfigurationManager::getSessionNameForEvent(const std::string & event_name) const
  -> std::optional<std::string>
{
  auto it = event_map_.find(event_name);
  if (it != event_map_.end()) {
    return it->second;
  }
  return std::nullopt;
}

auto ConfigurationManager::getSessionCapacitiesForSituation(const std::string & situation_name) const
  -> std::optional<std::vector<SessionCapacity>>
{
  auto it = robot_selection_priority_map_.find(situation_name);
  if (it != robot_selection_priority_map_.end()) {
    return it->second;
  }
  return std::nullopt;
}

auto ConfigurationManager::getEventMap() const
  -> const std::unordered_map<std::string, std::string> &
{
  return event_map_;
}

auto ConfigurationManager::updateEventMapping(
  const std::string & event_name, const std::string & situation_name) -> void
{
  event_map_[event_name] = situation_name;
}

auto ConfigurationManager::loadUnifiedConfig(const std::filesystem::path & config_file) -> void
{
  auto config = YAML::LoadFile(config_file.c_str());

  // Situationsの読み込み
  if (config["situations"]) {
    for (const auto & situation_entry : config["situations"]) {
      const std::string situation_name = situation_entry.first.as<std::string>();
      const auto & situation_data = situation_entry.second;

      std::stringstream ss;
      ss << "SITUATION : " << situation_name << std::endl;
      ss << "DESCRIPTION : " << situation_data["description"] << std::endl;
      ss << "SESSIONS : " << std::endl;

      std::vector<SessionCapacity> session_capacity_list;
      for (const auto & session_node : situation_data["sessions"]) {
        ss << "\tNAME     : " << session_node["name"] << std::endl;
        ss << "\tCAPACITY : " << session_node["capacity"] << std::endl;
        session_capacity_list.emplace_back(SessionCapacity(
          {session_node["name"].as<std::string>(), session_node["capacity"].as<int>()}));
      }
      robot_selection_priority_map_[situation_name] = session_capacity_list;

      ss << "----------------------------------------" << std::endl;
      RCLCPP_DEBUG(logger_, "%s", ss.str().c_str());
    }
  }

  // Eventsの読み込み
  if (config["events"]) {
    for (const auto & event_node : config["events"]) {
      event_map_[event_node["name"].as<std::string>()] = event_node["situation"].as<std::string>();
    }
  }

  RCLCPP_INFO(
    logger_, "設定ファイル読み込み完了: Situations=%zu個, Events=%zu個",
    robot_selection_priority_map_.size(), event_map_.size());
}

}  // namespace crane
