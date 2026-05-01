// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_coordinator/configuration_manager.hpp"

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

auto ConfigurationManager::getSessionCapacitiesForSituation(
  const std::string & situation_name) const -> std::optional<std::vector<SessionSlot>>
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

auto ConfigurationManager::getPracticeModeForSituation(const std::string & situation_name) const
  -> crane_msgs::msg::PracticeMode
{
  auto it = practice_mode_map_.find(situation_name);
  if (it != practice_mode_map_.end()) {
    return it->second;
  }
  // デフォルト: 練習モード無効
  return crane_msgs::msg::PracticeMode{};
}

auto ConfigurationManager::loadUnifiedConfig(const std::filesystem::path & config_file) -> void
{
  auto config = YAML::LoadFile(config_file.c_str());

  // Situationsの読み込み
  if (config["situations"]) {
    for (const auto & situation_entry : config["situations"]) {
      const std::string situation_name = situation_entry.first.as<std::string>();
      const auto & situation_data = situation_entry.second;

      std::vector<SessionSlot> session_capacity_list;
      std::stringstream ss;
      ss << "SITUATION : " << situation_name << "\n";
      ss << "DESCRIPTION : " << situation_data["description"] << "\n";
      ss << "SESSIONS : " << "\n";

      for (const auto & session_node : situation_data["sessions"]) {
        SessionSlot session_capacity;
        session_capacity.session_name = session_node["name"].as<std::string>();

        // max_robotsの読み込みとデフォルト値設定
        if (session_node["max_robots"]) {
          session_capacity.max_robots = session_node["max_robots"].as<int>();
        } else {
          session_capacity.max_robots = 1;  // デフォルト: 1
        }

        // バリデーション
        if (session_capacity.max_robots <= 0) {
          RCLCPP_WARN(
            logger_, "Invalid max_robots (%d) for session '%s': must be > 0. Using 1.",
            session_capacity.max_robots, session_capacity.session_name.c_str());
          session_capacity.max_robots = 1;
        }

        ss << "\tNAME       : " << session_capacity.session_name << "\n";
        ss << "\tMAX_ROBOTS : " << session_capacity.max_robots << "\n";

        // fixed_robots の読み込み（存在する場合のみ）
        // 指定されたIDは suitability 評価をスキップして優先割当される
        if (session_node["fixed_robots"]) {
          for (const auto & id_node : session_node["fixed_robots"]) {
            session_capacity.fixed_robots.push_back(static_cast<uint8_t>(id_node.as<int>()));
          }
          ss << "\tFIXED_ROBOTS : " << session_capacity.fixed_robots.size() << " entries\n";
        }

        // candidate_robots の読み込み（存在する場合のみ）
        // allocator は参照しない。Session 側のロジック（例: 熱ローテ）が使う。
        if (session_node["candidate_robots"]) {
          for (const auto & id_node : session_node["candidate_robots"]) {
            session_capacity.candidate_robots.push_back(static_cast<uint8_t>(id_node.as<int>()));
          }
          ss << "\tCANDIDATE_ROBOTS : " << session_capacity.candidate_robots.size() << " entries\n";
        }

        // params の読み込み（存在する場合のみ）
        if (session_node["params"]) {
          for (const auto & param : session_node["params"]) {
            std::string key = param.first.as<std::string>();
            const auto & value = param.second;
            if (value.IsScalar()) {
              std::string str_val = value.as<std::string>();
              // bool判定
              if (str_val == "true") {
                session_capacity.params[key] = true;
              } else if (str_val == "false") {
                session_capacity.params[key] = false;
              } else {
                // 数値判定
                try {
                  size_t pos;
                  int int_val = std::stoi(str_val, &pos);
                  if (pos == str_val.size()) {
                    session_capacity.params[key] = int_val;
                  } else {
                    session_capacity.params[key] = std::stod(str_val);
                  }
                } catch (...) {
                  session_capacity.params[key] = str_val;
                }
              }
            }
          }
          ss << "\tPARAMS   : " << session_capacity.params.size() << " entries\n";
        }

        session_capacity_list.emplace_back(session_capacity);
      }
      robot_selection_priority_map_[situation_name] = session_capacity_list;

      // practice_mode の読み込み（存在する場合のみ）
      if (situation_data["practice_mode"]) {
        crane_msgs::msg::PracticeMode pm;
        const auto & pm_node = situation_data["practice_mode"];
        pm.enabled = pm_node["enabled"] ? pm_node["enabled"].as<bool>() : true;
        pm.positive_side = pm_node["positive_side"] ? pm_node["positive_side"].as<bool>() : true;
        pm.kick_prohibition =
          pm_node["kick_prohibition"] ? pm_node["kick_prohibition"].as<bool>() : false;
        pm.reverse_attack =
          pm_node["reverse_attack"] ? pm_node["reverse_attack"].as<bool>() : false;
        pm.use_normal_speed =
          pm_node["use_normal_speed"] ? pm_node["use_normal_speed"].as<bool>() : true;
        if (pm_node["kick_allowed_skills"]) {
          for (const auto & skill_node : pm_node["kick_allowed_skills"]) {
            pm.kick_allowed_skills.push_back(skill_node.as<std::string>());
          }
        }
        practice_mode_map_[situation_name] = pm;
        ss << "PRACTICE_MODE : enabled=" << (pm.enabled ? "true" : "false")
           << " positive_side=" << (pm.positive_side ? "true" : "false")
           << " kick_prohibition=" << (pm.kick_prohibition ? "true" : "false")
           << " reverse_attack=" << (pm.reverse_attack ? "true" : "false")
           << " use_normal_speed=" << (pm.use_normal_speed ? "true" : "false");
        if (!pm.kick_allowed_skills.empty()) {
          ss << " kick_allowed_skills=[";
          for (size_t i = 0; i < pm.kick_allowed_skills.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << pm.kick_allowed_skills[i];
          }
          ss << "]";
        }
        ss << "\n";
      }

      ss << "----------------------------------------";
      RCLCPP_DEBUG_STREAM(logger_, ss.str());
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
