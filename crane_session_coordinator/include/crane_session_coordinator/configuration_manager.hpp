// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_COORDINATOR__CONFIGURATION_MANAGER_HPP_
#define CRANE_SESSION_COORDINATOR__CONFIGURATION_MANAGER_HPP_

#include <crane_msgs/msg/practice_mode.hpp>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace crane
{
using SessionParameterType = std::variant<double, bool, int, std::string>;

struct SessionSlot
{
  std::string session_name;
  int max_robots;
  std::unordered_map<std::string, SessionParameterType> params;
  /// セッションに固定で割り当てるロボットID。空なら従来通りsuitabilityで動的割当。
  std::vector<uint8_t> fixed_robots;
  /// セッション側のロジック（例: 熱ローテーション）で使う候補IDプール。
  /// 割当アルゴリズムには影響を与えず、Sessionが自身のsuitability関数等で参照する。
  std::vector<uint8_t> candidate_robots;
};

/**
 * @brief YAML設定ファイルの読み込みと管理を担当するクラス
 *
 * 責務:
 * - 統合設定ファイル (unified_session_config.yaml) からセッション設定を読み込む
 * - イベント→セッションのマッピングを読み込む
 * - 設定の取得インターフェースを提供
 */
class ConfigurationManager
{
public:
  explicit ConfigurationManager(
    const std::filesystem::path & config_path,
    rclcpp::Logger logger = rclcpp::get_logger("ConfigurationManager"));

  /// <package_share_directory>/config/<config_file_name> を読み込む。
  ConfigurationManager(
    const std::string & package_share_directory,
    const std::string & config_file_name = "unified_session_config.yaml",
    rclcpp::Logger logger = rclcpp::get_logger("ConfigurationManager"));

  auto getSessionNameForEvent(const std::string & event_name) const -> std::optional<std::string>;

  auto getSessionCapacitiesForSituation(const std::string & situation_name) const
    -> std::optional<std::vector<SessionSlot>>;

  /// セッション注入用。
  auto updateEventMapping(const std::string & event_name, const std::string & situation_name)
    -> void;

  /// 練習モードが設定されていない situation ではデフォルト（disabled）を返す。
  auto getPracticeModeForSituation(const std::string & situation_name) const
    -> crane_msgs::msg::PracticeMode;

private:
  // イベント名 → セッション名のマッピング
  std::unordered_map<std::string, std::string> event_map_;

  // セッション名 → SessionSlotリストのマッピング
  std::unordered_map<std::string, std::vector<SessionSlot>> robot_selection_priority_map_;

  // セッション名 → 練習モード設定のマッピング
  std::unordered_map<std::string, crane_msgs::msg::PracticeMode> practice_mode_map_;

  rclcpp::Logger logger_;

  auto loadUnifiedConfig(const std::filesystem::path & config_file) -> void;
};

}  // namespace crane

#endif  // CRANE_SESSION_COORDINATOR__CONFIGURATION_MANAGER_HPP_
