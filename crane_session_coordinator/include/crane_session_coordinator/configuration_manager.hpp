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
  /**
   * @brief コンストラクタ
   * @param package_share_directory パッケージのshareディレクトリパス
   * @param config_file_name 統合設定ファイル名（デフォルト: unified_session_config.yaml）
   * @param logger ロガー
   */
  ConfigurationManager(
    const std::string & package_share_directory,
    const std::string & config_file_name = "unified_session_config.yaml",
    rclcpp::Logger logger = rclcpp::get_logger("ConfigurationManager"));

  /**
   * @brief イベント名に対応するセッション名を取得
   * @param event_name イベント名
   * @return セッション名（存在しない場合はstd::nullopt）
   */
  auto getSessionNameForEvent(const std::string & event_name) const -> std::optional<std::string>;

  /**
   * @brief セッション名に対応するSessionSlotリストを取得
   * @param situation_name セッション名
   * @return SessionSlotリスト（存在しない場合はstd::nullopt）
   */
  auto getSessionCapacitiesForSituation(const std::string & situation_name) const
    -> std::optional<std::vector<SessionSlot>>;

  /**
   * @brief イベントマップを取得（読み取り専用）
   * @return イベント名→セッション名のマップ
   */
  auto getEventMap() const -> const std::unordered_map<std::string, std::string> &;

  /**
   * @brief イベントマップのエントリを更新（セッション注入用）
   * @param event_name イベント名
   * @param situation_name セッション名
   */
  auto updateEventMapping(const std::string & event_name, const std::string & situation_name)
    -> void;

  /**
   * @brief 指定situationの練習モード設定を取得
   * @param situation_name セッション名
   * @return PracticeModeメッセージ（練習モード未設定の場合はデフォルト=disabled）
   */
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

  /**
   * @brief 統合設定ファイルを読み込む
   * @param config_file 設定ファイルパス
   */
  auto loadUnifiedConfig(const std::filesystem::path & config_file) -> void;
};

}  // namespace crane

#endif  // CRANE_SESSION_COORDINATOR__CONFIGURATION_MANAGER_HPP_
