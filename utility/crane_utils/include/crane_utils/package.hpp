// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_UTILS__PACKAGE_HPP_
#define CRANE_UTILS__PACKAGE_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <optional>
#include <rclcpp/logging.hpp>
#include <string>

namespace crane
{

/**
 * @brief パッケージの share ディレクトリパスを取得する（取得失敗時は nullopt）
 *
 * @param package_name ROS 2 パッケージ名
 * @return std::optional<std::filesystem::path> パッケージの share ディレクトリパス
 */
inline auto get_package_share_path(const std::string & package_name)
  -> std::optional<std::filesystem::path>
{
  try {
    return std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

/**
 * @brief 設定ファイルやアセット等のパッケージ相対パスを安全に解決する
 *
 * パスが空または絶対パスの場合はそのまま返却します。
 * 相対パスの場合は <package_share_directory>/<sub_dir>/<path> を返却します。
 * パッケージが見つからない場合は path をそのまま返却します。
 *
 * @param package_name ROS 2 パッケージ名
 * @param path ファイルパス
 * @param sub_dir パッケージ share ディレクトリ内のサブディレクトリ（デフォルト: "config"）
 * @return std::filesystem::path 解決されたパス
 */
inline auto resolve_package_path(
  const std::string & package_name, const std::filesystem::path & path,
  const std::filesystem::path & sub_dir = "config") -> std::filesystem::path
{
  if (path.empty() || path.is_absolute()) {
    return path;
  }
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory(package_name);
    return std::filesystem::path(share_dir) / sub_dir / path;
  } catch (const std::exception &) {
    return path;
  }
}

/**
 * @brief ロガー付きオーバーロード（失敗時に WARN ログを出力）
 *
 * @param logger ROS 2 ロガー
 * @param package_name ROS 2 パッケージ名
 * @param path ファイルパス
 * @param sub_dir パッケージ share ディレクトリ内のサブディレクトリ（デフォルト: "config"）
 * @return std::filesystem::path 解決されたパス
 */
inline auto resolve_package_path(
  const rclcpp::Logger & logger, const std::string & package_name,
  const std::filesystem::path & path, const std::filesystem::path & sub_dir = "config")
  -> std::filesystem::path
{
  if (path.empty() || path.is_absolute()) {
    return path;
  }
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory(package_name);
    return std::filesystem::path(share_dir) / sub_dir / path;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      logger, "パッケージ '%s' のディレクトリ取得に失敗しました: %s (相対パスとして扱います)",
      package_name.c_str(), ex.what());
    return path;
  }
}

}  // namespace crane

#endif  // CRANE_UTILS__PACKAGE_HPP_
