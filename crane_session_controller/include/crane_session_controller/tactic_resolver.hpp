// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__TACTIC_RESOLVER_HPP_
#define CRANE_SESSION_CONTROLLER__TACTIC_RESOLVER_HPP_

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "configuration_manager.hpp"

namespace crane
{

/**
 * @brief イベント名からセッション名への解決、セッション変更の検出を行うクラス
 */
class TacticResolver
{
public:
  explicit TacticResolver(
    std::shared_ptr<ConfigurationManager> config_manager, rclcpp::Logger logger);

  /**
   * @brief イベント名を解決してセッション名を返す
   * @param event_name イベント名
   * @return セッション名（解決できない場合はnullopt）
   */
  auto resolve(const std::string & event_name) -> std::optional<std::string>;

  /**
   * @brief 前回からセッションが変更されたかチェック
   * @return 変更があればtrue
   */
  auto hasSessionChanged() const -> bool;

  /**
   * @brief 現在のセッション名を取得
   * @return 現在のセッション名
   */
  auto getCurrentSessionName() const -> std::string;

private:
  std::shared_ptr<ConfigurationManager> config_manager_;
  rclcpp::Logger logger_;

  std::string current_session_name_;
  std::string previous_session_name_;
};

}  // namespace crane

#endif  // CRANE_SESSION_CONTROLLER__TACTIC_RESOLVER_HPP_
