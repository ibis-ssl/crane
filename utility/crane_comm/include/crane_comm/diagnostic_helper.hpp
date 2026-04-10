// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_COMM__DIAGNOSTIC_HELPER_HPP_
#define CRANE_COMM__DIAGNOSTIC_HELPER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <functional>
#include <string>

namespace crane
{

/**
 * @brief diagnostic_updater::Updater のライフサイクルを管理するヘルパークラス
 *
 * 各ノードに散在していた Updater 初期化パターン（setHardwareID + add）を
 * コンストラクタ1行に集約する。DiagnosedPublisher と同じコンポジション哲学。
 *
 * 使用例:
 * @code
 * // ヘッダ
 * DiagnosticHelper diagnostic_helper_;
 *
 * // 初期化子リスト
 * diagnostic_helper_(this, "my_node", "my_node/status", this, &MyNode::updateDiagnostics)
 *
 * // コールバック/タイマー内
 * diagnostic_helper_.forceUpdate();
 * @endcode
 */
class DiagnosticHelper
{
public:
  /// メンバ関数ポインタ版（既存パターン互換）
  template <typename NodePtrT, typename T>
  DiagnosticHelper(
    NodePtrT node, const std::string & hardware_id, const std::string & task_name, T * obj,
    void (T::*callback)(diagnostic_updater::DiagnosticStatusWrapper &))
  : updater_(node)
  {
    updater_.setHardwareID(hardware_id);
    updater_.add(task_name, obj, callback);
  }

  /// std::function版（ラムダ対応）
  template <typename NodePtrT>
  DiagnosticHelper(
    NodePtrT node, const std::string & hardware_id, const std::string & task_name,
    std::function<void(diagnostic_updater::DiagnosticStatusWrapper &)> callback)
  : updater_(node)
  {
    updater_.setHardwareID(hardware_id);
    updater_.add(task_name, std::move(callback));
  }

  auto forceUpdate() -> void { updater_.force_update(); }

  /// 追加タスク登録が必要な場合のアクセサ
  auto updater() -> diagnostic_updater::Updater & { return updater_; }

private:
  diagnostic_updater::Updater updater_;
};

}  // namespace crane

#endif  // CRANE_COMM__DIAGNOSTIC_HELPER_HPP_
