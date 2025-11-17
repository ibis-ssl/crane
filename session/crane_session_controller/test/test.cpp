// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "crane_session_controller/session_controller.hpp"

// TODO: 以下のメソッドに対する単体テストを追加する（依存性注入パターン導入後）
// - getAssignedRobotIds(): プランナーから正しくロボットIDを取得できるか
// - buildAssignmentLog(): 割当状況のログ文字列が正しく生成されるか
// - logAssignmentIfChanged(): 変更時のみログ出力されるか
// - tryAssignRobotToPlanner(): ロボット割り当てが正しく動作するか、エラーハンドリングが正しく機能するか

TEST(SessionController, dummy_test)
{
  // 現在はダミーテスト
  // 依存性注入パターン導入後に本格的なテストを追加予定
  ASSERT_NEAR(1, 1, 1e-5);
}
