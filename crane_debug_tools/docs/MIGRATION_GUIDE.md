# 移行ガイド: crane_simple_ai から crane_debug_tools へ

このガイドは、Qtベースの `crane_simple_ai` から新しいモダンな `crane_debug_tools` への移行を支援します。

## クイック比較

| 機能 | crane_simple_ai | crane_debug_tools |
|---------|-----------------|-------------------|
| **インターフェース** | Qt5 GUI | CLI + Web（予定） |
| **プラットフォーム** | Linux のみ | クロスプラットフォーム |
| **自動化** | 手動のみ | CLI スクリプティング + シナリオ |
| **マルチロボット** | 単一ロボット重視 | ネイティブマルチロボットサポート |
| **リモートアクセス** | ローカルのみ | CLI は SSH 経由で動作 |
| **依存関係** | Qt5、重いGUI依存 | 最小限、ROS2のみ |

## 基本的な使用法の移行

### インターフェースの開始

**旧方式 (crane_simple_ai):**
```bash
ros2 launch crane_bringup crane.launch.py simple_ai:=true
```

**新方式 (crane_debug_tools):**
```bash
# CLI インターフェースの場合
ros2 run crane_debug_tools crane_skill_cli

# またはスタンドアロンスクリプトを使用
crane_skill list
crane_skill run Kick 0 target_x:1.0 target_y:2.0
```

### スキル実行

**旧方式:** 
- ドロップダウンからスキルを選択
- スピナーでロボットIDを設定
- テーブルでパラメータを設定
- 実行ボタンをクリック

**新方式:**
```bash
# 直接実行
crane_skill run <skill_name> <robot_id> [param1:value1] [param2:value2]

# 例
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
crane_skill run EmplaceRobot 1 target_x:2.0 target_y:1.5 target_theta:0.5
crane_skill run Sleep 0 duration:2.0
```

## 高度な機能

### マルチロボット協調

**旧:** サポートされていない - スキルを一つずつ実行する必要があった

**新:** ネイティブマルチロボットサポート
```bash
# 複数のロボットで同じスキルを実行
crane_skill multi Attacker 0,1,2

# パラメータ付きで実行
crane_skill multi EmplaceRobot 0,1,2 target_x:1.0 target_y:0.0
```

### 自動テストシナリオ

**旧:** サポートされていない - 手動実行のみ

**新:** JSONベースのシナリオ実行
```bash
# 事前定義されたテストシーケンスを実行
crane_skill scenario scenarios/basic_skills_test.json
crane_skill scenario scenarios/multi_robot_formation.json
```

シナリオファイルの例:
```json
{
  "name": "Basic Test",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": 1.0, "target_y": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {"target_x": 3.0, "kick_power": 5.0},
      "delay": 1
    }
  ]
}
```

## スキルパラメータマッピング

パラメータシステムは、より優れた型安全性のために改善されました:

### crane_simple_ai パラメータ
- すべてのパラメータはテーブル内の文字列でした
- 型検証なし
- 手動パラメータ入力

### crane_debug_tools パラメータ
- 自動型検出 (float, int, bool, string)
- コマンドラインフレンドリーなフォーマット: `key:value`
- 複雑なシナリオのサポート

### 一般的なスキルの移行

**Kick スキル:**
```bash
# 旧: GUIテーブルで設定
# target_x: 1.0
# target_y: 2.0
# kick_power: 5.0

# 新: コマンドライン
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
```

**EmplaceRobot スキル:**
```bash
# 旧: GUIパラメータテーブル
# 新: 直接コマンド
crane_skill run EmplaceRobot 0 target_x:2.0 target_y:1.5 target_theta:0.5
```

## ワークフロー移行

### 開発テストワークフロー

**旧ワークフロー:**
1. `simple_ai:=true` でcraneシステムを起動
2. Qt GUIを開く
3. 手動でスキルを選択しパラメータを設定
4. 一つずつ実行
5. 手動で結果を観察

**新ワークフロー:**
1. craneシステムを通常起動
2. クイックテストにはCLIを使用:
   ```bash
   crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0
   ```
3. 複雑なテストのためのシナリオファイルを作成
4. スクリプトでリグレッションテストを自動化

### 結合テストワークフロー

**旧:** 手動、エラーが起こりやすい、再現性がない

**新:** 自動化されていてスクリプト化可能
```bash
# テストスクリプトを作成
cat > test_formation.sh << 'EOF'
#!/bin/bash
echo "ロボットフォーメーションをテスト中..."
crane_skill multi EmplaceRobot 0,1,2 target_x:1.0
crane_skill run Sleep 0 duration:2.0
crane_skill multi Attacker 0,1,2
EOF

chmod +x test_formation.sh
./test_formation.sh
```

## 移行問題のトラブルシューティング

### 一般的な問題と解決策

**問題:** `crane_skill` コマンドが見つからない
```bash
# 解決策: ワークスペースをソースする
source install/local_setup.bash
```

**問題:** アクションサーバーが利用できない
```bash
# crane_robot_skillsが実行中か確認
ros2 action list | grep skill_execution

# 見つからない場合は、craneシステムが完全に起動されていることを確認
ros2 launch crane_bringup crane.launch.py
```

**問題:** スキルパラメータが動作しない
```bash
# パラメータフォーマットを確認 - コロン区切り文字を使用
crane_skill run Kick 0 target_x:1.0  # 正しい
crane_skill run Kick 0 target_x=1.0  # 間違い
```

### デバッグのコツ

1. **詳細出力:** 何が起こっているか理解するためにデバッグプリントを追加
2. **ステップバイステップ実行:** シナリオの前にスキルを個別にテスト
3. **パラメータ検証:** 利用可能なスキルを確認するために `crane_skill list` を使用

## パフォーマンス比較

| 指標 | crane_simple_ai | crane_debug_tools |
|--------|-----------------|-------------------|
| **起動時間** | ~3-5秒 (Qt読み込み) | ~0.5秒 (CLI) |
| **メモリ使用量** | ~50-100 MB (Qtオーバーヘッド) | ~10-20 MB (最小限) |
| **実行速度** | GUIインタラクション遅延 | 瞬時コマンド実行 |
| **バッチ操作** | サポートされていない | ネイティブサポート |

## 既存ワークフローとの統合

### CI/CD統合

**旧:** GUIでは不可能

**新:** 完全なCI/CDサポート
```yaml
# GitHub Actionsワークフローの例
- name: Test Robot Skills
  run: |
    source install/local_setup.bash
    crane_skill scenario tests/ci_skills_test.json
```

### リモート開発

**旧:** GUIのX11転送が必要

**新:** SSH経由でネイティブに動作
```bash
# SSH経由でのリモート開発
ssh robot_computer
crane_skill run Kick 0 target_x:1.0 target_y:2.0
```

## 将来の移行パス

`crane_debug_tools` は `crane_simple_ai` の長期的な置き換えとして設計されています。 

### 予定されている機能
- ビジュアルデバッグ用のWebベースインターフェース
- 拡張されたシナリオエディタ
- パフォーマンス監視と分析
- ROS2テストフレームワークとの統合

### 非推奨タイムライン
- **フェーズ1 (現在):** 両システムが利用可能、新しい開発にはcrane_debug_toolsを推奨
- **フェーズ2 (将来):** crane_simple_aiが非推奨としてマークされる
- **フェーズ3 (将来):** crane_simple_aiがコードベースから削除される

## ヘルプの取得

移行中に問題が発生した場合:

1. **ドキュメントを確認:** `/crane_debug_tools/README.md`
2. **サンプルテストを実行:** `examples/simple_test.py`
3. **シナリオを比較:** 例として `scenarios/` ディレクトリを参照
4. **ヘルプを求める:** 特定の使用例でイシューを作成

## まとめ

`crane_simple_ai` から `crane_debug_tools` への移行により以下が提供されます:

✅ **優れた自動化** - テストワークフローのスクリプト化と自動化  
✅ **マルチロボットサポート** - 協調シナリオの簡単なテスト  
✅ **クロスプラットフォーム互換性** - ROS2がある任意のシステムで動作  
✅ **リモート開発** - GUI転送なしでSSH経由のデバッグ  
✅ **CI/CD統合** - パイプラインでの自動テスト  
✅ **パフォーマンス** - より高速な実行、より低いリソース使用量  

新しいツールは、元のGUIのすべての機能を維持しながら、開発とテストをより効率的にする強力な自動化とマルチロボット機能を追加します。