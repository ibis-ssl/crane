# Crane Debug Tools

Craneロボットサッカーシステム用の現代的なデバッグ・テストツール。Qt ベースの `crane_simple_ai` をより柔軟で現代的な代替手段に置き換えるために設計されています。

## 機能

### 🌐 Web ベースインターフェース
- どのデバイスからでもアクセス可能な現代的なレスポンシブ Web インターフェース
- ロボット位置とボール追跡のリアルタイム可視化
- パラメータ設定によるインタラクティブなスキル実行
- ライブ実行ログとステータス監視

### 🖥️ コマンドラインインターフェース（CLI）
- 素早いスキルテスト用の軽量 CLI ツール
- シナリオファイルによるバッチ実行
- マルチロボット連携テスト
- スクリプト化・自動化対応

### 🔧 拡張機能
- 全ての crane ロボットスキルをサポート
- リアルタイムパラメータ調整
- マルチロボット連携テスト
- 自動テストシナリオ実行
- パフォーマンス監視とログ記録

## クイックスタート

### インストール

ツールは crane ワークスペースと一緒に自動的にビルドされます：

```bash
# ツールをビルド
colcon build --packages-select crane_debug_tools

# ワークスペースを読み込み
source install/local_setup.bash
```

### Web インターフェース

1. **デバッグツールを起動：**
   ```bash
   ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=true
   ```

2. **ブラウザを開いて以下に移動：**
   ```
   http://localhost:8081
   ```

3. **インターフェースの使用開始：**
   - 左パネルからスキルを選択
   - ロボット ID とパラメータを設定
   - "Execute Skill" をクリックして実行

### CLI インターフェース

1. **インタラクティブ CLI モード：**
   ```bash
   ros2 run crane_debug_tools crane_skill_cli
   ```

2. **直接スキル実行：**
   ```bash
   # パラメータ付きでスキルを実行
   crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
   
   # 複数ロボットで実行
   crane_skill multi Attacker 0,1,2
   
   # 利用可能なスキルをリスト表示
   crane_skill list
   ```

3. **シナリオ実行：**
   ```bash
   crane_skill scenario test_sequence.json
   ```

## 利用例

### 基本的なスキルテスト

**Web インターフェース：**
1. スキルリストから "Kick" スキルを選択
2. ロボット ID を 0 に設定
3. パラメータを設定：target_x=1.0, target_y=2.0, kick_power=5.0
4. "Execute Skill" をクリック

**CLI：**
```bash
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
```

### マルチロボット連携

**CLI 例：**
```bash
# ロボット 0、1、2 で Attacker スキルを実行
crane_skill multi Attacker 0,1,2
```

### 自動テストシナリオ

JSON シナリオファイルを作成（`test_sequence.json`）：
```json
{
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": 1.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {"target_x": 3.0, "target_y": 1.0, "kick_power": 5.0},
      "delay": 1
    }
  ]
}
```

シナリオを実行：
```bash
crane_skill scenario test_sequence.json
```

## 利用可能なスキル

デバッグツールは全ての crane ロボットスキルをサポートしています：

**基本スキル：**
- `Sleep` - 指定時間ロボットを停止
- `Idle` - ロボットをアイドル状態に保持
- `EmplaceRobot` - 指定位置・姿勢にロボットを移動

**ゲームスキル：**
- `Kick` - 目標に向かってボールをキック
- `Receive` - ボールを受け取る位置に移動
- `Goalie` - ゴールキーパーの動作
- `Attacker` - 攻撃動作
- `SubAttacker` - サポート攻撃動作
- `StealBall` - 積極的なボール奪取

**フォーメーションスキル：**
- `SingleBallPlacement` - セットピース用のボール配置
- `GoalKick` - ゴールキック実行
- `SimpleKickOff` - 基本キックオフ動作
- `KickOffAttack` - 攻撃的キックオフ
- `KickOffSupport` - サポートキックオフ

**テストスキル：**
- `TestMotionPosition` - 位置制御テスト
- `TestMotionVelocity` - 速度制御テスト
- `Marker` - デバッグ用視覚マーカー
- `Teleop` - 手動ロボット制御

## 設定

### 起動パラメータ

```bash
# カスタム Web ポート
ros2 launch crane_debug_tools debug_tools.launch.py web_port:=9090

# CLI ツールのみ有効化
ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=false enable_cli:=true
```

### Web インターフェース設定

Web インターフェースはポート 8080 の WebSocket サーバーに自動接続します。ポートを変更する場合は、`web/app.js` の JavaScript 設定を更新してください。

## アーキテクチャ

### Web ブリッジサーバー
- **目的：** ROS 2 と Web インターフェース間の橋渡し
- **技術：** JSON メッセージング付き WebSocket サーバー
- **機能：** リアルタイムデータストリーミング、スキル実行、パラメータ管理

### CLI ツール
- **crane_skill_cli：** スキルテスト用インタラクティブ CLI
- **crane_skill：** バッチ操作と自動化用 Python スクリプト

### 通信フロー
```
Web インターフェース ←→ WebSocket ブリッジ ←→ ROS 2 アクション ←→ crane_robot_skills
CLI ツール ←→ ROS 2 アクション ←→ crane_robot_skills
```

## Crane システムとの統合

デバッグツールは既存の crane システムとシームレスに統合されます：

1. **アクションインターフェース：** `crane_simple_ai` と同じ `SkillExecution` アクションを使用
2. **ワールドモデル：** 可視化のためのワールドモデル更新を購読
3. **ロボットコマンド：** デバッグ用にロボットコマンドを監視
4. **起動統合：** メイン crane 起動で有効/無効を切り替え可能

### crane_simple_ai の置き換え

Qt ベースのシンプル AI の代わりに新しいデバッグツールを使用するには：

```bash
# 旧方式（Qt GUI）
ros2 launch crane_bringup crane.launch.py simple_ai:=true

# 新方式（Web インターフェース）
ros2 launch crane_bringup crane.launch.py
ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=true
```

## 開発

### 依存関係
- ROS 2 Jazzy
- WebSocket++（Web ブリッジ用）
- nlohmann/json（JSON 処理用）
- Bootstrap 5（Web UI 用）

### ビルド
```bash
colcon build --packages-select crane_debug_tools
```

### テスト
```bash
colcon test --packages-select crane_debug_tools
```

## crane_simple_ai に対する利点

1. **クロスプラットフォーム：** Web インターフェースはブラウザ搭載のあらゆるデバイスで動作
2. **リモートデバッグ：** ネットワーク上のどこからでもデバッグツールにアクセス可能
3. **現代的な UI：** レスポンシブでモバイル対応のインターフェース
4. **自動化：** CLI ツールがスクリプト化とバッチ操作をサポート
5. **マルチロボット：** 複数ロボット連携の組み込みサポート
6. **リアルタイム：** ロボット位置とゲーム状態のライブ更新
7. **拡張可能：** 新機能と可視化の追加が容易

## トラブルシューティング

### Web インターフェースが読み込まれない
- ポート 8081 で Web サーバーが実行されているか確認
- ポート 8080 で WebSocket ブリッジが実行されているか確認
- ブラウザコンソールでエラーメッセージを確認

### CLI コマンドが動作しない
- ROS 2 環境が読み込まれているか確認
- crane_robot_skills アクションサーバーが実行されているか確認  
- ロボットスキルアクションサーバーが `/simple_ai/skill_execution` で利用可能か確認

### スキルが実行されない
- スキル実行アクションサーバーへの接続を確認
- ROS 2 ノードグラフを確認：`ros2 node list`
- アクションサーバーの状態を監視：`ros2 action list`

## コントリビューション

新しいスキルやパラメータを追加する際は：

1. Web と CLI 両方のインターフェースでスキルリストを更新
2. Web インターフェースにパラメータ定義を追加
3. ドキュメントと例を更新
4. 両方のインターフェースタイプでテスト