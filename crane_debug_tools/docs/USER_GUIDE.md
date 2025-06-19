# ユーザーガイド: Crane Debug Tools

ロボットスキルのテストとデバッグに crane_debug_tools を効果的に使用するために必要なすべての情報を網羅した包括的ガイドです。

## 目次
1. [始め方](#始め方)
2. [CLI インターフェース](#cli-インターフェース)
3. [バッチ操作](#バッチ操作)
4. [シナリオテスト](#シナリオテスト)
5. [マルチロボット連携](#マルチロボット連携)
6. [高度な使用方法](#高度な使用方法)
7. [トラブルシューティング](#トラブルシューティング)
8. [ベストプラクティス](#ベストプラクティス)

## 始め方

### 前提条件

crane_debug_tools を使用する前に、以下を確認してください：

1. **ROS 2 Jazzy** が適切にインストール・設定されている
2. **Crane ワークスペース** がビルド・読み込み済み
3. **crane_robot_skills** アクションサーバーが実行中

### インストールとセットアップ

```bash
# デバッグツールをビルド
colcon build --packages-select crane_debug_tools

# ワークスペースを読み込み
source install/local_setup.bash

# インストールを確認
crane_skill --help
```

### 最初のステップ

1. **利用可能なスキルを確認：**
   ```bash
   crane_skill list
   ```

2. **基本機能をテスト：**
   ```bash
   crane_skill run Sleep 0 duration:1.0
   ```

3. **インタラクティブモードを起動：**
   ```bash
   ros2 run crane_debug_tools crane_skill_cli
   ```

## CLI インターフェース

### インタラクティブ CLI モード

インタラクティブ CLI はスキルテスト用の対話型インターフェースを提供します：

```bash
ros2 run crane_debug_tools crane_skill_cli
```

**利用可能なコマンド：**
- `help, h` - ヘルプ情報を表示
- `list, l` - 利用可能なスキルをリスト表示
- `run <skill> <robot_id> [params]` - スキルを実行
- `robots <n>` - アクティブロボット数を設定
- `quit, exit, q` - プログラムを終了

**Example Session:**
```
=== Crane Skill Tester CLI ===
> list
Available skills:
  Sleep     Idle      Kick      Receive
  Goalie    Attacker  SubAttacker  StealBall
  ...

> robots 6
Set number of robots to 6

> run EmplaceRobot 0 target_x:1.0 target_y:2.0 target_theta:0.5
Executing skill 'EmplaceRobot' on robot 0 with parameters: target_x=1.0 target_y=2.0 target_theta=0.5
Goal accepted by server, executing...
Skill execution succeeded with result: 0

> run Kick 0 target_x:3.0 target_y:1.0 kick_power:5.0
Executing skill 'Kick' on robot 0 with parameters: target_x=3.0 target_y=1.0 kick_power=5.0
Goal accepted by server, executing...
Feedback: Ball detected, adjusting position
Feedback: Kick trajectory calculated
Skill execution succeeded with result: 0
```

### 直接 CLI コマンド

迅速な操作とスクリプト実行には、直接コマンドインターフェースを使用します：

```bash
# 基本構文
crane_skill <command> [arguments]
```

**使用可能なコマンド：**

#### `list` - 利用可能なスキルを表示
```bash
crane_skill list
```

#### `run` - 単一スキルの実行
```bash
crane_skill run <skill_name> <robot_id> [param1:value1] [param2:value2] ...
```

**例：**
```bash
# パラメータなしの簡単なスキル
crane_skill run Idle 0

# パラメータ付きスキル
crane_skill run Kick 0 target_x:1.5 target_y:0.5 kick_power:4.0

# ロボット配置
crane_skill run EmplaceRobot 1 target_x:2.0 target_y:1.0 target_theta:1.57

# 動作テスト
crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0
```

#### `multi` - マルチロボット実行
```bash
crane_skill multi <skill_name> <robot_ids> [parameters]
```

**例：**
```bash
# 複数ロボットでスキル実行
crane_skill multi Idle 0,1,2

# マルチロボット配置
crane_skill multi EmplaceRobot 0,1,2,3 target_x:1.0 target_y:0.0

# フォーメーション設定
crane_skill multi Attacker 1,2,3
```

#### `scenario` - テストシナリオの実行
```bash
crane_skill scenario <scenario_file.json>
```

**例：**
```bash
crane_skill scenario scenarios/basic_skills_test.json
```

## バッチ操作

### パラメータの型と形式

システムは自動的にパラメータの型を検出します：

| 型 | 形式 | 例 |
|------|--------|---------|
| **ブール値** | `true`/`false` | `enable_kick:true` |
| **整数** | 整数 | `robot_count:5` |
| **浮動小数点** | 小数 | `target_x:1.5`, `kick_power:3.14` |
| **文字列** | テキスト | `mode:attack`, `strategy:defensive` |

### 一般的なパラメータパターン

**位置パラメータ：**
```bash
# 2D位置
target_x:1.0 target_y:2.0

# 向き付き3D位置
target_x:1.0 target_y:2.0 target_theta:0.5

# 速度
velocity_x:0.5 velocity_y:0.3
```

**スキル固有のパラメータ：**
```bash
# キックパラメータ
kick_power:5.0 chip_enable:false

# 動作パラメータ
max_velocity:2.0 acceleration:1.0

# タイミングパラメータ
duration:3.0 delay:1.5
```

### スクリプト統合

**Bashスクリプトの例：**
```bash
#!/bin/bash
# フォーメーション設定スクリプト

echo "守備フォーメーションを設定中..."

# ゴールキーパーを配置
crane_skill run EmplaceRobot 0 target_x:-3.0 target_y:0.0 target_theta:0.0

# ディフェンダーを配置
crane_skill multi EmplaceRobot 1,2 target_x:-1.5 target_y:1.0
crane_skill run EmplaceRobot 2 target_x:-1.5 target_y:-1.0 target_theta:0.0

# 守備行動を起動
crane_skill run Goalie 0
crane_skill multi SecondThreatDefender 1,2

echo "守備フォーメーション準備完了！"
```

**Pythonスクリプトの例：**
```python
#!/usr/bin/env python3
import subprocess
import time

def execute_skill(skill, robot_id, **params):
    """パラメータ付きでスキルを実行"""
    cmd = ['crane_skill', 'run', skill, str(robot_id)]
    for key, value in params.items():
        cmd.append(f"{key}:{value}")
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.returncode == 0

# テストシーケンス
robots = [0, 1, 2]

# ロボットを配置
for i, robot in enumerate(robots):
    success = execute_skill('EmplaceRobot', robot, 
                          target_x=i*1.0, target_y=0.0, target_theta=0.0)
    if success:
        print(f"ロボット {robot} の配置が成功しました")
    time.sleep(1)

# 行動を起動
execute_skill('Goalie', 0)
execute_skill('Attacker', 1)
execute_skill('SubAttacker', 2)
```

## シナリオテスト

### シナリオファイル形式

シナリオは以下の構造でJSON形式で定義されます：

```json
{
  "name": "シナリオ名",
  "description": "このシナリオがテストする内容の説明",
  "skills": [
    {
      "name": "SkillName",
      "robot_id": 0,
      "parameters": {
        "param1": "value1",
        "param2": 123.45
      },
      "delay": 2.0,
      "description": "このステップの説明"
    }
  ]
}
```

### カスタムシナリオの作成

**基本シナリオの例：**
```json
{
  "name": "ボールハンドリングテスト",
  "description": "基本的なボールハンドリングスキルをテスト",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {
        "target_x": 0.0,
        "target_y": 0.0,
        "target_theta": 0.0
      },
      "delay": 2,
      "description": "ロボットをボール近くに配置"
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {
        "target_x": 2.0,
        "target_y": 0.0,
        "kick_power": 3.0
      },
      "delay": 3,
      "description": "ボールを前方にキック"
    },
    {
      "name": "Receive",
      "robot_id": 0,
      "parameters": {},
      "delay": 0,
      "description": "レシーブモードに切り替え"
    }
  ]
}
```

**複雑なマルチロボットシナリオ：**
```json
{
  "name": "連携攻撃",
  "description": "連携攻撃行動をテスト",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": -2.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 0,
      "description": "ゴールキーパーを配置"
    },
    {
      "name": "EmplaceRobot", 
      "robot_id": 1,
      "parameters": {"target_x": 0.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 0,
      "description": "メインアタッカーを配置"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 2,
      "parameters": {"target_x": -0.5, "target_y": 1.0, "target_theta": 0.0},
      "delay": 0,
      "description": "サポートアタッカーを配置"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 3,
      "parameters": {"target_x": -0.5, "target_y": -1.0, "target_theta": 0.0},
      "delay": 2,
      "description": "第二サポートを配置"
    },
    {
      "name": "Goalie",
      "robot_id": 0,
      "parameters": {},
      "delay": 0,
      "description": "ゴールキーパーを起動"
    },
    {
      "name": "Attacker",
      "robot_id": 1,
      "parameters": {},
      "delay": 0,
      "description": "メインアタッカーを起動"
    },
    {
      "name": "SubAttacker",
      "robot_id": 2,
      "parameters": {},
      "delay": 0,
      "description": "サポートアタッカーを起動"
    },
    {
      "name": "SubAttacker",
      "robot_id": 3,
      "parameters": {},
      "delay": 5,
      "description": "第二サポートを起動"
    }
  ]
}
```

### シナリオ実行

```bash
# シナリオを実行
crane_skill scenario my_scenario.json

# 詳細出力付き
crane_skill scenario my_scenario.json --verbose

# エラー時も継続（実装されている場合）
crane_skill scenario my_scenario.json --continue-on-error
```

**期待される出力：**
```
--- スキル 1/4 を実行中 ---
ロボット 0 でスキル 'EmplaceRobot' を実行
パラメータ: target_x:0.0 target_y:0.0 target_theta:0.0
✓ スキル実行が成功しました

次のスキルまで 2 秒待機中...

--- スキル 2/4 を実行中 ---
ロボット 0 でスキル 'Kick' を実行
パラメータ: target_x:2.0 target_y:0.0 kick_power:3.0
✓ スキル実行が成功しました

--- シナリオ完了: 4/4 スキルが成功 ---
```

## マルチロボット連携

### 並列実行

複数のロボットで同じスキルを同時に実行します：

```bash
# 基本的なマルチロボットコマンド
crane_skill multi <skill_name> <robot_list> [parameters]

# 例
crane_skill multi Idle 0,1,2,3,4,5
crane_skill multi EmplaceRobot 1,2,3 target_x:1.0 target_y:0.0
crane_skill multi Attacker 0,1,2
```

### フォーメーションテスト

**守備フォーメーション：**
```bash
# 迅速守備設定
crane_skill run EmplaceRobot 0 target_x:-3.0 target_y:0.0 target_theta:0.0  # ゴーリー
crane_skill multi EmplaceRobot 1,2 target_x:-1.5 target_y:1.0               # ディフェンダー
crane_skill run Goalie 0
crane_skill multi SecondThreatDefender 1,2
```

**攻撃フォーメーション：**
```bash
# 攻撃フォーメーション
crane_skill multi EmplaceRobot 0,1,2 target_x:1.0 target_y:0.0    # フォワードライン
crane_skill multi EmplaceRobot 3,4 target_x:0.0 target_y:1.0      # ミッドフィールド
crane_skill multi Attacker 0,1,2
crane_skill multi SubAttacker 3,4
```

### 連携シナリオ

**パスシーケンス：**
```json
{
  "name": "パスチェーン",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": -1.0, "target_y": 0.0},
      "delay": 0
    },
    {
      "name": "EmplaceRobot", 
      "robot_id": 1,
      "parameters": {"target_x": 0.0, "target_y": 1.0},
      "delay": 0
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 2,
      "parameters": {"target_x": 1.0, "target_y": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {"target_x": 0.0, "target_y": 1.0, "kick_power": 2.0},
      "delay": 1
    },
    {
      "name": "Receive",
      "robot_id": 1,
      "parameters": {},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 1,
      "parameters": {"target_x": 1.0, "target_y": 0.0, "kick_power": 2.0},
      "delay": 1
    },
    {
      "name": "Receive",
      "robot_id": 2,
      "parameters": {},
      "delay": 0
    }
  ]
}
```

## 高度な使用方法

### 環境変数

環境変数で動作を設定します：

```bash
# デフォルトロボット数を設定
export CRANE_DEBUG_ROBOT_COUNT=6

# アクションサーバーのタイムアウトを設定
export CRANE_DEBUG_TIMEOUT=30

# 詳細ログを有効化
export CRANE_DEBUG_VERBOSE=1

# カスタムアクションサーバートピックを使用
export CRANE_DEBUG_ACTION_TOPIC="/custom/skill_execution"
```

### カスタムパラメータ検証

特定のパラメータ組み合わせが必要なスキルの場合：

```bash
# 検証付きキック
crane_skill run Kick 0 target_x:3.0 target_y:0.0 kick_power:5.0 chip_enable:false

# 制約付き動作
crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0 max_velocity:2.0
```

### ROS 2 ツールとの統合

**実行の監視：**
```bash
# アクションサーバーを監視
ros2 action list | grep skill_execution

# トピックを監視
ros2 topic echo /world_model
ros2 topic echo /robot_commands

# ノードグラフを確認
ros2 node list | grep crane
```

**パラメータの検査：**
```bash
# ロボットスキルノードのパラメータを確認
ros2 param list /crane_robot_skills

# パラメータ値を取得
ros2 param get /crane_robot_skills use_world_model
```

### パフォーマンス監視

**実行時間の測定：**
```bash
# スキル実行時間を測定
time crane_skill run Kick 0 target_x:1.0 target_y:2.0

# シナリオ実行時間を測定
time crane_skill scenario scenarios/complex_test.json
```

**リソース監視：**
```bash
# 実行中の監視
htop &
crane_skill scenario scenarios/stress_test.json
```

## トラブルシューティング

### 一般的な問題

#### 1. アクションサーバーが利用不可

**エラー：** `Action server not available after waiting`

**解決方法：**
```bash
# crane_robot_skills が実行中か確認
ros2 node list | grep crane_robot_skills

# アクションサーバーを確認
ros2 action list | grep skill_execution

# crane システムを再起動
ros2 launch crane_bringup crane.launch.py
```

#### 2. パラメータ型エラー

**エラー：** `Invalid parameter type`

**解決方法：**
```bash
# パラメータ形式を確認
crane_skill run Kick 0 target_x:1.0  # 正しい（浮動小数点）
crane_skill run Kick 0 target_x:1    # 整数と解釈される可能性

# 浮動小数点には明示的な小数表記を使用
crane_skill run Kick 0 kick_power:5.0  # 単に "5" ではなく
```

#### 3. ロボットIDの範囲外

**エラー：** `Robot ID must be between 0 and 15`

**解決方法：**
```bash
# 有効なロボット範囲を確認
crane_skill list  # 有効なロボットを表示

# 有効なロボットIDを使用
crane_skill run Kick 0 target_x:1.0    # ロボット 0（有効）
crane_skill run Kick 16 target_x:1.0   # ロボット 16（無効）
```

#### 4. スキル実行タイムアウト

**エラー：** `Skill execution timed out`

**解決方法：**
```bash
# ロボットの状態を確認
ros2 topic echo /world_model --once

# ロボットが表示され応答しているか確認
ros2 topic echo /robot_commands --once

# まず簡単なスキルを試す
crane_skill run Idle 0
```

### デバッグコマンド

**システム状態の確認：**
```bash
# ROS 2 環境
ros2 node list
ros2 topic list
ros2 action list

# Crane 固有
ros2 topic echo /world_model --once
ros2 param list /crane_robot_skills
```

**詳細実行：**
```bash
# デバッグ出力を有効化（利用可能な場合）
export ROS_LOG_LEVEL=DEBUG
crane_skill run Kick 0 target_x:1.0 target_y:2.0
```

**ネットワーク診断：**
```bash
# ROS 2 ディスカバリを確認
ros2 doctor

# ネットワーク接続性
ros2 multicast send
ros2 multicast receive
```

## ベストプラクティス

### スキルテストワークフロー

1. **簡単なことから始める：**
   ```bash
   crane_skill run Idle 0
   crane_skill run Sleep 0 duration:1.0
   ```

2. **動作をテスト：**
   ```bash
   crane_skill run EmplaceRobot 0 target_x:0.0 target_y:0.0
   crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0
   ```

3. **ゲームスキルをテスト：**
   ```bash
   crane_skill run Kick 0 target_x:2.0 target_y:0.0 kick_power:3.0
   crane_skill run Receive 0
   ```

4. **複雑な行動をテスト：**
   ```bash
   crane_skill run Attacker 0
   crane_skill run Goalie 1
   ```

### シナリオ開発

1. **漸進的テスト：**
   - 単一ロボットシナリオから始める
   - 復雑さを段階的に追加
   - 各ステップを個別にテスト

2. **ドキュメント化：**
   - 説明的な名前と説明を使用
   - 複雑なパラメータ選択にコメントを追加
   - 期待される結果を含める

3. **エラーハンドリング：**
   - 失敗ケースを計画
   - スキル間に適切な遅延を使用
   - 複雑な操作前にロボット位置を検証

### マルチロボットテスト

1. **連携：**
   - まず個別ロボットをテスト
   - チームサイズを段階的に増加
   - ロボット間の干渉を監視

2. **タイミング：**
   - ロボットの動作に適切な遅延を使用
   - 通信レイテンシを考慮
   - 同期シナリオをテスト

3. **検証：**
   - グループ操作前にロボット位置を確認
   - 衝突回避を確認
   - チーム連携の効果を監視

このユーザーガイドは、基本操作から高度なマルチロボット連携テストまで、crane_debug_tools の使用方法を包括的に網羅しています。