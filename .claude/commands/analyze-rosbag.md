---
description: "craneのrosbagを解析して試合状況・ロボット動作・異常を診断"
allowed-tools: ["Bash", "Read", "Glob", "Grep", "Agent"]
---

# Crane Rosbag 解析

crane ROS 2 rosbag（MCAP形式）を解析して、試合状況・ロボット動作・異常を診断します。

## 引数

```text
$ARGUMENTS
```

引数の解釈:

- `<rosbag_path>` — 解析するrosbagディレクトリのパス（必須）
- 追加テキスト — 分析の焦点（例: "Attackerが止まった原因", "ゴールキーパーの動き"）

## 実行手順

### Step 1: Bag情報の確認

```bash
cd /home/hans/workspace/ibis_ws && source install/setup.bash 2>/dev/null
ros2 run crane_debug_tools crane_bag info <rosbag_path>
```

以下を確認:

- 収録時間（Duration）
- 主要トピックのメッセージ数（`/world_model`, `/robot_commands`, `/play_situation` など）

### Step 2: 概要サーベイ

```bash
ros2 run crane_debug_tools crane_bag survey <rosbag_path>
```

出力される7セクション:

- **PLAY SITUATIONS** — ゲーム状態遷移の全履歴
- **ROLE ASSIGNMENTS (last)** — 最終ロールアサイン
- **WORLD MODEL** — ボール・ロボット位置（5秒サンプリング）
- **CONTROL_TARGETS: UNIQUE PLANNING_FACTORS** — 各ロボットのスキル状態変化
- **ROBOT VELOCITY STATUS** — 各ロボットの速度状態（10秒サンプリング）
- **GAME ANALYSIS** — アタッカー推奨・パス情報（5秒サンプリング）
- **ROSOUT (WARN/ERROR)** — 重要ログ（重複排除済み）

### Step 3: 深掘り分析

Step 2の結果を踏まえ、ユーザーの質問に合わせて追加調査を行う。

#### ロボットが止まった原因を調べる場合

```bash
# ロボット追跡（位置・速度・ボール距離）
ros2 run crane_debug_tools crane_bag track <path> --robot <id> --interval 0.5
# 時間範囲を絞る場合（例: 10秒〜30秒）
ros2 run crane_debug_tools crane_bag track <path> --robot <id> --time 10.0:30.0

# planning_factors の変化だけを抽出
ros2 run crane_debug_tools crane_bag control <path> --robot <id> --changes-only
# control_target の詳細（位置ターゲット・速度コマンド）
ros2 run crane_debug_tools crane_bag control <path> --robot <id> --time 10.0:30.0
```

#### イベントを調べる場合

```bash
# 全イベント（ゴール・プレイ遷移・ロール変更・キック・ボール急加速）
ros2 run crane_debug_tools crane_bag events <path>
# 種類を絞る: goal, play, role, kick, ball_speed
ros2 run crane_debug_tools crane_bag events <path> --type kick ball_speed
```

#### 敵ロボットを追跡する場合

```bash
ros2 run crane_debug_tools crane_bag track <path> --robot <id> --enemy
```

## フィールドアクセスリファレンス（Python深掘り用）

Step 3でさらに詳細が必要な場合のPythonフィールドリファレンス:

```python
# ===== WorldModel =====
ball = msg.ball_info
ball.position.x, ball.position.y   # ボール位置（NOT ball.pose）
ball.velocity.x, ball.velocity.y   # ボール速度
ball.velocity_norm                  # 速度の大きさ
ball.detected, ball.tracker_detected
ball.state                          # 0=STOPPED, 1=ROLLING, 2=FLYING
for r in msg.robot_info_ours:
    r.id, r.pose.x, r.pose.y, r.pose.theta
    r.velocity.x, r.velocity.y
    r.available_vision, r.has_error  # 注意: r.detected は存在しない

# ===== RobotCommand（control_targets / robot_commands の要素）=====
rc.robot_id, rc.stop_flag
rc.control_mode   # 1=SIMPLE_VELOCITY, 2=POSITION_TARGET, 3=POLAR_VELOCITY
rc.kick_power, rc.dribble_power, rc.chip_enable
rc.target_theta
rc.planner_name
rc.polar_velocity_target_mode   # list型（len==0 or 1）
  pv = rc.polar_velocity_target_mode
  pv[0].target_velocity_r, pv[0].target_velocity_theta
rc.position_target_mode         # list型（len==0 or 1）
  pt = rc.position_target_mode
  pt[0].target_x, pt[0].target_y, pt[0].position_tolerance
rc.planning_factors             # list[NamedString]型 — スキル状態の追跡に最重要
  # .name: スキル名や属性（例: "Kick", "Attacker", "CommandAction"）
  # .value: 状態値（例: "AROUND_BALL_AND_KICK", "KICK::GOAL_KICK", "STOP_HERE"）

# ===== PlaySituation =====
msg.command.name   # 例: "INPLAY", "OUR_KICKOFF_START", "HALT"
msg.command.value  # 数値
msg.reason_text    # 遷移理由の説明文

# ===== GameAnalysis =====
msg.recommended_attacker_id      # 推奨アタッカーID（-1=未選択）
msg.attacker_suitability_score   # アタッカー適性スコア
msg.pass_target_id               # パス先ID（-1=未選択）
msg.recommended_pass_receiver_id
msg.ongoing_kick                 # list型（キック中かどうか）
msg.ball_threat, msg.our_slack, msg.their_slack
msg.has_sub_attacker_position

# ===== RobotSelectResults =====
for r in msg.results:
    r.name              # ロール名（例: "attacker_skill", "goalie_skill", "defender"）
    r.selected_robots   # 選択されたロボットIDリスト
    r.selectable_robots # 選択可能なロボットIDリスト
    r.min_robots_num, r.max_robots_num
```
