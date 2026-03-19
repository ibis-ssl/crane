---
description: "craneのrosbagを解析して試合状況・ロボット動作・異常を診断"
allowed-tools: ["Bash", "Read", "Glob", "Grep", "Agent"]
---

# Crane Rosbag 解析

crane ROS 2 rosbag（MCAP形式）を解析して、試合状況・ロボット動作・異常を診断します。

コマンドプレフィックス（以降 `crane_bag` と略記）:

```bash
cd /home/hans/workspace/ibis_ws && source install/setup.bash 2>/dev/null
# crane_bag <subcommand> ...  は以下の意味:
# ros2 run crane_bag crane_bag <subcommand> ...
```

## 引数

```text
$ARGUMENTS
```

- `<rosbag_path>` — 解析するrosbagディレクトリのパス（必須）
- 追加テキスト — 分析の焦点（例: "Attackerが止まった原因", "ゴールキーパーの動き"）

## 実行手順

### Step 1: Bag情報の確認

```bash
ros2 run crane_bag crane_bag info <rosbag_path>
```

収録時間（Duration）と主要トピックのメッセージ数（`/world_model`, `/robot_commands`, `/play_situation` 等）を確認する。

### Step 2: 概要サーベイ

```bash
ros2 run crane_bag crane_bag survey <rosbag_path>
```

出力される7セクション（PLAY SITUATIONS / ROLE ASSIGNMENTS / WORLD MODEL / CONTROL_TARGETS / ROBOT VELOCITY STATUS / GAME ANALYSIS / ROSOUT WARN/ERROR）を確認し、異常箇所を特定する。

**重要**: WORLD MODEL セクションのヘッダーに `[OUR_TEAM=YELLOW]` または `[OUR_TEAM=BLUE]` が表示される。これを確認し、以降の分析レポートでは必ず「味方=○色」「敵=○色」を明記すること。`our[ID]` ラベルは crane（自チーム）のロボット、`theirs` は相手チームを指す。

### Step 3: 深掘り分析

Step 2の結果とユーザーの質問に合わせて追加調査を行う。

| 調査目的 | サブコマンド | 主要オプション |
|---------|------------|--------------|
| ロボット位置・速度の時系列 | `track` | `--robot <id>` / `--ball` / `--interval <秒>` / `--time <s>:<e>` |
| planning_factors の変化 | `control` | `--robot <id>` / `--changes-only` / `--time <s>:<e>` |
| ゴール・キック・ファウル等 | `events` | `--type goal kick ball_speed foul role play` |
| 敵ロボット追跡 | `track` | `--robot <id> --enemy` |
| レフェリーコマンド | `referee` | デフォルト: サンプリング表示 / `--changes-only`: 遷移のみ |

代表的なコマンド例:

```bash
# ロボット追跡（0.5秒間隔・10〜30秒範囲）
ros2 run crane_bag crane_bag track <path> --robot <id> --interval 0.5 --time 10.0:30.0

# planning_factors の変化のみ抽出
ros2 run crane_bag crane_bag control <path> --robot <id> --changes-only

# キック・ファウルイベント
ros2 run crane_bag crane_bag events <path> --type kick foul

# レフェリーコマンド遷移のみ
ros2 run crane_bag crane_bag referee <path> --changes-only

# JSON + jq 連携（ゴール時刻抽出 / ファウル集計 / 停止フレーム抽出）
ros2 run crane_bag crane_bag events <path> --type goal --format json | jq -r '.[].t'
ros2 run crane_bag crane_bag events <path> --type foul --format json \
  | jq '[.[] | {type: (.description | split(":")[0])}] | group_by(.type) | map({type: .[0].type, count: length})'
ros2 run crane_bag crane_bag track <path> --robot 0 --format json | jq '.[] | select(.speed < 0.01)'
```

Pythonで直接解析が必要な場合は `.claude/commands/rosbag-python-reference.md` を Read で参照すること。
