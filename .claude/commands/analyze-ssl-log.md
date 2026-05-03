---
name: analyze-ssl-log
description: "RoboCup SSL公式ログ(.log.gz)を解析して試合状況・referee・ロボット動作を診断"
allowed-tools: ["Bash", "Read", "Glob", "Grep", "Agent"]
---

# SSL公式ログ解析

RoboCup SSL公式レコーダーが記録する `.log.gz` 形式のログを解析します。
SSL公式ツール群（ssl-vision, ssl-game-controller等）がUDPで配信するパケットをレコーダーがキャプチャしたもので、
ボール・ロボット位置（Vision/Tracker）とreferee状態（スコア・ファウル・ゲームイベント）が含まれます。

コマンドプレフィックス:

```bash
# craneリポジトリのルートで実行する
# 以降 ssl_log <subcommand> ... は以下の意味:
# python3 scripts/analyze_ssl_log.py <subcommand> ...
```

## 引数

```text
$ARGUMENTS
```

- `<log_path>` — 解析する `.log.gz`（または `.log`）ファイルのパス（必須）
- 追加テキスト — 分析の焦点（例: "ゴール後の判定", "ファウルが多発した時間帯", "ロボット3番の動き"）

## 実行手順

### Step 0: ファイル確認

```bash
LOG="<log_path>"
ls -lh "$LOG"
```

`.log.gz` または `.log` 形式であることを確認する。
初回実行時のみ `scripts/ssl_proto/` に protobuf Pythonバインディングが自動生成される（数秒かかる）。
`grpcio-tools` または `protoc` がインストールされていない場合は事前に `pip install grpcio-tools` を実行する。

> **注意**: `.log.gz` は SSL公式レコーダーが記録した形式（Vision/Tracker/Refereeパケットを含む）。
> ROS bag（.mcap）の解析には `/analyze-rosbag` を使用すること。

### Step 1: ログ基本情報

```bash
python3 scripts/analyze_ssl_log.py info "$LOG"
```

時間範囲、チーム名、最終スコア、メッセージ数（Referee/Vision/Tracker）を確認する。

> Referee: 0件 の場合、このログには referee 情報が含まれていない（Vision専用ログの可能性）。
> ボールフレーム: 0件 の場合、Trackerデータなし（Vision 2014のみのログ）。

### Step 2: 試合概要サーベイ

```bash
python3 scripts/analyze_ssl_log.py survey "$LOG"
```

出力される5セクション（TEAMS / REFEREE TRANSITIONS / BALL STATISTICS / ROBOT COUNT / GAME EVENTS SUMMARY）を確認し、異常箇所・注目時間帯を特定する。

**重要**: TEAMS セクションでチーム名と色を確認し、以降の分析レポートでは必ず「Yellow=○○チーム」「Blue=○○チーム」を明記すること。

### Step 3: 深掘り分析

Step 2の結果とユーザーの質問に合わせて追加調査を行う。

| 調査目的 | サブコマンド | 主要オプション |
|---------|------------|--------------|
| ロボット位置・速度の時系列 | `track` | `--robot <id>` `--team blue\|yellow` `--interval <秒>` `--time <s>:<e>` |
| ボール位置・速度の時系列 | `track` | `--ball` `--interval <秒>` `--time <s>:<e>` |
| referee状態遷移 | `referee` | `--changes-only` |
| ゴール・キック・ファウル等 | `events` | `--type goal kick foul card placement` |

代表的なコマンド例:

```bash
# ボール追跡（0.5秒間隔・100〜200秒範囲）
python3 scripts/analyze_ssl_log.py track "$LOG" --ball --interval 0.5 --time 100.0:200.0

# Yellow チームのロボット3番追跡（1秒間隔）
python3 scripts/analyze_ssl_log.py track "$LOG" --robot 3 --team yellow --interval 1.0

# referee遷移のみ表示
python3 scripts/analyze_ssl_log.py referee "$LOG" --changes-only

# 全イベント表示
python3 scripts/analyze_ssl_log.py events "$LOG"

# ゴールとファウルのみ
python3 scripts/analyze_ssl_log.py events "$LOG" --type goal foul

# JSON + jq 連携（ゴール時刻抽出）
python3 scripts/analyze_ssl_log.py events "$LOG" --type goal --format json | jq -r '.[].t'

# ファウルの集計
python3 scripts/analyze_ssl_log.py events "$LOG" --type foul --format json \
  | jq '[.[] | {type: .description}] | group_by(.type) | map({type: .[0].type, count: length})'

# referee遷移のJSON取得（特定コマンドの時刻抽出）
python3 scripts/analyze_ssl_log.py referee "$LOG" --changes-only --format json \
  | jq '.[] | select(.command == "NORMAL_START") | .t'

# ボール追跡のJSON（停止フレーム抽出）
python3 scripts/analyze_ssl_log.py track "$LOG" --ball --format json \
  | jq '.[] | select(.speed < 0.05)'
```
