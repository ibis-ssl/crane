---
name: analyze-ssl-log
description: "SSL公式ログ(.log.gz)を解析して試合状況・referee・ロボット動作を診断"
allowed-tools: ["Bash", "Read", "Glob", "Grep", "Agent"]
---

# SSL公式ログ解析

SSL公式ログ（`.log.gz`）を解析します。

コマンドプレフィックス（以降 `ssl_log` と略記）:

```bash
# craneリポジトリのルートで実行
# ssl_log <subcommand> ... は以下の意味:
# python3 scripts/analyze_ssl_log.py <subcommand> ...
```

## 引数

```text
$ARGUMENTS
```

- `<log_path>` — 解析する `.log.gz` ファイルのパス（必須）
- 追加テキスト — 分析の焦点（例: "ファウルが多発した時間帯", "ロボット3番の動き"）

## 実行手順

### Step 0: ファイル確認

```bash
LOG="<log_path>"
ls -lh "$LOG"
```

初回実行時のみ `scripts/ssl_proto/` に protobuf バインディングが自動生成される（`grpcio-tools` または `protoc` が必要）。

### Step 1: ログ基本情報

```bash
python3 scripts/analyze_ssl_log.py info "$LOG"
```

時間範囲、チーム名、最終スコア、メッセージ数（Referee/Vision/Tracker）を確認する。

### Step 2: 試合概要サーベイ

```bash
python3 scripts/analyze_ssl_log.py survey "$LOG"
```

TEAMS / REFEREE TRANSITIONS / BALL STATISTICS / ROBOT COUNT / GAME EVENTS SUMMARY を確認し、異常箇所・注目時間帯を特定する。

**重要**: TEAMS セクションでチーム名と色を確認し、以降の分析レポートでは必ず「Yellow=○○チーム」「Blue=○○チーム」を明記すること。

### Step 3: 深掘り分析

Step 2の結果とユーザーの質問に合わせて追加調査を行う。

| 調査目的 | サブコマンド | 主要オプション |
|---------|------------|--------------|
| ロボット・ボールの時系列 | `track` | `--robot <id>` / `--ball` / `--team blue\|yellow` / `--interval <秒>` / `--time <s>:<e>` |
| referee状態遷移 | `referee` | `--changes-only` |
| ゴール・キック・ファウル等 | `events` | `--type goal kick foul card placement` |

代表的なコマンド例:

```bash
# ボール追跡（0.5秒間隔・100〜200秒範囲）
python3 scripts/analyze_ssl_log.py track "$LOG" --ball --interval 0.5 --time 100.0:200.0

# Yellow チームのロボット3番追跡
python3 scripts/analyze_ssl_log.py track "$LOG" --robot 3 --team yellow --interval 1.0

# referee遷移のみ表示
python3 scripts/analyze_ssl_log.py referee "$LOG" --changes-only

# ゴールとファウルイベント
python3 scripts/analyze_ssl_log.py events "$LOG" --type goal foul

# JSON + jq 連携（ゴール時刻抽出 / ファウル集計）
python3 scripts/analyze_ssl_log.py events "$LOG" --type goal --format json | jq -r '.[].t'
python3 scripts/analyze_ssl_log.py events "$LOG" --type foul --format json \
  | jq '[.[] | {type: .description}] | group_by(.type) | map({type: .[0].type, count: length})'

# referee遷移のJSON（特定コマンドの時刻抽出）
python3 scripts/analyze_ssl_log.py referee "$LOG" --changes-only --format json \
  | jq '.[] | select(.command == "NORMAL_START") | .t'
```
