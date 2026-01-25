# crane_play_switcher

## 概要

**crane_play_switcher**は、SSL-Referee（審判）からのコマンドを解釈し、Craneシステム内部で使用する**プレイ状況（PlaySituation）**へ変換・配信するROS 2ノードです。レフェリーコマンドの単純なマッピングだけでなく、ルール（ボール移動によるインプレイ開始など）に基づいた状況遷移判定も行います。

## 主要機能

- **レフェリーコマンド解釈**: SSL-Refereeプロトコルの受信と解釈
- **状況遷移管理**: ルールに基づくインプレイ/ストップ等の状態遷移判定
- **セッション注入**: デバッグ用の強制イベント注入機能
- **状況配信**: `/play_situation` トピックによるシステム全体への状況通知

## アーキテクチャ上の役割

Craneシステムの**ゲーム状態管理層**として、外部からの審判指示をシステム内部の共通言語（PlaySituation）に翻訳し、`crane_session_coordinator` などの上位層が適切な戦術を決定するための基礎情報を提供します。

## 処理ロジック

### コマンドマッピング

SSL-Refereeのコマンド（`STOP`, `FORCE_START`など）をCrane内部コマンドに変換します。

- `NORMAL_START` -> `KICKOFF_START` / `PENALTY_START` (直前のPREPARATION状態による)
- `STOP` -> 次のコマンド（`NEXT_COMMAND`）を考慮した詳細なSTOP状態へ

### インプレイ判定（自動遷移）

以下の条件で `INPLAY` 状態へ自動遷移します：

- ボールが0.05m以上移動した（キックオフ・フリーキック時）
- キックオフから10秒経過
- フリーキックからN秒経過（DivA: 5秒, DivB: 10秒）

## 使用方法

### 起動（crane_bringup経由）

`crane.launch.xml` に含まれており、自動的に起動します。

```bash
ros2 launch crane_bringup crane.launch.xml
```

### パラメータ

- `team_name`: チーム名（デフォルト: "ibis"）- 自分のチームカラー判定に使用

### 入出力トピック

- **Sub**: `/referee` (robocup_ssl_msgs/Referee)
- **Sub**: `/session_injection` (std_msgs/String) - デバッグ用
- **Pub**: `/play_situation` (crane_msgs/PlaySituation)

## 最近の開発状況

🟡 **中活動**: 戦術切り替えロジックの改良と新しい状況判定機能の追加が継続的に行われています。

---

**関連パッケージ**: [crane_game_analyzer](./crane_game_analyzer.md) | [crane_session_coordinator](./crane_session_coordinator.md)
