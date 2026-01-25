# crane_session_coordinator

## 概要

**crane_session_coordinator**パッケージは、Crane SSLロボットシステムの**最上位制御レイヤー**として、包括的な試合管理とゲーム状態制御を提供します。セッションベースのロボット役割管理システムを実装し、動的タクティックプラグイン管理とYAML駆動設定を通じてSSL試合中の複数ロボットを協調制御します。柔軟で設定駆動なロボット役割管理により、SSL試合での洗練されたマルチロボット協調を実現します。

## 主要機能

- **試合統括制御**: SSL試合全体のフロー管理
- **タクティックプラグイン管理**: 状況に応じた戦略タクティックの選択・切り替え
- **ゲーム状態管理**: Referee信号に基づく状態遷移制御
- **コンテキスト管理**: 試合状況の履歴・コンテキスト保持
- **YAML設定駆動**: 柔軟な状況対応設定

## アーキテクチャ上の役割

Craneシステムの**最上位制御層**として、SSL Refereeからの指示を解釈し、適切な戦略タクティックを選択・実行することで、試合全体を統括します。タクティックプラグインアーキテクチャにより、様々な戦術を状況に応じて動的に切り替えます。

## コンポーネント構成

### SessionCoordinator（メイン制御）

- **状態管理**: SSL Referee状態の追跡と対応
- **タクティック選択**: 状況に応じた最適タクティックの選択
- **実行制御**: 選択されたタクティックの実行管理
- **エラーハンドリング**: 異常状況での安全な動作

## 統一設定ファイル（unified_session_config.yaml）

**設定ファイル**: `crane_session_coordinator/config/unified_session_config.yaml`
**ROSパラメータ**: `session_config_file_name` (デフォルト: "unified_session_config.yaml")

すべての試合状況設定が単一ファイルに統合されています。
**イベント（events）** が **状況（situations）** にマッピングされ、各状況は **セッション（sessions）** のリストとして定義されます。

### 設定構造

```yaml
situations:
  HALT:
    description: 試合停止
    sessions:
      - name: waiter
        max_robots: 20
  INPLAY:
    description: 通常プレイ
    sessions:
      - name: attacker_skill
        max_robots: 1
      - name: total_defense
        max_robots: 3
      # ...
  OUR_FREE_KICK:
    description: 自チームフリーキック
    sessions:
      - name: attacker_skill
        max_robots: 1
      - name: defender
        max_robots: 2
      # ...

events:
  - name: HALT
    situation: HALT
  - name: STOP
    situation: STOP
  - name: INPLAY
    situation: INPLAY
  - name: OUR_DIRECT_FREE
    situation: OUR_FREE_KICK
```

### 主要な構成要素

- **situations**: ロボットの役割分担（セッション構成）を定義
- **sessions**: 特定のタクティック（`crane_sessions`）と最大割り当て台数（`max_robots`）のペア
- **events**: `PlaySituation` のイベント名と `situations` の対応付け

## 動作フロー

### 状況判定とロボット割当

1. **イベント受信**: `/play_situation` トピックから現在のイベント名（例: "STOP", "INPLAY"）を受信
2. **状況解決**: 設定ファイルの `events` マッピングを使用して、イベント名から状況名（Situation Name）を特定
3. **セッション取得**: 特定された状況に対応するセッションリストを取得
4. **ロボット割当**: `RobotAllocator` が利用可能なロボットを各セッション（タクティック）に割り当て
   - 優先度や役割適性（`game_analysis` 推奨など）を考慮して最適化
5. **コマンド生成**: 各タクティックが割り当てられたロボットに対して動作コマンドを生成

### コンポーネント構成

- **ConfigurationManager**: YAML設定ファイルの読み込みとイベント・状況マッピングの管理
- **RobotAllocator**: 定義されたセッションへのロボット動的割り当てロジック
- **SessionRegistry**: 利用可能なタクティックの管理
- **CommandAggregator**: 各タクティックから生成されたコマンドの集約

## 依存関係

### コア依存

- **crane_sessions**: 実際の戦略タクティック群
- **crane_msg_wrappers**: メッセージ変換・統合
- **crane_msgs**: システムメッセージ定義

### 機能依存

- **yaml-cpp**: YAML設定ファイル処理
- **diagnostic_updater**: システム診断
- **closest_point_vendor**: 幾何学計算支援

## 使用方法

### 基本起動

```bash
# セッションコントローラー起動
ros2 run crane_session_coordinator crane_session_coordinator_node

# システム全体起動（含む）
ros2 launch crane_bringup crane.launch.xml
```

### カスタム設定

```yaml
# config/custom_situation.yaml
situation: "CUSTOM_DEFENSE"
robots:
  - id: 0
    role: "goalie"
    tactic: "GoalieTactic"
  - id: [1,2,3]
    role: "defender"
    tactic: "DefenseTactic"
```

## 診断機能

このパッケージはAI計画サイクルの健全性を監視する診断情報を提供します：

### 提供する診断項目

- **AI計画サイクル状態** (`ai_tactic/planning_cycle`)
  - WorldModelの準備状態
  - 計画サイクルの更新頻度

### 診断レベル

- **ERROR**: 1秒以上計画サイクル未実行
- **WARN**: WorldModel未準備、または500ms以上更新遅延
- **OK**: 正常動作中（更新時間も報告）

詳細は[診断システムドキュメント](../diagnostics.md#crane_session_coordinator)を参照してください。

## 最近の開発状況

### 2025年の主要変更

- **セッション自動復帰**: 途中再起動時のロール再割当フローを整備
- **プレイバンドル化**: キック系シナリオのテンプレート化で設定ファイルを統一
- **状況判定精度向上**: Referee+Analyzer複合判定の閾値再調整
- **パフォーマンス最適化**: タクティック切り替え時のデータ同期をノンブロッキング化

### 開発活発度

🟡 **中活動**: 試合規則変更や新フォーメーション適用に伴う設定更新が継続中。特にテンプレート化されたYAMLシナリオとタクティック連携の改修が活発。

### 技術的特徴

- **設定駆動アーキテクチャ**: コード変更なしでの戦術調整
- **プラグイン統合**: 動的なタクティック選択・切り替え
- **状況適応**: SSL規則変更への柔軟な対応

## パフォーマンス特性

### 応答性能

- **状況判定時間**: <5ms
- **タクティック切り替え**: <10ms
- **全体制御周期**: 60Hz対応

### 管理容量

- **同時管理ロボット**: 最大11台
- **設定ファイル**: 25種類の状況対応
- **タクティック種類**: 20種類以上

## 将来展望

### 技術発展

- **AI統合**: 機械学習による状況判定の高度化
- **適応制御**: 対戦相手に応じた動的戦術調整
- **予測制御**: より長期的な戦略計画

---

**関連パッケージ**: [crane_sessions](./crane_sessions.md) | [crane_play_switcher](./crane_play_switcher.md) | [crane_robot_skills](./crane_robot_skills.md)
