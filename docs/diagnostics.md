# Crane Diagnostics System（診断システム）

## 目次

- [概要](#概要)
- [アーキテクチャ](#アーキテクチャ)
- [診断階層構造](#診断階層構造)
- [コンポーネント別診断詳細](#コンポーネント別診断詳細)
- [利用可否判定の多層設計](#利用可否判定の多層設計)
- [エラーコードリファレンス](#エラーコードリファレンス)
- [診断情報の活用](#診断情報の活用)
- [設定ファイル](#設定ファイル)
- [診断トピック](#診断トピック)
- [開発ガイド](#開発ガイド)
- [トラブルシューティング](#トラブルシューティング)
- [参考資料](#参考資料)

---

## 概要

CraneシステムはROS 2の標準diagnostics機能を活用した包括的な診断・監視システムを実装しています。このシステムにより、システム全体の健全性をリアルタイムで監視し、問題を早期に検出できます。

### 主な特徴

- **ROS 2 diagnosticsの活用**: `diagnostic_updater`と`diagnostic_aggregator`を使用した標準準拠の診断システム
- **システム全体の監視**: Vision処理、AI計画、経路計画、ロボット通信・バッテリー・エラー状態を包括的に監視
- **階層的な情報管理**: Base Station（計算PC）とTeam（ロボット群）の2大分類による体系的な診断情報の管理
- **リアルタイムエラー検出**: 通信タイムアウト、バッテリー低下、ハードウェアエラーの即座の検出
- **可視化統合**: Foxgloveでのロボット位置へのエラーメッセージ表示
- **ロボット利用可否判定**: 診断情報を`robot->available`フラグに統合し、故障ロボットの自動除外

### 診断レベル

診断システムは以下の4段階のレベルで状態を報告します：

| レベル | 値 | 意味 | 色（可視化） |
|--------|-----|------|--------------|
| **OK** | 0 | 正常動作 | 白 |
| **WARN** | 1 | 警告状態（動作可能だが注意が必要） | 黄色 |
| **ERROR** | 2 | エラー状態（動作不可または重大な問題） | 赤 |
| **STALE** | 3 | 診断情報が古い（タイムアウト） | グレー |

---

## アーキテクチャ

Crane診断システムは以下の3層構造で構成されています：

### 層1: データ収集層（各コンポーネント）

各主要コンポーネントが独自の診断情報を生成・配信します。

```text
┌──────────────────────────────────────────────────────────┐
│           データ収集層（Diagnostic Publishers）           │
├──────────────────────────────────────────────────────────┤
│  ┌────────────────────┐  ┌───────────────────────┐      │
│  │ DiagnosticPublisher│  │ WorldModelPublisher   │      │
│  │ (robot_receiver)   │  │ - vision/processing   │      │
│  │ - communication    │  └───────────────────────┘      │
│  │ - battery          │                                  │
│  │ - robot_error      │  ┌───────────────────────┐      │
│  └────────────────────┘  │ TacticCoordinator     │      │
│                          │ - ai_planner/planning │      │
│  ┌────────────────────┐  └───────────────────────┘      │
│  │ LocalTactic       │                                  │
│  │ - path_planning    │  ┌───────────────────────┐      │
│  └────────────────────┘  │ DiagnosedPublisher    │      │
│                          │ (トピック頻度監視)     │      │
│                          └───────────────────────┘      │
└──────────────────────────────────────────────────────────┘
                            ↓ /diagnostics
```

**使用技術**: ROS 2 `diagnostic_updater`パッケージ

### 層2: 統合層（diagnostic_aggregator）

`/diagnostics`トピックの情報を階層化して集約します。

```text
┌──────────────────────────────────────────────────────────┐
│            統合層（diagnostic_aggregator）                │
├──────────────────────────────────────────────────────────┤
│                                                            │
│  SSL_System/                                               │
│  ├── Base_Station/                                         │
│  │   ├── Vision/       (vision/*)                          │
│  │   ├── AI_Planner/   (ai_planner/*)                      │
│  │   └── Local_Planner/ (local_planner/*)                  │
│  └── Team/                                                  │
│      ├── Robot_00/      (robot_00/*)                        │
│      ├── Robot_01/      (robot_01/*)                        │
│      ├── ...                                                │
│      └── Robot_12/      (robot_12/*)                        │
│                                                            │
└──────────────────────────────────────────────────────────┘
        ↓ /diagnostics_agg, /diagnostics_toplevel_state
```

**設定ファイル**: `crane_bringup/config/diagnostic_aggregator.yaml`

### 層3: 統合活用層（WorldModelWrapper）

診断情報をロボット利用可否判定に反映します。**循環参照を回避するため、入力ソース別にavailableを分離**しています。

```text
┌──────────────────────────────────────────────────────────┐
│         統合活用層（WorldModelWrapper）                   │
├──────────────────────────────────────────────────────────┤
│  【循環参照回避設計】                                      │
│                                                            │
│  入力ソース別にavailableを分離:                            │
│  ┌─────────────────────────────────────────┐            │
│  │ available_vision    (診断結果に依存しない) │            │
│  │ available_feedback  (診断結果に依存しない) │            │
│  │ available_hardware  (診断結果を反映)       │            │
│  └─────────────────────────────────────────┘            │
│                                                            │
│  /diagnostics_agg購読                                      │
│    ↓                                                       │
│  ERRORレベル診断検出 → available_hardware更新              │
│                                                            │
│  総合判定: available() = available_vision && available_hardware │
│                                                            │
│  ※診断の入力はavailable_vision等のみ参照 → 循環なし       │
└──────────────────────────────────────────────────────────┘
```

**コード**: `crane_msg_wrappers/world_model_wrapper.cpp`, `crane_physics/robot_info.hpp`

---

## 診断階層構造

診断情報は以下の階層構造で管理されます（`diagnostic_aggregator.yaml`で定義）：

```text
SSL_System/
├── Base_Station/           # 計算PC上のコンポーネント
│   ├── Vision/            # vision/* の診断
│   │   └── vision/processing
│   ├── AI_Planner/        # ai_planner/* の診断
│   │   └── ai_planner/planning_cycle
│   └── Local_Planner/     # local_planner/* の診断
│       └── local_planner/path_planning
└── Team/                   # ロボット群
    ├── Robot_00/          # robot_00/* の診断
    │   ├── robot_00/communication
    │   ├── robot_00/battery
    │   └── robot_00/robot_error
    ├── Robot_01/          # robot_01/* の診断
    ├── ...
    └── Robot_12/          # robot_12/* の診断
```

### タイムアウト設定

すべてのアナライザーに対して**5.0秒**のタイムアウトが設定されています。診断情報が5秒以上更新されない場合、状態は`STALE`となります。

---

## コンポーネント別診断詳細

### crane_robot_receiver（DiagnosticPublisher）

最も包括的な診断機能を提供する中核コンポーネントです。各ロボット（ID 0-12）について3種類の診断情報を提供します。

#### 診断名の命名規則

```text
robot_{:02d}/communication    # ロボット00-12の通信状態
robot_{:02d}/battery          # ロボット00-12のバッテリー状態
robot_{:02d}/robot_error      # ロボット00-12のロボットエラー
```

**注**: ロボット番号は必ず2桁形式（例: `robot_00`, `robot_01`）で表記されます。

#### 1. 通信状態監視（communication）

ロボットとの通信品質を監視します。

**監視項目**: Ping応答時間

**判定基準**:

| 条件 | レベル | メッセージ |
|------|--------|------------|
| Ping時間 > 50ms | ERROR | "Communication is too slow: {ping_time} ms" |
| Ping時間 > 10ms | WARN | "Communication is slow: {ping_time} ms" |
| Ping時間 ≤ 10ms | OK | "Communication is OK: {ping_time} ms" |
| Pingデータなし | WARN | "No ping data available" |

**KeyValue情報**:

- `ping_time`: Ping応答時間（ミリ秒）

**シミュレータモード対応**:

`sim_mode`パラメータが`true`の場合、Pingデータがなくても`OK`として扱います（シミュレータではPing機能が使用されないため）。

**実装ファイル**: `crane_robot_receiver/src/diagnostic_publisher.cpp:communicationDiagnosticCallback`

#### 2. バッテリー監視（battery）

ロボットのバッテリー電圧を監視します。

**監視項目**: バッテリー電圧

**判定基準**:

| 条件 | レベル | メッセージ |
|------|--------|------------|
| 電圧 < 22.0V | ERROR | "Battery voltage is too low: {voltage} V" |
| 電圧 < 23.0V | WARN | "Battery voltage is low: {voltage} V" |
| 電圧 ≥ 23.0V | OK | "Battery voltage is OK: {voltage} V" |
| バッテリーデータなし | WARN | "No battery data available" |

**KeyValue情報**:

- `voltage`: バッテリー電圧（ボルト）

**実装ファイル**: `crane_robot_receiver/src/diagnostic_publisher.cpp:batteryDiagnosticCallback`

#### 3. ロボットエラー監視（robot_error）

RobotFeedbackメッセージから報告されるハードウェアエラーを監視します。

**監視対象**:

- **POWERエラー**: 電源系統のエラー（14種類）
- **BLDCモーターエラー**: 4つの駆動モーター（RF, RB, LB, LF）のエラー（各7種類）

**判定基準**:

| 条件 | レベル | メッセージ |
|------|--------|------------|
| エラーあり (`has_error == true`) | ERROR | "{エラー種別}: {エラー詳細} (継続時間: {duration}s, 値: {value})" |
| エラーなし | OK | "No robot error detected" |
| フィードバックなし | WARN | "No feedback data available" |

**エラー種別の判定**:

- `error_id == 100`: POWERエラー
- `error_id == 0-3`: BLDCモーターエラー（0=RF, 1=RB, 2=LB, 3=LF）

**KeyValue情報**:

- `error_id`: エラー発生箇所ID
- `error_info`: エラー種別コード
- `error_value`: エラー値
- `error_duration`: エラー継続時間（秒）

詳細なエラーコードは[エラーコードリファレンス](#エラーコードリファレンス)を参照してください。

**実装ファイル**: `crane_robot_receiver/src/diagnostic_publisher.cpp:robotErrorDiagnosticCallback`

#### エラーマップ管理

DiagnosticPublisherは各ロボットについて**エラーマップ**（`std::map<std::string, ErrorInfo>`）を維持し、複数のエラーを同時に追跡します。

```cpp
struct ErrorInfo {
  std::string type;        // エラータイプ (robot_error, communication, battery)
  std::string message;     // エラーメッセージ
  int level;               // エラーレベル (0: OK, 1: WARN, 2: ERROR, 3: STALE)
  rclcpp::Time timestamp;  // エラーが発生した時刻
};
```

**エラータイムアウト処理**:

10秒以上経過したエラーはマップから削除され、可視化からも消えます。これにより、一時的なエラーが解消された後の状態を正確に反映できます。

#### 可視化機能

DiagnosticPublisherは診断情報をFoxgloveで可視化するための機能を提供します：

- **エラーメッセージの表示**: ロボットの位置にエラーメッセージをテキストとして表示
- **色分け**: エラーレベルに応じて色を変更（WARN=黄色、ERROR=赤、STALE=グレー）
- **タイムスタンプ管理**: 古いエラーは自動的に非表示

**実装ファイル**: `crane_robot_receiver/src/diagnostic_publisher.cpp:visualizeRobotErrors`

#### シミュレータモード

`sim_mode`パラメータ（デフォルト: `false`）により、シミュレータと実機で動作を切り替えます：

- **実機モード** (`sim_mode: false`):
  - Pingデータ必須
  - バッテリーデータ必須
  - すべての診断を厳密に実行

- **シミュレータモード** (`sim_mode: true`):
  - Pingデータなしでも`OK`
  - その他の診断は通常通り実行

**パラメータ設定**: `crane.launch.xml`

---

### crane_local_planner

経路計画システムの健全性を監視します。

#### 診断名

```text
local_planner/path_planning
```

#### 監視項目

LocalPlannerの初期化状態とWorldModelの取得状態を監視します。

**判定基準**:

| 条件 | レベル | メッセージ |
|------|--------|------------|
| プランナー未初期化 | ERROR | "Planner is not initialized" |
| WorldModel未取得 | WARN | "WorldModel is not available" |
| WorldModel未更新（1秒以上） | WARN | "WorldModel is stale" |
| 正常動作 | OK | "Path planning is working normally" |

**実装ファイル**: `crane_local_planner/src/local_planner.cpp`

---

### crane_world_model_publisher

Vision処理の状態を監視します。

#### 診断名

```text
vision/processing
```

#### 監視項目

Vision GCからのデータ受信状態を監視します。

**判定基準**:

| 条件 | レベル | メッセージ |
|------|--------|------------|
| Visionデータなし（1秒以上） | ERROR | "No vision data received for over 1 second" |
| 正常処理中 | OK | "Vision processing is running" |

**実装ファイル**: `crane_world_model_publisher/src/world_model_publisher.cpp`

---

### crane_session_coordinator

AI計画サイクルの監視を行います。

#### 診断名

```text
ai_planner/planning_cycle
```

#### 監視項目

AI計画の実行サイクルとWorldModelの更新状態を監視します。

**判定基準**:

| 条件 | レベル | メッセージ |
|------|--------|------------|
| WorldModel未準備 | WARN | "WorldModel is not ready" |
| 更新なし（1秒以上） | ERROR | "Planning cycle has not run for over 1 second" |
| 更新遅延（500ms以上） | WARN | "Planning cycle is slow (last update: {time} ms ago)" |
| 正常動作 | OK | "Planning cycle is running normally (last update: {time} ms ago)" |

**実装ファイル**: `crane_session_coordinator/src/crane_session_coordinator.cpp`

---

### DiagnosedPublisher（crane_comm）

トピックの配信頻度を自動監視するテンプレートクラスです。

#### 概要

`DiagnosedPublisher`は通常のROS 2 Publisherをラップし、トピックの配信頻度を自動的に診断します。

#### 使用箇所

以下のコンポーネントで使用されています：

| コンポーネント | トピック | 最小頻度 | 最大頻度 |
|----------------|----------|----------|----------|
| WorldModelPublisher | `/world_model` | 50 Hz | 70 Hz |
| TacticCoordinator | `/robot_commands` | 50 Hz | 70 Hz |
| LocalTactic | `/robot_commands` | 50 Hz | 70 Hz |
| SimSender | grSimコマンド | 50 Hz | 70 Hz |

#### 動作原理

```cpp
template <typename MessageT>
class DiagnosedPublisher {
  // トピック配信と同時に診断情報を更新
  void publish(const MessageT & message) {
    topic_diagnostic.tick(clock->now());  // 配信頻度を記録
    publisher->publish(message);
  }
};
```

`diagnostic_updater::TopicDiagnostic`が内部で配信頻度を計測し、設定された範囲（min/max Hz）外の場合に自動的にWARNまたはERRORを報告します。

**実装ファイル**: `utility/crane_comm/include/crane_comm/diagnosed_publisher.hpp`

---

## 利用可否判定の多層設計

### 循環参照回避のための設計思想

診断結果を利用可否判定に反映する際、**診断の入力として利用可否を参照すると循環参照が発生**します。これを回避するため、入力ソース別にavailableを分離しています。

### RobotInfo（C++構造体）の多層available

```cpp
// crane_physics/robot_info.hpp
struct RobotInfo {
  // 入力ソース別の基礎データ（相互に独立）
  bool available_vision = false;     // ビジョン検出（診断結果に依存しない）
  bool available_hardware = false;   // ハードウェア診断結果
  bool available_feedback = false;   // フィードバック検出

  // 判定関数（用途に応じて使い分け）
  auto available() const -> bool {
    return available_vision && available_hardware;  // 基本判定
  }
  auto availableStrict() const -> bool {
    return available_vision && available_hardware && available_feedback;
  }
  auto availableLoose() const -> bool {
    return available_vision;
  }
};
```

### RobotInfoメッセージフィールド

```msg
# crane_msgs/msg/RobotInfo.msg
# 入力ソース別の検出状態（循環参照回避のため個別に保持）
bool available_vision      # ビジョンで検出されているか
bool available_feedback    # フィードバックで検出されているか
bool available_tracker     # 内部トラッカーで検出されているか

# エラー情報（診断システムへの入力）
bool has_error
uint16 error_id
uint16 error_info
```

### 統合処理とデータフロー

```text
[ビジョン/フィードバック] → available_vision, available_feedback
                            ↓（診断の入力）
                       Diagnostic Publisher
                            ↓
                     /diagnostics_agg
                            ↓（診断の出力）
                     available_hardware
                            ↓
            available() = available_vision && available_hardware
```

**重要**: 診断は`available_vision`等のみを参照し、診断結果の`available_hardware`には依存しないため、循環参照を完全に回避できます。

### エラーレベルの扱い

- **OK/WARN**: `available_hardware = true`（動作可能）
- **ERROR/STALE**: `available_hardware = false`（動作不可）

**実装ファイル**: `crane_msg_wrappers/src/world_model_wrapper.cpp`, `crane_physics/robot_info.hpp`

---

## エラーコードリファレンス

### エラー種別

ロボットのハードウェアエラーは以下の2種類に分類されます：

- **POWERエラー** (`error_id == 100`): 電源系統のエラー（電圧不足、過電圧、過電流、キャパシタ異常、過熱など14種類）
- **BLDCモーターエラー** (`error_id == 0-3`): 4つの駆動モーター（RF, RB, LB, LF）のエラー（電圧異常、過電流、過熱、エンコーダエラーなど7種類）

### エラーコード詳細

詳細なエラーコード一覧とビットフラグ定義は以下を参照：

- **定義**: `crane_robot_receiver/include/crane_robot_receiver/robot_errors.hpp`
- **変換関数**: `getPowerErrorString()`, `getBLDCErrorString()`

エラーメッセージ例: `POWER : UNDER_VOLTAGE`, `BLDC-RF : MOTOR_OVER_HEAT`

---

## 診断情報の活用

### ロボット利用可否判定

診断結果は`available()`関数で総合判定され、故障ロボットが自動的にAI計画から除外されます。

```cpp
// 基本判定: vision && hardware
info->available()        // プランナーで使用
info->availableStrict()  // 重要操作で使用
info->availableLoose()   // 緊急時/表示用
```

### 可視化

- **Foxglove**: ロボット位置にエラーメッセージを色分け表示（10秒タイムアウト）
- **rqt_robot_monitor**: `ros2 run rqt_robot_monitor rqt_robot_monitor` で階層的な診断情報を確認

---

## 設定ファイル

### diagnostic_aggregator.yaml

診断情報の階層構造を定義する設定ファイルです。

**ファイルパス**: `crane_bringup/config/diagnostic_aggregator.yaml`

#### 基本構造

```yaml
analyzers:
  ros__parameters:
    path: 'SSL_System'  # ルートパス

    # Base Station（計算PC）の診断
    base_station:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Base_Station
      analyzers:
        vision:
          type: diagnostic_aggregator/GenericAnalyzer
          path: Vision
          contains: ['vision/']
          timeout: 5.0

        ai_planner:
          type: diagnostic_aggregator/GenericAnalyzer
          path: AI_Planner
          contains: ['ai_planner/']
          timeout: 5.0

        local_planner:
          type: diagnostic_aggregator/GenericAnalyzer
          path: Local_Planner
          contains: ['local_planner/']
          timeout: 5.0

    # Team（ロボット群）の診断
    team:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Team
      analyzers:
        robot_00:
          type: diagnostic_aggregator/GenericAnalyzer
          path: Robot_00
          contains: ['robot_00/']
          timeout: 5.0
        # robot_01 から robot_12 まで同様
```

#### アナライザータイプ

- **AnalyzerGroup**: 子アナライザーをグループ化
- **GenericAnalyzer**: パターンマッチングで診断を集約

#### パラメータ

- **path**: 階層パス
- **contains**: マッチングパターン（リスト）
- **timeout**: タイムアウト時間（秒）

### crane.launch.xml統合

診断ノードの起動設定です。

**ファイルパス**: `crane_bringup/launch/crane.launch.xml`

#### diagnostic_publisher_node起動

```xml
<node pkg="crane_robot_receiver" exec="diagnostic_publisher_node" name="diagnostic_publisher">
  <param name="sim_mode" value="$(var sim_mode)" />
</node>
```

**パラメータ**:

- `sim_mode`: シミュレータモードフラグ（デフォルト: `false`）

#### diagnostic_aggregator起動

```xml
<node pkg="diagnostic_aggregator" exec="aggregator_node" name="diagnostic_aggregator">
  <param from="$(find-pkg-share crane_bringup)/config/diagnostic_aggregator.yaml" />
</node>
```

---

## 診断トピック

### 標準トピック

ROS 2 diagnosticsが提供する標準トピックです。

#### /diagnostics

**型**: `diagnostic_msgs/msg/DiagnosticArray`

**説明**: すべてのコンポーネントから配信される生の診断情報

**配信者**:

- DiagnosticPublisher（各ロボット×3診断）
- WorldModelPublisher
- TacticCoordinator
- LocalTactic
- DiagnosedPublisher（各トピック）

**サンプル確認**:

```bash
ros2 topic echo /diagnostics
```

#### /diagnostics_agg

**型**: `diagnostic_msgs/msg/DiagnosticArray`

**説明**: `diagnostic_aggregator`によって階層化・集約された診断情報

**配信者**: diagnostic_aggregator

**サンプル確認**:

```bash
ros2 topic echo /diagnostics_agg
```

#### /diagnostics_toplevel_state

**型**: `diagnostic_msgs/msg/DiagnosticStatus`

**説明**: システム全体の最上位レベルの診断状態

**配信者**: diagnostic_aggregator

**サンプル確認**:

```bash
ros2 topic echo /diagnostics_toplevel_state
```

### ロボット別トピック（後方互換）

各ロボット専用の診断トピックも提供されています。

#### /diagnostics/robot_{:02d}

**型**: `diagnostic_msgs/msg/DiagnosticArray`

**説明**: 特定ロボットの診断情報（後方互換のため提供）

**配信者**: DiagnosticPublisher

**例**:

```bash
ros2 topic echo /diagnostics/robot_00
ros2 topic echo /diagnostics/robot_05
```

---

## 開発ガイド

### 新規診断項目の追加方法

新しいコンポーネントに診断機能を追加する手順です。

#### 1. diagnostic_updaterの使用

**ヘッダーインクルード**:

```cpp
#include <diagnostic_updater/diagnostic_updater.hpp>
```

**メンバ変数追加**:

```cpp
class MyNode : public rclcpp::Node {
private:
  diagnostic_updater::Updater diagnostics_updater_;
};
```

**コンストラクタで初期化**:

```cpp
MyNode::MyNode() : Node("my_node"), diagnostics_updater_(this) {
  // 診断タスクを追加
  diagnostics_updater_.add(
    "my_component/status",  // 診断名
    this,
    &MyNode::diagnosticCallback  // コールバック関数
  );

  // 更新頻度を設定（オプション）
  diagnostics_updater_.setHardwareID("my_hardware");
}
```

**診断コールバック実装**:

```cpp
void MyNode::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat) {
  if (is_error_condition) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "エラーが発生しました");
    stat.add("error_code", error_code);
    stat.add("detail", error_detail);
  } else if (is_warn_condition) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "警告状態です");
    stat.add("warning_info", warn_info);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "正常動作中");
    stat.add("status", "running");
  }
}
```

**定期的に更新**:

```cpp
void MyNode::timerCallback() {
  // 処理...

  // 診断情報を更新
  diagnostics_updater_.force_update();
}
```

#### 2. 診断名の命名規則

診断名は以下の規則に従ってください：

- **カテゴリ/項目名**: `category/item_name`形式
- **ロボット関連**: `robot_{:02d}/category`（必ず2桁）
- **コンポーネント名を明確に**: 他と重複しない固有の名前

**良い例**:

- `vision/processing`
- `ai_planner/planning_cycle`
- `robot_00/communication`
- `my_component/initialization`

**悪い例**:

- `status`（曖昧すぎる）
- `robot_0/error`（1桁表記）
- `MyComponent`（階層なし）

#### 3. diagnostic_aggregator.yamlへの追加

新しい診断を階層に追加します。

**Base Station配下に追加する場合**:

```yaml
analyzers:
  ros__parameters:
    path: 'SSL_System'
    base_station:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Base_Station
      analyzers:
        # 既存のアナライザー...

        # 新規追加
        my_component:
          type: diagnostic_aggregator/GenericAnalyzer
          path: MyComponent
          contains: ['my_component/']
          timeout: 5.0
```

**Team（ロボット）配下に追加する場合**:

ロボット別の診断は自動的に`robot_{:02d}/`パターンでマッチングされるため、設定変更不要です。

#### 4. DiagnosedPublisherの使用

トピック配信頻度を監視する場合は、`DiagnosedPublisher`を使用します。

```cpp
#include <crane_comm/diagnosed_publisher.hpp>

// 通常のPublisherの代わりにDiagnosedPublisherを使用
crane::DiagnosedPublisher<MyMessageType> diagnosed_pub(
  this,                    // ノード
  "/my_topic",             // トピック名
  10,                      // QoS履歴深度
  50.0,                    // 最小更新頻度（Hz）
  70.0                     // 最大更新頻度（Hz）
);

// 配信（診断も自動的に更新される）
diagnosed_pub.publish(message);
```

### デバッグ方法

#### rqt_robot_monitorの使用

GUIで診断情報を確認します。

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

#### Foxgloveでの確認

Foxglove Studioで以下を表示：

1. **Diagnosticsパネル**: `/diagnostics_agg`を表示
2. **3Dパネル**: ロボット位置のエラーメッセージを確認

#### トピックの直接確認

ターミナルでトピックを監視します。

```bash
# 生の診断情報
ros2 topic echo /diagnostics

# 集約された診断情報
ros2 topic echo /diagnostics_agg

# 最上位レベルの状態
ros2 topic echo /diagnostics_toplevel_state

# 特定ロボットの診断
ros2 topic echo /diagnostics/robot_00
```

#### 診断情報のフィルタリング

特定の診断名のみを表示します。

```bash
ros2 topic echo /diagnostics | grep -A 10 "robot_00/battery"
```

---

## トラブルシューティング

### 主な問題と対処

| 症状 | 確認方法 | 対処法 |
|------|----------|--------|
| 診断情報が表示されない | `ros2 node list \| grep aggregator`<br>`ros2 topic echo /diagnostics` | aggregator起動確認<br>YAMLの`contains`パターン確認 |
| エラーが誤検出される | 閾値とsim_mode確認 | `diagnostic_publisher.cpp`の閾値調整<br>`sim_mode: true`（シミュレータ時） |
| STALEエラー頻発 | `ros2 topic hz /diagnostics` | `force_update()`呼び出し確認<br>タイムアウト延長 |
| ロボットが利用不可 | `ros2 topic echo /diagnostics/robot_XX` | ERRORレベル診断を確認<br>`available_vision/hardware/feedback`の各状態を確認 |

---

## 参考資料

### ROS 2公式ドキュメント

- **diagnostic_updater**: <https://github.com/ros/diagnostics/tree/ros2/diagnostic_updater>
- **diagnostic_aggregator**: <https://github.com/ros/diagnostics/tree/ros2/diagnostic_aggregator>
- **diagnostic_msgs**: <https://github.com/ros2/common_interfaces/tree/rolling/diagnostic_msgs>

### 関連パッケージドキュメント

- [crane_robot_receiver](./packages/crane_robot_receiver.md) - DiagnosticPublisher実装詳細
- [crane_local_planner](./packages/crane_local_planner.md) - LocalPlanner診断実装
- [crane_world_model_publisher](./packages/crane_world_model_publisher.md) - Vision診断実装
- [crane_session_coordinator](./packages/crane_session_coordinator.md) - AI計画診断実装
- [crane_comm](./packages/crane_comm.md) - DiagnosedPublisher実装詳細

### Crane内部実装ファイル

#### ヘッダーファイル

- `crane_robot_receiver/include/crane_robot_receiver/diagnostic_publisher.hpp` - DiagnosticPublisherクラス定義
- `crane_robot_receiver/include/crane_robot_receiver/robot_errors.hpp` - エラーコード定義
- `utility/crane_comm/include/crane_comm/diagnosed_publisher.hpp` - DiagnosedPublisherテンプレート

#### 実装ファイル

- `crane_robot_receiver/src/diagnostic_publisher.cpp` - DiagnosticPublisher実装
- `crane_msg_wrappers/src/world_model_wrapper.cpp` - 診断情報の利用可否判定への統合
- `crane_local_planner/src/local_planner.cpp` - LocalPlanner診断実装
- `crane_world_model_publisher/src/world_model_publisher.cpp` - Vision診断実装
- `crane_session_coordinator/src/crane_session_coordinator.cpp` - TacticCoordinator診断実装

#### 設定ファイル

- `crane_bringup/config/diagnostic_aggregator.yaml` - 診断階層構造定義
- `crane_bringup/launch/crane.launch.xml` - 診断ノード起動設定

#### メッセージ定義

- `crane_msgs/msg/RobotInfo.msg` - エラー情報フィールド
- `crane_msgs/msg/RobotFeedback.msg` - ハードウェアフィードバック

### 可視化ツール

- **Foxglove Studio**: <https://foxglove.dev/>
- **rqt_robot_monitor**: ROS 2パッケージ（`ros-${ROS_DISTRO}-rqt-robot-monitor`）

---

## まとめ

Crane診断システムは、ROS 2の標準diagnostics機能を活用し、システム全体の健全性を包括的に監視します。

**主要な設計原則**:

1. **3層アーキテクチャ**: データ収集→統合→活用により、各コンポーネントの状態を階層的に管理
2. **循環参照回避**: 入力ソース別にavailableを分離し、診断の入力（`available_vision`等）と出力（`available_hardware`）を明確に分離
3. **多層判定**: 用途に応じて`available()`, `availableStrict()`, `availableLoose()`を使い分け

診断情報は単なるログではなく、**システムの自律的な動作調整**（故障ロボットの自動除外）に活用される重要な情報です。新規診断項目の追加は[開発ガイド](#開発ガイド)を参照してください。
