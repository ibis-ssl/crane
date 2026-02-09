# crane_msgs

## 概要

**crane_msgs**パッケージは、CRANE SSL（RoboCup Small Size League）自律ロボットサッカーシステムの中核となるメッセージ定義パッケージです。システム内の全コンポーネント間で使用されるROS 2メッセージ型、サービス、アクションの包括的なセットを提供し、型安全で効率的な通信インフラを実現します。

## 主要機能

- **分析系メッセージ**: 試合状況・ボール解析・戦術判断用のメッセージ定義
- **制御系メッセージ**: ロボット制御・コマンド送信用のメッセージ定義  
- **ワールドモデルメッセージ**: 世界状態・ロボット状態・ボール情報の表現
- **統一インターフェース**: システム全体のメッセージ標準化

## アーキテクチャ上の役割

Craneシステムの**メッセージ基盤層**として、全コンポーネント間のデータ交換を支える基礎パッケージです。ROS 2のIDL（Interface Definition Language）機能を活用し、型安全なメッセージ通信を実現します。

## メッセージ定義

### 分析系メッセージ

- **BallAnalysis.msg**: ボール状態の分析結果
- **FieldAnalysis.msg**: フィールド状況の分析
- **GameAnalysis.msg**: 試合全体の分析データ
- **PlaySituation.msg**: プレイ状況の判定結果
- **Kick.msg**: キック動作の解析
- **Slack.msg**: 余裕度の評価
- **NamedValue系**: 名前付き値の汎用型（Bool/Float/Int/Position/String）

### 制御系メッセージ

- **RobotCommand.msg**: 個別ロボットへの制御コマンド
- **RobotCommands.msg**: 複数ロボットへのコマンド配列
- **RobotFeedback.msg**: ロボットからのフィードバック情報
- **RobotSelectResults.msg**: ロボット選択結果
- **TargetMode系**: 位置・速度・極座標での目標値指定
- **LocalCameraMode.msg**: ローカルカメラ制御
- **RobotCommand.msg / RobotCommands.msg**: 統一ロボット制御コマンド（位置/速度両モード）

### 予測予実管理系メッセージ（PR #1105-1107）

ボールやキックの予測精度を評価するための予実比較メッセージ群。

**ボール予測**:

- **BallPredictionActual.msg**: 過去の予測と実際の位置・速度の比較データ
- **BallPredictionPoint.msg**: 予測軌道上の単一点（位置・速度・状態）
- **BallPredictionTrace.msg**: 予測軌道全体（複数Pointの配列）

**キック予測**:

- **KickPredictionActual.msg**: キック後のボール軌道の予測vs実績
- **KickPredictionPoint.msg**: キック予測軌道上の単一点
- **KickPredictionTrace.msg**: キック予測軌道全体

**速度計画**:

- **VelocityCorrection.msg**: 速度修正の詳細（RVO2等による修正内容）
- **VelocityPlanActual.msg**: 速度計画の予実比較
- **VelocityPlanPoint.msg**: 速度計画上の単一点
- **VelocityPlanTrace.msg**: 速度計画の軌道全体

### デバッグ・アノテーション系メッセージ

- **HumanAnnotation.msg** (PR #1103): リアルタイムアノテーション用
  - カテゴリ: ISSUE/OBSERVATION/QUESTION/POSITIVE/STRATEGY/TIMING/CUSTOM
  - 優先度: LOW/MEDIUM/HIGH/CRITICAL
  - 位置情報、ロボットコンテキスト、タイムスタンプ含む

## サービス・アクション定義

### サービス (srv/)

現在、サービス定義はありません。

### アクション (action/)

## 依存関係

- **基本依存**: `builtin_interfaces`, `geometry_msgs`, `std_msgs`
- **SSL依存**: `robocup_ssl_msgs`（SSL公式プロトコル）
- **ビルド依存**: `rosidl_default_generators`（メッセージ生成）
- **実行依存**: `rosidl_default_runtime`

## 設計思想

### 3Dボール座標系

```cpp
// BallInfo.msg での3D座標表現
geometry_msgs/Vector3 position  # x,y,z座標
geometry_msgs/Vector3 velocity  # x,y,z速度
```

position.z/velocity.z フィールドを活用してボールの3D軌道を表現し、空中でのボール物理シミュレーションに対応。

### 統一命名規則

- **Info系**: 状態情報（BallInfo, RobotInfo）
- **Analysis系**: 分析結果（BallAnalysis, GameAnalysis）
- **Command系**: 制御指令（RobotCommand, RobotCommands）
- **Named系**: 名前付きデータ型

## 使用方法

### C++での使用例

```cpp
#include "crane_msgs/msg/robot_command.hpp"
#include "crane_msgs/msg/world_model.hpp"

// ロボットコマンド作成
crane_msgs::msg::RobotCommand cmd;
cmd.robot_id = 0;
cmd.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
cmd.position_target_mode.emplace_back();
cmd.position_target_mode.front().target_x = 1.0;
cmd.position_target_mode.front().target_y = 0.5;

// ワールドモデル購読
auto subscription = create_subscription<crane_msgs::msg::WorldModel>(
  "world_model", 10, callback);
```

### Pythonでの使用例

```python
from crane_msgs.msg import PositionTargetMode, RobotCommand, WorldModel

# メッセージ作成
cmd = RobotCommand()
cmd.robot_id = 0
cmd.control_mode = RobotCommand.POSITION_TARGET_MODE
cmd.position_target_mode = [PositionTargetMode(target_x=1.0, target_y=0.5)]
```

## 最近の開発状況

### 2026年の主要変更

- **予測予実管理メッセージ群追加** (PR #1105-1107, 2026年1月):
  - ボール/キック予測の精度評価用メッセージ（9種追加）
  - 速度計画の予実比較メッセージ
  - デバッグツールとの連携強化
- **HumanAnnotationメッセージ追加** (PR #1103, 2026年1月):
  - リアルタイムアノテーション用のメッセージ定義
  - カテゴリ・優先度・コンテキスト情報を含む構造化メッセージ
- **メッセージ構造フラット化** (PR #1118, 2026年1月):
  - `analysis/`、`control/`サブディレクトリの廃止
  - 全メッセージを`msg/`直下に統一配置

### 2025年の主要変更

- **Vision/Tracker統合**: WorldModel向けにトラッカー観測の確信度フィールドを追加（2025年2月）
- **BallState刷新**: 3段階状態＋飛翔フラグの再整理で推定精度を改善（2025年3月）
- **通信効率化**: ロボットコマンドのcompactフィールド追加によるバースト送信量削減

### 開発活発度

🟡 **活発**: 2026年1月に予測予実管理機能の追加により大幅な拡張。基盤メッセージとして成熟しているが、新機能追加に伴い継続的に拡張されている。

### 今後の展望

- SSL規格変更への対応
- より高精度な状態表現の導入
- パフォーマンス最適化の継続
- 予測精度評価の可視化強化

---

**関連パッケージ**: [crane_msg_wrappers](./crane_msg_wrappers.md) | [crane_world_model_publisher](./crane_world_model_publisher.md) | [robocup_ssl_msgs](./robocup_ssl_msgs.md)
