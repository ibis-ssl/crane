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

### 分析系メッセージ (analysis/)

- **BallAnalysis.msg**: ボール状態の分析結果
- **FieldAnalysis.msg**: フィールド状況の分析
- **GameAnalysis.msg**: 試合全体の分析データ
- **PlaySituation.msg**: プレイ状況の判定結果
- **Kick.msg**: キック動作の解析
- **Slack.msg**: 余裕度の評価
- **NamedValue系**: 名前付き値の汎用型（Bool/Float/Int/Position/String）

### 制御系メッセージ (control/)

- **RobotCommand.msg**: 個別ロボットへの制御コマンド
- **RobotCommands.msg**: 複数ロボットへのコマンド配列
- **RobotFeedback.msg**: ロボットからのフィードバック情報
- **TargetMode系**: 位置・速度・極座標での目標値指定
- **LocalCameraMode.msg**: ローカルカメラ制御
- **StateFactor.msg**: 状態評価ファクター

### ワールドモデルメッセージ (world_model/)

- **WorldModel.msg**: 統合世界モデル（全体状況）
- **BallInfo.msg**: ボール情報（位置・速度・状態）
- **RobotInfo.msg**: ロボット情報（位置・姿勢・状態）
- **BallContact.msg**: ボール接触検出
- **FieldSize.msg**: フィールド寸法定義
- **Pose2DStamped.msg**: 時刻付き2D姿勢

## サービス・アクション定義

### サービス (srv/)

- **RobotSelect.srv**: ロボット選択サービス

### アクション (action/)

- **SkillExecution.action**: スキル実行アクション

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
cmd.target_x = 1.0;
cmd.target_y = 0.5;

// ワールドモデル購読
auto subscription = create_subscription<crane_msgs::msg::WorldModel>(
  "world_model", 10, callback);
```

### Pythonでの使用例

```python
from crane_msgs.msg import RobotCommand, WorldModel

# メッセージ作成
cmd = RobotCommand()
cmd.robot_id = 0
cmd.target_x = 1.0
```

## 最近の開発状況

### 2024年の主要変更

- **Vector3D拡張**: 3Dボール座標への対応強化（2024年11月）
- **ボールモデル拡充**: ボール物理状態メッセージの改良（2024年12月）
- **メッセージ最適化**: 通信効率向上のための型調整

### 開発活発度

🟢 **安定**: 基盤メッセージとして成熟しており、主要な変更は少ないが継続的にメンテナンスされている。新機能追加時には関連メッセージが追加される。

### 今後の展望

- SSL規格変更への対応
- より高精度な状態表現の導入
- パフォーマンス最適化の継続

---

**関連パッケージ**: [crane_msg_wrappers](./crane_msg_wrappers.md) | [crane_world_model_publisher](./crane_world_model_publisher.md) | [robocup_ssl_msgs](./robocup_ssl_msgs.md)
