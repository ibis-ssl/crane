# WorldModelWrapper

`WorldModelWrapper`はcraneプロジェクトの中核となるクラスで、ロボットサッカーの試合状況（ワールドモデル）へのアクセスを簡潔かつ統一的に提供します。このクラスを使用することで、ロボットの位置や速度、ボールの状態、フィールド情報など、試合に関するすべての情報に容易にアクセスできます。

## 概要

`WorldModelWrapper`は以下の機能を提供します：

- ROS 2トピックからのワールドモデル情報の自動購読
- 自チーム/相手チームのロボット情報へのアクセス
- ボール情報（物理モデル統合済み）へのアクセス
- フィールド・ゴール・ペナルティエリア情報
- ゲーム状態情報（PlaySituation）
- 遅延監視システム（DelayMonitor）
- ボール所有権判定（BallOwnerCalculator）
- 各種幾何学計算ユーティリティ

## 初期化と基本使用法

`WorldModelWrapper`は通常、ROS 2ノードのメンバー変数として初期化します：

```cpp
#include <crane_msg_wrappers/world_model_wrapper.hpp>

class MyNode : public rclcpp::Node
{
private:
  crane::WorldModelWrapper::SharedPtr world_model;

public:
  MyNode() : Node("my_node")
  {
    world_model = std::make_shared<crane::WorldModelWrapper>(*this);

    // コールバック登録
    world_model->addCallback([this]() {
       // 更新時の処理
    });
  }
};
```

## 主要コンポーネント

### 1. チーム情報

```cpp
// 自チーム情報
const auto& our_robots = world_model->ours().robots;
uint8_t goalie_id = world_model->getOurGoalieId();

// 利用可能なロボットの取得（退場中などを除外）
// 引数: (除外するID, キーパーを除外するか)
auto available_bots = world_model->ours().getAvailableRobots(my_id, true);

// 相手チーム情報
const auto& their_robots = world_model->theirs().robots;
```

#### RobotsQuery Fluent Builder API

より柔軟なロボット選択のための新しいAPIが追加されました。メソッドチェーンで条件を組み合わせることができます。

```cpp
// 基本的な使用方法
auto robots = world_model->ours().robotsWhere().available().get();

// 複数条件の組み合わせ
auto robots = world_model->ours().robotsWhere()
    .available()
    .excludeGoalie()
    .excludeId(my_id)
    .get();

// カスタム条件の追加
auto nearby_robots = world_model->ours().robotsWhere()
    .available()
    .where([&](const auto& robot) {
        return robot->getDistance(ball_pos) < 1.0;
    })
    .get();

// IDリストのみ取得
auto ids = world_model->ours().robotsWhere()
    .available()
    .excludeGoalie()
    .getIds();

// 数を取得
size_t count = world_model->ours().robotsWhere()
    .availableStrict()
    .count();

// 空かどうかを判定
bool has_robots = !world_model->ours().robotsWhere()
    .available()
    .empty();
```

**可用性条件**（相互排他）:

- `available()`: 標準判定（(available_vision || available_tracker) && available_hardware）
- `availableStrict()`: 厳密判定（available_vision && available_hardware && available_feedback）
- `availableLoose()`: 緩和判定（available_vision || available_tracker）

**除外条件**:

- `excludeId(uint8_t id)`: 特定IDを除外
- `excludeIds(const std::vector<uint8_t>& ids)`: 複数IDを除外
- `excludeGoalie()`: ゴーリーを除外

**カスタム条件**:

- `where(Predicate pred)`: ラムダ式で自由な条件を追加

**終端操作**:

- `get()`: フィルタリングされたロボットリスト取得
- `getIds()`: ロボットIDリスト取得
- `getView()`: Rangeビュー取得
- `count()`: ロボット数取得
- `empty()`: 空判定

### 2. ボール情報

`crane::Ball` クラスを通じて物理モデルに基づいた高度な情報にアクセスできます。

```cpp
// 基本情報
Point pos = world_model->ball().pos;
Vector2 vel = world_model->ball().vel;

// 物理予測
Point predicted_pos = world_model->ball().getPredictedPosition(1.0); // 1秒後
double stop_time = world_model->ball().getStopTime();

// 状態判定
bool is_moving = world_model->ball().isMoving();
bool is_moving_towards = world_model->ball().isMovingTowards(target_point);
```

### 3. フィールド情報

```cpp
// フィールドサイズ（半分）
Point size = world_model->fieldSize();

// ゴール・ペナルティエリア
Point goal_center = world_model->getOurGoalCenter();
std::pair<Point, Point> posts = world_model->getOurGoalPosts();
Box penalty_area = world_model->getOurPenaltyArea();

// サイド判定
bool is_positive_side = world_model->onPositiveHalf();
double side_sign = world_model->getOurSideSign(); // 1.0 or -1.0
```

### 4. ゲーム状態情報

```cpp
// プレイ状況メッセージ
const auto& situation = world_model->getMsg().play_situation;

// 判定ヘルパー
bool is_halt = (situation.command.value == crane_msgs::msg::PlaySituation::HALT);
```

## 高度な機能

### ボール所有者判定 (BallOwnerCalculator)

誰がボールに最も近いか（スラックタイム基準）を計算します。

```cpp
// 計算の有効化（重い処理のためデフォルト無効の場合あり）
world_model->setBallOwnerCalculatorEnabled(true);

// 自チームのフロンティア（最有力ロボット）取得
if (auto frontier = world_model->getOurFrontier()) {
    auto robot = frontier->robot;
    double min_slack = frontier->min_slack;
}
```

### スラックタイム計算

ロボットがボールに追いつくまでの余裕時間を計算します。

```cpp
// 全ロボットのインターセプト計算
auto results = world_model->getSlackInterceptPointAndSlackTimeArray(
    world_model->ours().getAvailableRobots()
);

// 最も早い・遅い結果を取得
auto [min, max] = world_model->getMinMaxSlackInterceptPointAndSlackTime(robots);
```

### 遅延監視 (DelayMonitor)

システム内の処理遅延を計測・伝播させるための機能です。

```cpp
// チェックポイント追加
world_model->addDelayCheckpoint("planning_start");

// 遅延計算
double delay_ms = world_model->calculateTotalDelayMs("now");
```

### 幾何学ユーティリティ

```cpp
// ゴールが見える角度範囲（シュートコース判定）
auto range = world_model->getLargestGoalAngleRangeFromPoint(pos);
if (range.angle_width > threshold) { /* シュート可能 */ }

// 線分とロボットの距離判定
auto result = world_model->getNearestRobotWithDistanceFromSegment(seg, robots);

// PointChecker（エリア判定ヘルパー）
bool in_field = world_model->point_checker.isFieldInside(pos);
bool in_penalty = world_model->point_checker.isPenaltyArea(pos);
```

## 便利な判定メソッド

- `isShootingTowardsOurGoal()`: ボールが自ゴール枠内に向かっているか
- `getBallPlacementTarget()`: ボールプレースメントの目標位置
- `getBallPlacementArea()`: プレースメント禁止エリア（Capsule）

## 注意点

- **更新タイミング**: `hasUpdated()` で更新確認が可能です。
- **スレッド安全性**: コールバックはサブスクライバスレッドで実行されるため、メインループとの競合に注意してください。
- **物理設定**: スラックタイム計算などは `SlackTimeConfig` で挙動を調整可能です。
