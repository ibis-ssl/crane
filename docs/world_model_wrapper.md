# WorldModelWrapper

`WorldModelWrapper`はcraneプロジェクトの中核となるクラスで、ロボットサッカーの試合状況（ワールドモデル）へのアクセスを簡潔かつ統一的に提供します。このクラスを使用することで、ロボットの位置や速度、ボールの状態、フィールド情報など、試合に関するすべての情報に容易にアクセスできます。

## 概要

`WorldModelWrapper`は以下の機能を提供します：

- ROS 2トピックからのワールドモデル情報の自動購読
- 自チーム/相手チームのロボット情報へのアクセス
- ボール情報へのアクセス
- フィールド情報へのアクセス
- ゲーム状態情報へのアクセス
- 情報更新時のコールバック機能
- 各種便利な計算機能（最近傍ロボットの取得、角度計算など）

## 初期化と基本使用法

`WorldModelWrapper`は通常、ROS 2ノードのメンバー変数として初期化します：

```cpp
#include <crane_msg_wrappers/world_model_wrapper.hpp>

class MyNode : public rclcpp::Node
{
private:
  crane::WorldModelWrapper::UniquePtr world_model;

public:
  MyNode()
  : Node("my_node")
  {
    world_model = std::make_unique<crane::WorldModelWrapper>(*this);

    // 更新コールバックの設定（オプション）
    world_model->addCallback([this]() {
      // ワールドモデルが更新されたときに実行される処理
      processWorldModel();
    });
  }

  void processWorldModel()
  {
    // ワールドモデル情報の利用
    auto ball_pos = world_model->ball.pos;
    // ...
  }
};
```

## 主要コンポーネント

`WorldModelWrapper`は以下の主要コンポーネントを持っています：

### 1. チーム情報

```cpp
// 自チームのロボット情報
auto our_robots = world_model->ours.robots;
auto our_goalie_id = world_model->getOurGoalieId();
auto available_robots = world_model->ours.getAvailableRobots();

// 相手チームのロボット情報
auto their_robots = world_model->theirs.robots;
auto their_goalie_id = world_model->getTheirGoalieId();
```

#### 利用可能なロボットの取得

特定の条件でロボットをフィルタリングして取得することができます：

```cpp
// 特定のロボットIDを除外して取得
auto robots = world_model->ours.getAvailableRobots(excluded_robot_id);

// 特定の位置にいるロボットのみを取得（自コートのみ、相手コートのみなど）
auto own_half_robots = world_model->ours.getAvailableRobots(
  [&](const RobotInfo::SharedPtr & robot) {
    return robot->pose.pos.x() < 0.0;  // 自コートにいるロボットのみ
  });

// IDと条件の両方を使用
auto filtered_robots = world_model->ours.getAvailableRobots(
  excluded_robot_id,
  [&](const RobotInfo::SharedPtr & robot) {
    return robot->pose.pos.norm() < 3.0;  // 原点から3m以内のロボットのみ
  });
```

#### 特定IDのロボット取得

```cpp
// IDを指定して自チームのロボットを取得
auto robot = world_model->getOurRobot(robot_id);

// IDを指定して相手チームのロボットを取得
auto enemy_robot = world_model->getTheirRobot(robot_id);
```

### 2. ボール情報

```cpp
// ボールの位置
Point ball_pos = world_model->ball.pos;

// ボールの速度
Vector2 ball_vel = world_model->ball.vel;

// ボールが特定の方向に向かっているかの判定
bool is_moving_towards = world_model->ball.isMovingTowards(target_pos);

// ボールが特定の方向から離れているかの判定
bool is_moving_away = world_model->ball.isMovingAwayFrom(source_pos);
```

### 3. フィールド情報

```cpp
// フィールドサイズ（半分のサイズ）
auto field_size = world_model->field_size;

// 自陣ゴール中心
auto our_goal_center = world_model->getOurGoalCenter();

// 敵陣ゴール中心
auto their_goal_center = world_model->getTheirGoalCenter();

// 自陣ペナルティエリア
auto our_penalty_area = world_model->getOurPenaltyArea();

// 敵陣ペナルティエリア
auto their_penalty_area = world_model->getTheirPenaltyArea();
```

### 4. ゲーム状態情報

```cpp
// 現在のプレイコマンド
auto play_command = world_model->getMsg().play_situation.command.value;

// HALTかどうか
bool is_halt = (play_command == crane_msgs::msg::PlaySituation::HALT);

// STOPかどうか
bool is_stop = (play_command == crane_msgs::msg::PlaySituation::STOP);

// 自チームのフリーキックかどうか
bool is_our_freekick = (play_command == crane_msgs::msg::PlaySituation::OUR_DIRECT_FREE);
```

## 便利な計算機能

`WorldModelWrapper`は様々な便利な計算機能を提供しています：

### 1. 最近傍ロボットの取得

```cpp
// 特定の位置と最も近いロボットとその距離を取得
auto nearest_result = world_model->getNearestRobotWithDistanceFromPoint(
  target_pos, world_model->ours.getAvailableRobots());
if (nearest_result) {
  auto [nearest_robot, distance] = *nearest_result;
  // nearest_robot を使用...
}

// 特定の線分と最も近いロボットとその距離を取得
Segment line_segment{Point(0.0, 0.0), Point(1.0, 1.0)};
auto nearest_to_line = world_model->getNearestRobotWithDistanceFromSegment(
  line_segment, world_model->ours.getAvailableRobots());
if (nearest_to_line) {
  auto [nearest_robot, distance] = *nearest_to_line;
  // nearest_robot を使用...
}
```

### 2. ゴールに関する計算

```cpp
// 特定の位置からゴールが見える最大角度範囲を取得
auto goal_angle_range = world_model->getLargestGoalAngleRangeFromPoint(shot_pos);
double best_angle = goal_angle_range.center_angle;
double goal_angle_width = goal_angle_range.angle_width;

// 特定の位置から自ゴールが見える最大角度範囲を取得
auto our_goal_angle_range = world_model->getLargestOurGoalAngleRangeFromPoint(shot_pos);
```

### 3. ボールの将来位置予測とスラックタイム計算

```cpp
// ボールの将来位置のシーケンスを取得
auto ball_sequence = world_model->getBallSequence(5.0, 0.1); // 5秒先まで、0.1秒間隔

// スラックタイム計算（ロボットがボールに追いつけるかの判定）
auto slack_time_results = world_model->getSlackInterceptPointAndSlackTimeArray(
  world_model->ours.getAvailableRobots(), 5.0, 0.1, 0.0, 4.0, 5.0, 3.0);
// 引数: 対象ロボットリスト, 予測時間, 予測ステップ, スラック時間オフセット, 最大加速度, 最大速度, 距離ホライズン

// 最小と最大のスラックタイムを取得
auto [min_slack, max_slack] = world_model->getMinMaxSlackInterceptPointAndSlackTime(
  world_model->ours.getAvailableRobots());
if (min_slack) {
  // ボールに追いつける場合の処理
  auto intercept_point = min_slack->intercept_point;
  double slack_time = min_slack->slack_time;
  auto robot = min_slack->robot;
}
```

### 4. 距離計算

```cpp
// ロボットからボールまでの距離を計算
double distance = world_model->getDistanceFromRobotToBall(robot_id);

// IDを指定してロボットから特定の点までの距離を計算
double distance = world_model->getDistanceFromRobot(robot_id, point);

// ボールから特定の点までの距離を計算
double distance = world_model->getDistanceFromBall(point);

// 特定の線分と最も近い相手ロボットとその距離を取得
auto nearest = world_model->getNearestRobotWithDistanceFromSegment(
  segment, world_model->theirs.getAvailableRobots());
if (nearest) {
  auto [nearest_robot, distance] = *nearest;
  // nearest_robot を使用...
}
```

## ボール所有者の計算

`WorldModelWrapper`はボール所有者（ボールに最も近づける/ボールを扱える）ロボットの計算機能を提供しています：

```cpp
// ボール所有者計算機能の有効化
world_model->setBallOwnerCalculatorEnabled(true);

// 自チームのフロンティア（ボールに最も早く到達できるロボット）を取得
auto our_frontier = world_model->getOurFrontier();
if (our_frontier) {
  auto robot = our_frontier->robot;
  double min_slack = our_frontier->min_slack; // ボールに追いつける最小スラックタイム
  double max_slack = our_frontier->max_slack; // ボールに追いつける最大スラックタイム
  double score = our_frontier->score; // 総合評価スコア
}

// 相手チームのフロンティアを取得
auto their_frontier = world_model->getTheirFrontier();
```

## チーム側の取得

`WorldModelWrapper`は、チームが配置されるフィールド側に関する情報を提供します：

```cpp
// 自チームが配置されるフィールド側の符号を取得 (正側なら1.0、負側なら-1.0)
double side_sign = world_model->getOurSideSign();

// 自チームが正側に配置されているかどうか
bool is_on_positive = world_model->onPositiveHalf();

// 自チームが黄色チームかどうか
bool is_yellow = world_model->isYellow();

// ロボットを配置する側が正側かどうか
bool is_emplace_positive = world_model->isEmplacePositiveSide();
```

## その他の機能

### PointChecker

`WorldModelWrapper`は、点がフィールドのさまざまな領域に含まれるかどうかをチェックする機能を提供しています：

```cpp
// PointCheckerを取得
auto & point_checker = world_model->point_checker;

// 点がフィールド内にあるかどうかをチェック
bool is_inside = point_checker.isFieldInside(point, 0.1);  // 0.1はフィールド境界からのオフセット

// 点が敵陣ペナルティエリア内にあるかどうかをチェック
bool is_enemy_penalty = point_checker.isEnemyPenaltyArea(point);

// 点が自陣ペナルティエリア内にあるかどうかをチェック
bool is_friend_penalty = point_checker.isFriendPenaltyArea(point);

// 点が任意のペナルティエリア内にあるかどうかをチェック
bool is_penalty = point_checker.isPenaltyArea(point);

// 点が自陣側にあるかどうかをチェック
bool is_our_half = point_checker.isInOurHalf(point);

// 点からの距離をチェック
bool is_close = point_checker.checkDistance(
  point, target, 1.0, PointChecker::Rule::LESS_THAN);
```

### コールバック機能

`WorldModelWrapper`は、ワールドモデルが更新されたときに呼び出されるコールバック関数を登録することができます：

```cpp
// コールバック関数の追加
world_model->addCallback([this]() {
  // ワールドモデルが更新されたときの処理
  processBall();
  processRobots();
});
```

### ユーティリティ機能

`WorldModelWrapper`は、いくつかのユーティリティ機能も提供しています：

```cpp
// 現在のメッセージを取得
const auto & msg = world_model->getMsg();

// 編集可能なメッセージを取得
auto & editable_msg = world_model->getEditableMsg();

// ボール位置を上書き
world_model->overwriteBallPos(Point(1.0, 2.0));

// フィールドポイントを生成
auto points = world_model->generateFieldPoints(0.5);  // 0.5mグリッド
```

## 実用的な例

### 例1: 最も近いロボットをボールに向かわせる

```cpp
void moveNearestRobotToBall()
{
  // ボールに最も近いロボットとその距離を取得
  auto nearest_result = world_model->getNearestRobotWithDistanceFromPoint(
    world_model->ball.pos, world_model->ours.getAvailableRobots());

  if (nearest_result) {
    auto [nearest_robot, distance] = *nearest_result;

    // ロボットをボールに向かわせる
    Vector2 direction = world_model->ball.pos - nearest_robot->pose.pos;
    double target_angle = std::atan2(direction.y(), direction.x());

    // ロボットコマンドを生成して送信
    // ...
  }
}
```

### 例2: シュートの可能性を評価する

```cpp
double evaluateShotOpportunity(const Point & shoot_pos)
{
  // ゴールが見える最大角度範囲を取得
  auto goal_angle_range = world_model->getLargestGoalAngleRangeFromPoint(shoot_pos);
  double best_angle = goal_angle_range.center_angle;
  double goal_angle_width = goal_angle_range.angle_width;

  // 角度範囲が狭すぎる場合は低評価
  if (goal_angle_width < 0.1) {  // 約5.7度未満
    return 0.0;
  }

  // ボールからシュート位置までの距離
  double distance_to_ball = (shoot_pos - world_model->ball.pos).norm();

  // シュート位置から敵ゴールまでの距離
  double distance_to_goal = (shoot_pos - world_model->getTheirGoalCenter()).norm();

  // 距離と角度に基づいて評価値を計算
  // 近いほど、角度が広いほど評価が高い
  double score = goal_angle_width * 10.0 / (distance_to_ball * 0.3 + distance_to_goal * 0.7);

  return std::clamp(score, 0.0, 1.0);
}
```

### 例3: ボールインターセプトの計算

```cpp
void interceptBall()
{
  // ボールの将来位置とスラックタイムを計算
  auto [min_slack, max_slack] = world_model->getMinMaxSlackInterceptPointAndSlackTime(
    world_model->ours.getAvailableRobots(), 5.0, 0.1, 0.0, 4.0, 5.0, 5.0);

  if (min_slack) {
    // ボールに追いつける場合の処理
    auto robot = min_slack->robot;
    auto intercept_point = min_slack->intercept_point;
    double slack_time = min_slack->slack_time;

    if (slack_time > 0.0) {
      // ロボットをインターセプトポイントに向かわせる
      // ...
    } else {
      // ボールに追いつけない場合の処理
      // ...
    }
  }
}
```

## 注意点

- `WorldModelWrapper`は内部でROS 2のサブスクリプションを使用しており、メッセージの受信は非同期に行われます。そのため、初期化直後はデータが揃っていない可能性があります。
- 複数のコールバック関数を登録する場合、それらの実行順序は保証されません。
- 大量の計算や描画を行うコールバック関数を登録すると、パフォーマンスに影響を与える可能性があります。
- 座標系の変換を行う場合は、絶対座標系と相対座標系の違いに注意してください。

## まとめ

`WorldModelWrapper`は、ロボットサッカーの試合状況にアクセスするための強力なツールです。このクラスを効果的に使用することで、ロボットの位置や動き、ボールの状態、フィールド情報などに簡単にアクセスでき、複雑な戦略アルゴリズムの実装が容易になります。

また、提供される様々な便利関数により、よく使われる計算や判定を簡潔に記述することができます。これにより、開発者はゲーム戦略の核心部分に集中することができます。
