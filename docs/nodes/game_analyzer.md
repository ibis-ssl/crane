# Game Analyzer

Game Analyzer（ゲームアナライザ）は、試合状況をリアルタイムで分析し、高度な戦略決定をサポートするためのノードです。主に`/world_model`トピックから得られる情報（ボールの状態、ロボットの位置・速度など）を元に解析を行います。Referee情報に直接由来するゲーム状態の解釈は行いませんが、ワールドモデルに含まれるゲーム状態（例：`PlaySituation`）を利用することはあります。

## 概要

Game Analyzerは主に以下の機能を提供します：

- ボールの状態分析（停止検知など）
- ロボット間の衝突検知
- パス判定
- 各種試合状況の可視化

分析結果は主に視覚化メッセージとして表示されます。以前は`crane_msgs::msg::GameAnalysis`メッセージを発行していましたが、この機能は他のノード（例：`world_model_publisher`）に移行されたか、現在はアクティブではありません。計算負荷の高い処理は数フレームに1回実行され、それ以外のフレームでは前回の結果が使用されることがあります。

## アーキテクチャの変更

最近のアップデートにより、Game Analyzerの一部機能は`world_model_publisher`ノードに統合されました。この変更により、データの流れがよりシンプルになり、パフォーマンスが向上しています。

## ボールの状態検知

### ボール停止検知

`getBallIdle()`メソッドは、ボールが一定時間内に閾値以上動いていないかどうかを検出します。これは以下の手順で実行されます：

1. ボールの位置と時間をスタンプ付きで記録
2. 過去の記録（最大で閾値の2倍の時間分）を保持
3. 最新の位置と過去の位置を比較し、動きが閾値以下であればボールは停止していると判断

設定パラメータ：

- `threshold_duration`: 判定に使用する時間閾値（デフォルト: 5秒）
- `move_distance_threshold_meter`: 動きの閾値（デフォルト: 0.05m）

```cpp
bool getBallIdle() {
  // 最新のボール位置と時間を記録
  BallPositionStamped record;
  record.position = world_model->ball.pos;
  record.stamp = now();
  static std::deque<BallPositionStamped> ball_records;
  ball_records.push_front(record);

  // 古い記録を削除
  auto latest_time = ball_records.front().stamp;
  auto latest_position = ball_records.front().position;
  std::erase_if(ball_records, [&](auto & ball_record) {
    return (latest_time - ball_record.stamp) > config.ball_idle.threshold_duration * 2;
  });

  // ボールが動いているかチェック
  return not std::ranges::any_of(ball_records, [&](const auto & ball_record) {
    bool distance_cond = (latest_position - ball_record.position).norm() <
                       config.ball_idle.move_distance_threshold_meter;
    bool time_cond = (latest_time - ball_record.stamp) < config.ball_idle.threshold_duration;
    return distance_cond && time_cond;
  });
}
```

## ロボット衝突検知機能

最新のアップデートで実装されたロボット衝突検知機能は、試合中のロボット間の衝突を自動的に検出し、視覚化するための機能です。この機能は、以下の3つの主要なメソッドから構成されています：

### 1. ロボット状態記録（recordCurrentRobotStates）

```cpp
void recordCurrentRobotStates() {
  auto current_time = now();

  // 自チームのロボット位置を記録
  for (const auto & robot : world_model->ours.getAvailableRobots()) {
    RobotPositionStamped record;
    record.id = robot->id;
    record.is_ours = true;
    record.position = robot->pose.pos;
    record.velocity = robot->vel.linear;
    record.stamp = current_time;
    robot_records_.push_front(record);
  }

  // 相手チームのロボット位置を記録
  for (const auto & robot : world_model->theirs.getAvailableRobots()) {
    RobotPositionStamped record;
    record.id = robot->id;
    record.is_ours = false;
    record.position = robot->pose.pos;
    record.velocity = robot->vel.linear;
    record.stamp = current_time;
    robot_records_.push_front(record);
  }

  // 古い記録を削除
  auto time_threshold = current_time - rclcpp::Duration::from_seconds(config.robot_collision.time_window * 2);
  std::erase_if(robot_records_, [&](const auto & record) {
    return record.stamp < time_threshold;
  });
}
```

このメソッドは：

- 自チームと相手チームのロボット位置と速度を記録
- 時系列データをリングバッファとして管理
- 設定された時間ウィンドウより古いデータを自動的に削除

### 2. 衝突検知アルゴリズム（detectCollision）

```cpp
std::optional<RobotCollisionInfo> detectCollision() {
  // 全てのロボットペアをチェック
  for (size_t i = 0; i < world_model->ours.robots.size(); ++i) {
    auto & our_robot = world_model->ours.robots[i];

    for (size_t j = 0; j < world_model->theirs.robots.size(); ++j) {
      auto & their_robot = world_model->theirs.robots[j];

      // ロボット間の距離
      double distance = (our_robot->pose.pos - their_robot->pose.pos).norm();

      // 距離が閾値以下ならば衝突の可能性
      if (distance < config.robot_collision.distance_threshold) {
        // 相対速度を計算
        Vector2 relative_velocity = our_robot->vel.linear - their_robot->vel.linear;
        double rel_vel_norm = relative_velocity.norm();

        // 相対速度が閾値以上ならば衝突と判定
        if (rel_vel_norm > config.robot_collision.velocity_threshold) {
          // 速度ベクトルが互いに向かい合っているかチェック
          Vector2 direction = (their_robot->pose.pos - our_robot->pose.pos).normalized();
          double approach_factor = relative_velocity.normalized().dot(direction);

          // 正の値は互いに近づいていることを示す
          if (approach_factor > 0.5) {
            RobotCollisionInfo info;

            // 速度が大きい方を「攻撃側」と判定
            if (our_robot->vel.linear.norm() > their_robot->vel.linear.norm()) {
              info.attack_robot = RobotIdentifier{true, our_robot->id};
              info.attacked_robot = RobotIdentifier{false, their_robot->id};
            } else {
              info.attack_robot = RobotIdentifier{false, their_robot->id};
              info.attacked_robot = RobotIdentifier{true, our_robot->id};
            }

            info.relative_velocity = rel_vel_norm;
            return info;
          }
        }
      }
    }
  }

  return std::nullopt;
}
```

衝突検知アルゴリズムは以下の基準で衝突を判定します：

1. **距離条件**: ロボット間の距離が閾値（デフォルト: 0.2m）未満
2. **相対速度条件**: ロボット間の相対速度が閾値（デフォルト: 1.0m/s）を超過
3. **接近方向条件**: 相対速度ベクトルがロボット同士を結ぶ方向と一致（衝突方向に動いている）

衝突が検出されると、より高速に動いていたロボットを「攻撃側」として判定し、衝突情報を返します。

### 3. 衝突の可視化（visualizeCollision）

```cpp
void visualizeCollision(const RobotCollisionInfo & collision) {
  // 衝突ロボットの位置を取得
  Point attack_pos, attacked_pos;

  if (collision.attack_robot.is_ours) {
    attack_pos = world_model->getOurRobot(collision.attack_robot.id)->pose.pos;
  } else {
    attack_pos = world_model->getTheirRobot(collision.attack_robot.id)->pose.pos;
  }

  if (collision.attacked_robot.is_ours) {
    attacked_pos = world_model->getOurRobot(collision.attacked_robot.id)->pose.pos;
  } else {
    attacked_pos = world_model->getTheirRobot(collision.attacked_robot.id)->pose.pos;
  }

  // 衝突点を可視化（中間点）
  Point collision_point = (attack_pos + attacked_pos) * 0.5;

  // 衝突箇所に赤い円を描画
  visualizer->circle()
    .center(collision_point)
    .radius(0.15)
    .stroke("red")
    .strokeWidth(3)
    .fill("red", 0.3)
    .build();

  // 衝突ロボット間に線を描画
  visualizer->line()
    .start(attack_pos)
    .end(attacked_pos)
    .stroke("red")
    .strokeWidth(2)
    .build();

  // 速度表示
  std::string velocity_text = std::to_string(collision.relative_velocity).substr(0, 4) + " m/s";
  visualizer->text()
    .position(collision_point + Vector2(0, 0.2))
    .text(velocity_text)
    .fontSize(40)
    .fill("red")
    .textAnchor("middle")
    .build();
}
```

衝突検知結果は以下の形で視覚化されます：

- 衝突点（両ロボットの中間点）に赤い円を表示
- 衝突したロボット間を赤い線で結ぶ
- 衝突時の相対速度を衝突点近くにテキスト表示

### 設定パラメータ

ロボット衝突検知機能は以下のパラメータで調整できます：

- `velocity_threshold`: 衝突判定の速度閾値（デフォルト: 1.0 m/s）
- `distance_threshold`: 衝突判定の距離閾値（デフォルト: 0.2 m）
- `time_window`: 履歴保存の時間窓（デフォルト: 0.5 秒）

これらのパラメータはlaunchファイルやコマンドラインから変更できます：

```bash
ros2 run crane_game_analyzer crane_game_analyzer_node --ros-args -p robot_collision.velocity_threshold:=1.2
```

## 活用方法

ロボット衝突検知機能は以下のような用途に活用できます：

1. 試合分析 - 危険な衝突の検出・記録
2. 戦略改善 - 衝突の多い場所や状態の特定
3. ロボット制御の評価 - 衝突回避アルゴリズムの検証
4. 審判判定の補助 - 衝突の責任の明確化

## 可視化

Game Analyzerは、分析結果を視覚的に表示するために`CraneVisualizerBuffer`を使用します。これにより、デバッグや状況理解が容易になります。

```cpp
visualizer = std::make_shared<VisualizerMessageBuilder>("game_analyzer");
CraneVisualizerBuffer::activate(*this);
// ...
visualizer->flush();
CraneVisualizerBuffer::publish();
```

## 注意事項

- 一部の機能（ボール所持判定など）は最適化のために`world_model_publisher`に移動されました。`game_analyzer`コード内には、この移動された機能に関連する古いパラメータ定義（例：`ball_possession.threshold_meter`）が残っている場合がありますが、これらは現在`game_analyzer`内では使用されていません。
- パス判定機能は継続的に改善されています。
- パフォーマンスを重視するため、重い処理は間引いて実行されることがあります。
- 本ノードは`/world_model`トピックから情報を取得して分析を行います。

## 使用方法

Game Analyzerは通常、起動スクリプトの一部として自動的に起動されますが、単独で起動することも可能です：

```bash
ros2 run crane_game_analyzer crane_game_analyzer_node
```

または、compositionを使用して：

```bash
ros2 component load /component_container crane_game_analyzer::GameAnalyzerComponent
```

## 将来の開発計画

- パス成功率の予測モデルの実装
- 機械学習を用いたより高度な状況認識
- リアルタイムヒートマップ生成による戦術分析
