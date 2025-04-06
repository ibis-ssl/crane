# Game Analyzer

Game Analyzer（ゲームアナライザ）は、試合状況をリアルタイムで分析し、高度な戦略決定をサポートするためのノードです。Refereeから直接提供されない試合中の状況（ボールの状態、衝突検知など）を解析し、他のノードが利用できる形で配信します。

## 概要

Game Analyzerは主に以下の機能を提供します：

- ボールの状態分析（停止検知など）
- ロボット間の衝突検知
- パス判定
- 各種試合状況の可視化

分析結果は`crane_msgs::msg::GameAnalysis`メッセージとして毎フレーム配信されます。計算負荷の高い処理は数フレームに1回実行され、それ以外のフレームでは前回の結果が使用されます。

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

## ロボット衝突検知

`getRobotCollisionInfo()`メソッドは、ロボット間の衝突を検出し、衝突情報（攻撃側ロボット、被攻撃側ロボット、相対速度）を提供します。

```cpp
std::optional<RobotCollisionInfo> getRobotCollisionInfo() {
  // 検出ロジック（TODO: 実装中）
  return std::nullopt;
}
```

衝突が検出された場合、ログにメッセージが出力されます：

```cpp
if (robot_collision_info) {
  RCLCPP_INFO(
    get_logger(), "Collision Detected : ( %d, %d ) , %f [m/s]",
    robot_collision_info->attack_robot.id, robot_collision_info->attacked_robot.id,
    robot_collision_info->relative_velocity);
}
```

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

- 一部の機能（ボール所持判定など）は最適化のために`world_model_publisher`に移動されました
- パス判定機能は継続的に改善されています
- パフォーマンスを重視するため、重い処理は間引いて実行されます

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
