# ボールモデル・キッカーモデルキャリブレーションガイド

## 概要

ROSBAGデータからボール物理パラメータとキッカーパワー-速度関係を自動キャリブレーションし、統合されたKickerModelとBallPhysicsModelの設定を生成するシステム。YAMLベース統合設定ファイルで高精度なパラメータ推定を実現。

## 基本使用方法

### データ収集

#### 手動データ収集

キック練習セッションでROSBAG記録（最低20回以上のキック）

```bash
ros2 bag record /ball_info /robot_command_* -o kick_calibration_data
```

#### 自動データ収集

BallCalibrationDataCollectorTacticを使用した完全自動キャリブレーション

### ROSトピック経由での開始

```bash
# session injectionでキャリブレーション開始
ros2 topic pub --once /session_injection std_msgs/String '{data: "BALL_CALIBRATION_DATA_COLLECTION"}'
```

### 動作仕様

- 自動的に2台のロボットを選択（ID順）
- キッカーロボット: 自陣ゴール前1mの位置からキック実行  
- 球拾いロボット: ボール停止後に回収・返球
- 20サイクルの自動データ収集を実行
- 9段階のキックパワー設定(0.2-1.0)でデータ収集

### キャリブレーション実行

自動キャリブレーション（ROSBAGからJSON自動生成 → 統合YAML設定生成）

```bash
ros2 launch crane_world_model_publisher ball_calibration.launch.py auto_calibrate:=true rosbag_path:=/path/to/rosbag
```

システムは自動的に以下を実行：

1. ROSBAGから`ball_calibration_analysis/`ディレクトリにJSONデータ生成
2. グローバル減速度パラメータを0.01刻みで最適化
3. 各パワー値（0.0-1.0の0.1刻み）での平均初速度を算出
4. KickerModel + BallPhysicsModel統合YAML設定ファイルを生成
5. crane.launch.xml用設定パスを標準出力に表示

### 可視化確認

キックイベントのプロット表示

```bash
ros2 run crane_world_model_publisher plot_kick_events.py kick_event_visualization_0_data.json
```

データ概要のみ表示

```bash
ros2 run crane_world_model_publisher plot_kick_events.py data.json --summary-only
```

## 出力例

### 標準出力（KickerModel統合設定用）

```text
==================================================
KickerModel + BallPhysicsModel統合キャリブレーション結果
==================================================
以下のファイルが生成されました:
/path/to/calibrated_kicker_physics.yaml

crane.launch.xmlで以下を設定してください:
{"kicker_physics_config": "/path/to/calibrated_kicker_physics.yaml"}

測定結果詳細:
  パワー 0.00 -> 速度 0.0 m/s (サンプル数: 3)
  パワー 0.10 -> 速度 1.2 m/s (サンプル数: 4)
  ...
```

### 統合YAML設定ファイル

```yaml
# calibrated_kicker_physics.yaml
kicker_model:
  # ストレートキック設定（キャリブレーション結果）
  straight_kick_powers: [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
  straight_kick_speeds: [0.0, 1.2, 2.3, 3.1, 4.2, 5.0, 5.8, 6.5, 7.3, 8.1, 8.9]

  # チップキック設定（デフォルト値）
  chip_kick_powers: [0.0, 0.5, 0.75, 1.0]
  chip_kick_distances: [0.0, 0.3, 1.0, 2.5]

ball_physics_model:
  deceleration: 0.700      # 最適化された減速度 [m/s²]
  gravity: -9.81           # 重力加速度 [m/s²]
  air_resistance: 0.0      # 空気抵抗係数
  height_threshold: 0.05   # 飛行判定の高度閾値 [m]
  speed_threshold: 0.1     # 移動判定の速度閾値 [m/s]
  stop_threshold: 0.05     # 停止判定の速度閾値 [m/s]

calibration_info:
  physics_rmse: 0.58        # 物理モデルRMSE
  physics_r_squared: 0.85   # 物理モデルR²
  trajectories_analyzed: 25  # 解析軌道数
  trajectories_used: 19     # 有効軌道数
  calibration_date: "2025-08-15T10:30:00Z"
```

## 品質基準

- **物理モデルR²**: > 0.8
- **有効パワーグループ**: 3/11以上（各パワーで最低3サンプル）
- **有効軌道数**: > 15軌道
- **減速度範囲**: 0.5-1.0 m/s²程度

## KickerModel統合設定の適用

### 自動適用（推奨）

1. キャリブレーション実行で統合YAML設定ファイルが自動生成
2. `crane.launch.xml`が自動的に統合設定ファイルを参照
3. システム再起動で新しいKickerModel + BallPhysicsModel設定が適用

### 手動適用

生成されたYAML設定ファイルを手動で配置:

```bash
# 生成されたキャリブレーション結果をコピー
cp /path/to/calibrated_kicker_physics.yaml src/crane/crane_world_model_publisher/config/kicker_physics.yaml

# システム再起動
ros2 launch crane_bringup crane.launch.xml
```

### 新機能の使用

キャリブレーション後は以下の高度なキック機能が利用可能:

```cpp
// 停止距離指定キック
command.kickStraightToStopAt(2.5)  // 2.5m地点で停止

// 初速度直接指定キック  
command.kickStraightWithInitialSpeed(5.0)  // 5.0 m/s の初速度

// 停止距離予測
double predicted_distance = command.predictStraightKickStopDistance(4.0);
```

## トラブルシューティング

### JSON生成失敗

ROSBAGに必要なトピックが含まれているか確認

```bash
ros2 bag info /path/to/rosbag/file  # /ball_info, /robot_command_* が必要
```

### パワー別データ不足

特定のパワー値でサンプル数が少ない場合、そのパワーでの追加データ収集が必要。
BallCalibrationDataCollectorTacticで自動収集を推奨。

### 減速度最適化失敗

- 各軌道で最低0.5秒以上の有効データが必要
- ball_state=1（移動中）のデータ点が10点以上必要

### KickerModel設定エラー

`"KickerModelが設定されていません"`エラーが発生する場合:

```cpp
// RobotCommandWrapperにKickerModelを設定
auto kicker_model = createKickerModelFromYAML("config/kicker_physics.yaml");
command.setKickerModel(kicker_model);
```

### 停止距離指定キック精度不良

1. キャリブレーションデータの品質確認
2. BallPhysicsModelの減速度パラメータ調整
3. 実機環境での追加キャリブレーション実行

## CenterStopKickTacticでの検証

キャリブレーション結果の検証にはCenterStopKickTacticを使用:

```bash
# CENTER_STOP_KICKプレイシチュエーションでテスト
ros2 topic pub --once /session_injection std_msgs/String '{data: "CENTER_STOP_KICK"}'
```

このプレイシチュエーションでは:

- 停止距離指定キックの精度確認
- KickerModelとBallPhysicsModelの統合動作検証
- 実際のゲームシナリオでの動作確認
