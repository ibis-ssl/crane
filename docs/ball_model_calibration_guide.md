# ボールモデルキャリブレーションガイド

## 概要

ROSBAGデータからボール物理パラメータとキッカーパワー-速度関係を自動キャリブレーションするJSONベースシステム。固定減速度モデル（v(t) = v0 - decel*t）と0.1刻みパワーマッピング（0.0-1.0の11段階）で高精度なパラメータ推定を実現。

## 基本使用方法

### データ収集

#### 手動データ収集

キック練習セッションでROSBAG記録（最低20回以上のキック）

```bash
ros2 bag record /ball_info /robot_command_* -o kick_calibration_data
```

#### 自動データ収集

BallCalibrationDataCollectorPlannerを使用した完全自動キャリブレーション

### ROSトピック経由での開始

```bash
# session injectionでキャリブレーション開始
ros2 topic pub --once /session_injection std_msgs/String '{data: "BALL_CALIBRATION_DATA_COLLECTION"}'
```

### GUIビューアー経由での開始

1. consai_visualizerを起動:

   ```bash
   ros2 run consai_visualizer consai_visualizer
   ```

2. 「セッション挿入」コンボボックスから`BALL_CALIBRATION_DATA_COLLECTION`を選択
3. 「セッション挿入」ボタンをクリック

### 動作仕様

- 自動的に2台のロボットを選択（ID順）
- キッカーロボット: 自陣ゴール前1mの位置からキック実行  
- 球拾いロボット: ボール停止後に回収・返球
- 20サイクルの自動データ収集を実行
- 9段階のキックパワー設定(0.2-1.0)でデータ収集

### パラメータ設定

ROS 2パラメータで調整可能：

```bash
# キッカー位置調整
ros2 param set /session_controller calibration.kicker_x_offset 1.5

# 収集サイクル数調整  
ros2 param set /session_controller calibration.data_collection_cycles 30
```

### キャリブレーション実行

自動キャリブレーション（ROSBAGからJSON自動生成 → 最適化実行）

```bash
ros2 launch crane_world_model_publisher ball_calibration.launch.py auto_calibrate:=true rosbag_path:=/path/to/rosbag
```

システムは自動的に以下を実行：

1. ROSBAGから`ball_calibration_analysis/`ディレクトリにJSONデータ生成
2. グローバル減速度パラメータを0.01刻みで最適化
3. 各パワー値（0.0-1.0の0.1刻み）での平均初速度を算出
4. crane.launch.py用配列を標準出力に表示

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

### 標準出力（crane.launch.py用）

```
==================================================
crane.launch.py用キャリブレーション結果
==================================================
以下の値をcrane.launch.pyのL198-199に貼り付けてください:

                            {"straight_kick_power_array": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]},
                            {"straight_kick_speed_array": [0.0, 1.2, 2.3, 3.1, 4.2, 5.0, 5.8, 6.5, 7.3, 8.1, 8.9]},

測定結果詳細:
  パワー 0.00 -> 速度 0.0 m/s (サンプル数: 3)
  パワー 0.10 -> 速度 1.2 m/s (サンプル数: 4)
  ...
```

### YAML設定ファイル

```yaml
# calibrated_ball_physics.yaml
ball_physics_model:
  deceleration: 0.700      # 最適化された減速度 [m/s²]

kicker_power_mapping:
  straight_kick:
    power_0:
      mean_velocity: 0.0
      sample_count: 3
    power_10:
      mean_velocity: 1.2
      sample_count: 4
    # ... power_100まで

calibration_info:
  physics_rmse: 0.58        # 物理モデルRMSE
  physics_r_squared: 0.85   # 物理モデルR²
  trajectories_analyzed: 25  # 解析軌道数
  trajectories_used: 19     # 有効軌道数
```

## 品質基準

- **物理モデルR²**: > 0.8
- **有効パワーグループ**: 3/11以上（各パワーで最低3サンプル）
- **有効軌道数**: > 15軌道
- **減速度範囲**: 0.5-1.0 m/s²程度

## crane.launch.py への適用

1. キャリブレーション実行後の標準出力をコピー
2. `crane_bringup/launch/crane.launch.py` のL198-199を置換
3. システム再起動で新しいパワーマッピングが適用

## トラブルシューティング

### JSON生成失敗

ROSBAGに必要なトピックが含まれているか確認

```bash
ros2 bag info /path/to/rosbag/file  # /ball_info, /robot_command_* が必要
```

### パワー別データ不足

特定のパワー値でサンプル数が少ない場合、そのパワーでの追加データ収集が必要。
BallCalibrationDataCollectorPlannerで自動収集を推奨。

### 減速度最適化失敗

- 各軌道で最低0.5秒以上の有効データが必要
- ball_state=1（移動中）のデータ点が10点以上必要
