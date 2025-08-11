# ボールモデルキャリブレーションガイド

## 概要

ROSBAGデータからボール物理パラメータとキッカーパワー-速度関係を自動キャリブレーションするツール。Vision生データを使用してフィルタ歪みを排除し、高精度なパラメータ推定を実現。

## 基本使用方法

### データ収集

#### 手動データ収集

キック練習セッションでROSBAG記録（最低20回以上のキック）

```bash
ros2 bag record /ball_info /robot_command_* -o kick_calibration_data
```

#### 自動データ収集

BallCalibrationDataCollectorPlannerを使用した完全自動キャリブレーション

**ROSトピック経由での開始**

```bash
# session injectionでキャリブレーション開始
ros2 topic pub --once /session_injection std_msgs/String '{data: "BALL_CALIBRATION_DATA_COLLECTION"}'
```

**GUIビューアー経由での開始**

1. consai_visualizerを起動
```bash
ros2 run consai_visualizer consai_visualizer
```

2. 「セッション挿入」コンボボックスから`BALL_CALIBRATION_DATA_COLLECTION`を選択
3. 「セッション挿入」ボタンをクリック

**動作仕様**
- 自動的に2台のロボットを選択（ID順）
- キッカーロボット: 自陣ゴール前1mの位置からキック実行  
- 球拾いロボット: ボール停止後に回収・返球
- 20サイクルの自動データ収集を実行
- 9段階のキックパワー設定(0.2-1.0)でデータ収集

**パラメータ設定**

ROS 2パラメータで調整可能：
```bash
# キッカー位置調整
ros2 param set /session_controller calibration.kicker_x_offset 1.5

# 収集サイクル数調整  
ros2 param set /session_controller calibration.data_collection_cycles 30
```

### キャリブレーション実行

自動キャリブレーション（最新ROSBAGを自動検出）

```bash
ros2 launch crane_world_model_publisher ball_calibration.launch.py auto_calibrate:=true
```

特定のROSBAGを指定

```bash
ros2 launch crane_world_model_publisher ball_calibration.launch.py auto_calibrate:=true rosbag_path:=/path/to/rosbag
```

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

```yaml
# calibrated_ball_physics.yaml
ball_physics_model:
  deceleration: 0.485      # 最適化された減速度

kicker_power_mapping:
  straight_kick:
    linear_coefficient: 2.45  # パワー-速度関係
    r_squared: 0.89          # 決定係数

calibration_info:
  physics_rmse: 0.08        # 物理モデル精度 [m]
  physics_r_squared: 0.92   # 決定係数
```

## 品質基準

- **位置予測RMSE**: < 0.2 m
- **物理モデルR²**: > 0.8
- **キッカーモデルR²**: > 0.7
- **データ点数**: > 15点

## 制限事項

- ストレートキックのみ対応
- 転がりボール物理のみ最適化
- Vision生データのノイズ影響（統計的補正で対応）

## トラブルシューティング

**"有効なキックデータが見つからない"**

必要なトピックが含まれているか確認

```bash
ros2 bag info /path/to/rosbag/file
```

**最適化失敗**

- キック後3秒以上の軌道記録が必要
- kick_power > 0 で設定されているか確認
- 最低20回以上のキックデータを収集
