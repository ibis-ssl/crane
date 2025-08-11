# ボールモデルキャリブレーションガイド

## 概要

このシステムは、ROSBAGデータを解析してボール物理パラメータとキッカーパワー-速度関係を自動キャリブレーションするためのツールです。

## 機能

- **データ抽出**: ROSBAGからキックイベントとボール軌道を自動抽出（**Vision生データ使用**）
- **物理パラメータ最適化**: 最小二乗法による減速度パラメータの最適化
- **キッカーモデル最適化**: 線形回帰によるパワー-速度関係の導出
- **品質検証**: 予測精度の評価と品質レポート生成
- **可視化機能**: キックイベントの軌道プロット生成（matplotlib使用）
- **テレポート検出**: Vision オクルージョンによる異常データの自動除外

## システム構成

```
crane_world_model_publisher/
├── calibration/
│   ├── include/calibration/
│   │   ├── ball_calibration_data_extractor.hpp
│   │   ├── simple_ball_physics_optimizer.hpp
│   │   └── calibration_validator.hpp
│   ├── src/calibration/
│   │   ├── ball_calibration_data_extractor.cpp
│   │   ├── simple_ball_physics_optimizer.cpp
│   │   ├── calibration_validator.cpp
│   │   └── ball_calibration_node.cpp
│   ├── config/
│   │   └── default_ball_physics.yaml
│   └── launch/
│       └── ball_calibration.launch.py
└── docs/
    └── ball_model_calibration_guide.md
```

## 使用方法

### 1. データ収集

キャリブレーション用のROSBAGデータを収集します：

```bash
# キック練習セッションでROSBAG記録
ros2 bag record /ball_info /robot_command_* -o kick_calibration_data

# 推奨データ収集条件:
# - 多様なキックパワー設定 (0.2 ~ 1.0)
# - 異なるボール初期状態 (停止、転がり)
# - 最低20回以上のストレートキック
# - 各キック後3秒以上の軌道記録
```

### 2. キャリブレーション実行

#### 方法A: Launchファイルを使用 (推奨)

```bash
# 自動キャリブレーション（最新ROSBAGを自動検出）
ros2 launch crane_world_model_publisher ball_calibration.launch.py \
    auto_calibrate:=true \
    output_config_path:=/path/to/calibrated_params.yaml

# 特定のROSBAGを指定
ros2 launch crane_world_model_publisher ball_calibration.launch.py \
    rosbag_path:=/path/to/kick_calibration_data \
    auto_calibrate:=true \
    output_config_path:=/path/to/calibrated_params.yaml
```

#### 方法B: サービス呼び出し

```bash
# ノード起動
ros2 launch crane_world_model_publisher ball_calibration.launch.py \
    rosbag_path:=/path/to/kick_calibration_data

# サービス呼び出し
ros2 service call /calibrate_ball_physics std_srvs/srv/Trigger
```

### 3. 結果確認

キャリブレーション完了後、出力される設定ファイルを確認：

```yaml
# calibrated_ball_physics.yaml
ball_physics_model:
  deceleration: 0.485      # 最適化された減速度
  gravity: -9.81
  air_resistance: 0.0
  # ...

kicker_power_mapping:
  straight_kick:
    linear_coefficient: 2.45  # 最適化された線形係数
    offset: 0.05
    r_squared: 0.89          # 決定係数
    data_points_used: 25

calibration_info:
  physics_rmse: 0.08        # 物理モデルRMSE [m]
  physics_r_squared: 0.92   # 物理モデルR²
  timestamp: 1640995200     # キャリブレーション実行時刻
```

### 4. 可視化とデータ検証

キャリブレーション時に生成される可視化データを確認：

```bash
# 生成されたプロットを確認
ls -la kick_event_visualization_*.png

# ROS2コマンドで可視化実行
ros2 run crane_world_model_publisher plot_kick_events.py kick_event_visualization_0_data.json

# データ概要のみ表示
ros2 run crane_world_model_publisher plot_kick_events.py data.json --summary-only
```

**可視化データの見方：**
- **Ball Trajectory (XY)**: ボールの軌道（2次元）
- **Position vs Time**: 位置の時系列変化
- **Velocity vs Time**: 速度の時系列変化（キック瞬間の検出確認）
- **Ball Velocity Vectors**: 軌道上の速度ベクトル表示

### 5. パラメータ適用

最適化されたパラメータを本番システムに適用：

```bash
# BallPhysicsModelFactoryに新しい設定を適用
ros2 param load /world_model_publisher /path/to/calibrated_params.yaml

# システム再起動
ros2 launch crane_bringup crane.launch.py
```

## Vision生データの活用

### データソースの選択理由

キャリブレーション精度向上のため、**SSL-Visionの生データ**を直接使用します：

**フィルタ後データ vs Vision生データ：**

| 項目 | フィルタ後データ | Vision生データ |
|------|------------------|----------------|
| **精度** | EKF処理済み | カメラ直接検出値 |
| **ノイズ** | 低い | 多い |
| **遅延** | あり（フィルタ処理） | 最小 |
| **物理的妥当性** | 保証 | 生の観測値 |
| **キャリブレーション適性** | 不適切（循環依存） | **最適** |

### 技術実装

Vision生データの処理フロー：

```cpp
// BallInfo.msg の vision フィールドから生データ取得
const auto & ball_info = world_model_msg->ball_info;
if (ball_info.detected && ball_info.vision.stamp.sec != 0) {
  // 位置情報
  vision_ball.pos = Point(ball_info.vision.pos.x, ball_info.vision.pos.y);
  vision_ball.pos_z = ball_info.vision.pos.z;
  
  // 速度計算（時間微分）
  Point position_diff = vision_ball.pos - prev_ball.pos;
  double dt = (current_time - prev_time).seconds();
  vision_ball.vel = position_diff / dt;
}
```

### ノイズ対策

1. **テレポート検出**: SSL-Visionのオクルージョンによる位置ジャンプを検出・除外
2. **時系列フィルタリング**: 異常な速度変化を検出・補正
3. **物理的妥当性チェック**: 加速度制限による外れ値除去

## 品質評価基準

### 合格基準

- **位置予測RMSE**: < 0.2 m
- **速度予測誤差**: < 0.5 m/s  
- **物理モデルR²**: > 0.8
- **キッカーモデルR²**: > 0.7
- **データ点数**: > 15点

### 品質改善のヒント

1. **低い位置予測精度 (RMSE > 0.2m)**
   - より多くの高品質な軌道データを収集
   - SSL-Visionのキャリブレーション確認
   - フィールド表面の状態確認

2. **低いキッカーモデルR² (< 0.7)**
   - より広範囲のパワー設定でデータ収集
   - キッカー機構の機械的ばらつき確認
   - 複数回同じパワーでテスト実行

3. **データ不足**
   - 最低20回以上のキックデータ収集
   - 各軌道3秒以上の記録
   - 異なる条件での多様なキック

## トラブルシューティング

### エラー: "ROSBAGファイルが見つからない"

```bash
# ファイルパス確認
ls -la /path/to/rosbag/file

# 絶対パスを指定
ros2 launch crane_world_model_publisher ball_calibration.launch.py \
    rosbag_path:=/absolute/path/to/bag/file
```

### エラー: "有効なキックデータが見つからない"

ROSBAGに以下のトピックが含まれているか確認：

```bash
ros2 bag info /path/to/rosbag/file

# 必要なトピック:
# - /ball_info
# - /robot_command_*
```

### 最適化失敗

1. データ品質を確認：
   - キック後のボール軌道が3秒以上記録されているか
   - kick_powerが0より大きい値で設定されているか
   - ボール初期速度が0.5 m/s以上か

2. 設定調整：

   ```yaml
   extractor_config:
     min_kick_speed: 0.3        # より低い閾値に調整
     min_trajectory_points: 5   # より少ない点数に調整
   ```

## 高度な設定

### カスタム最適化パラメータ

```yaml
optimizer_config:
  min_deceleration: 0.1
  max_deceleration: 1.5
  convergence_threshold: 1.0e-6
  max_iterations: 200           # より多くの反復
  remove_outliers: true         # 外れ値除去有効化
```

### 特定ロボットのみキャリブレーション

```yaml
extractor_config:
  target_robot_ids: [0, 1, 2]   # ロボットID 0,1,2のみ
```

## パフォーマンス

- **データ抽出**: ~1-2秒 (1000メッセージ/GB)
- **最適化**: ~5-10秒 (20-50軌道)
- **検証**: ~1-2秒
- **総処理時間**: ~10-15秒 (典型的なデータセット)

## 制限事項

1. **ストレートキックのみ対応** (チップキックは将来対応予定)
2. **転がりボール物理のみ最適化** (飛行ボールは固定パラメータ)
3. **線形キッカーモデルのみ** (非線形関係は今後の改善点)
4. **単一環境での最適化** (フィールド表面変化への適応なし)
5. **Vision生データのノイズ影響** (十分なデータ点数で統計的に補正)

## FAQ

**Q: どのくらいの頻度でキャリブレーションすべきか？**
A: フィールド環境変更時、ロボット機構変更時、または予測精度低下時に実行を推奨。

**Q: キャリブレーション中にエラーが発生した場合は？**
A: ログを確認し、データ品質やファイルパスを確認してください。問題が解決しない場合は開発チームに相談。

**Q: 他のボール物理パラメータもキャリブレーション可能か？**
A: 現在は減速度のみ。重力や空気抵抗は将来のバージョンで対応予定。

**Q: Vision生データを使用することでどのような利点があるか？**
A: フィルタ処理による歪みがなく、真のボール軌道に基づいたキャリブレーションが可能。ただしノイズが多いため十分なデータ点数が必要。

**Q: テレポートイベントとは何か？**
A: SSL-Visionのオクルージョン（遮蔽）により、ボールが瞬間移動したかのように見える現象。自動検出・除外されます。

**Q: 可視化データの活用方法は？**
A: キック検出の妥当性確認、軌道の物理的妥当性検証、データ品質の視覚的評価に使用できます。
