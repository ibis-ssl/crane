# ボール・キッカーの校正

## 収集と推定

CraneとVisionを起動し、キック開始前から記録します。現行の抽出器が読むのは世界モデルとロボット指令です。

```bash
ros2 bag record -s mcap -o kick_calibration_data /world_model /robot_commands
```

手動キック、または次の収集セッションを使います。収集後に録画を停止します。

```bash
ros2 topic pub --once /session_injection std_msgs/msg/String \
  '{data: "BALL_CALIBRATION_DATA_COLLECTION"}'
```

出力先を明示して校正を実行します。`ball_calibration_analysis/` が既にある場合は再利用されるため、別の収集結果と混同しないでください。

```bash
ros2 launch crane_world_model_publisher ball_calibration.launch.py \
  rosbag_path:=/absolute/path/to/kick_calibration_data \
  output_config_path:=/absolute/path/to/calibrated_ball_physics.yaml \
  auto_calibrate:=true
```

結果の軌道数、RMSE、R²と元の軌道を確認します。採否条件と抽出の詳細は[校正実装](https://github.com/ibis-ssl/crane/tree/develop/crane_world_model_publisher/src/calibration)を正本とします。

## 適用と確認

ボール物理設定は起動引数で指定します。

```bash
ros2 launch crane_bringup crane.launch.xml \
  ball_physics_config_path:=/absolute/path/to/calibrated_ball_physics.yaml
```

起動ログで指定ファイルの読込成功を確認し、別途収集したキックの実軌道と予測を比較します。読込失敗時はデフォルトへフォールバックするため、起動しただけでは適用確認になりません。

生成YAMLの `kicker_power_mapping` は分析結果であり、`kicker_physics.yaml` が要求する `kicker_model` 形式ではありません。丸ごと上書きせず、実測結果を確認して既存設定の対応する校正配列へ反映します。設定変更後はワークスペースルートで対象パッケージを再ビルドし、環境を読み直してCraneを再起動します。

`CENTER_STOP_KICK` はデフォルトモデルを内部生成するため、現状では校正YAMLの適用検証には使えません。

## 実装リファレンス

- [ball_calibration.launch.py](https://github.com/ibis-ssl/crane/blob/develop/crane_world_model_publisher/launch/ball_calibration.launch.py)
- [ball_calibration_node.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_world_model_publisher/src/calibration/ball_calibration_node.cpp)
- [kicker_physics.yaml](https://github.com/ibis-ssl/crane/blob/develop/crane_world_model_publisher/config/kicker_physics.yaml)
- [ボールトラッキングシステム](./ball_tracking_system.md)
