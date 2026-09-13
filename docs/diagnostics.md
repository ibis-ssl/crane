# 診断システム

ROS 2標準の diagnostics を使い、計算PCとロボットの状態を監視します。診断結果は `/diagnostics_agg` に集約され、異常な機体を計画から除外するために使われます。

## 可用性と循環参照

診断の入力に診断結果を使うと、診断結果が可用性を変え、その可用性が診断入力を変える循環が生じます。入力ソースと診断結果を別の状態として保持してください。

```text
Vision/Tracker/feedback ──> available_vision, available_tracker, available_feedback
                                  │
                                  ▼
                          Diagnostic Publisher
                                  │
                                  ▼
                         /diagnostics_agg
                                  │
                                  ▼
                         available_hardware
```

- `available()`: `(available_vision || available_tracker) && available_hardware`
- `availableStrict()`: `available_vision && available_hardware && available_feedback`
- `availableLoose()`: `available_vision || available_tracker`

`available_hardware` は診断で ERROR が記録されたときだけ false になります。WARN は計画から機体を除外せず、OK でエラー状態を解除します。

## 監視対象

- 計算PC: Vision、AI planner、local planner、主要トピックの配信状態
- ロボット: `robot_{:02d}/communication`、`battery`、`robot_error`
- 詳細な判定値・エラー名・集約階層は実装と設定ファイルを正本とします。

## 確認

```bash
ros2 topic echo /diagnostics_agg
ros2 run rqt_robot_monitor rqt_robot_monitor
```

実機での異常は、Craneの指令生成、UDP送信、ロボット側中継、フィードバック受信の順に切り分けます。

## 実装リファレンス

- [diagnostic_publisher.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_receiver/src/diagnostic_publisher.cpp)
- [robot_info.hpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/include/crane_physics/robot_info.hpp)
- [world_model_wrapper.cpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_msg_wrappers/src/world_model_wrapper.cpp)
- [diagnosed_publisher.hpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_comm/include/crane_comm/diagnosed_publisher.hpp)
- [diagnostic_aggregator.yaml](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/config/diagnostic_aggregator.yaml)
