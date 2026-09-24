# crane_robot_receiver

## 役割

実機ロボットやシミュレータからのフィードバック（RobotFeedback）を受信し、Ping応答、バッテリー電圧、ハードウェアエラーの健全性監視および診断情報を配信するパッケージです。

## 固有の制約

- **循環参照回避**: 診断入力には `available_vision` 等のみを参照し、診断結果の `available_hardware` に依存させないこと。
- 各ロボットの診断名は `robot_{:02d}/communication`, `robot_{:02d}/battery`, `robot_{:02d}/robot_error` の2桁表記とすること。
- エラー表示は記録から10秒を超えると非表示になり、診断がOKへ戻った項目はエラーマップから削除される。

## トピックガイド

- [Crane Diagnostics System](../docs/diagnostics.md)

## リンク

- ソースディレクトリ: [crane_robot_receiver](https://github.com/ibis-ssl/crane/tree/develop/crane_robot_receiver)
- エラー定義: [robot_errors.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_receiver/include/crane_robot_receiver/robot_errors.hpp)
