# ボール所有権判定（非推奨）

⚠️ **このドキュメントは非推奨です**

ボール所有権の判定ロジックは`crane_world_model_publisher`パッケージに移行されました。
現在の実装については以下を参照してください：

- **メインドキュメント**: [crane_world_model_publisher](./packages/crane_world_model_publisher.md)
- **詳細な技術仕様**: [ball_tracking_system.md](./ball_tracking_system.md)
- **ソースコード**: `crane_world_model_publisher/src/world_model_publisher.cpp`

## 移行された機能

以下の機能が`crane_world_model_publisher`に統合されています：

- ボール所有権の判定ロジック
- ロボット-ボール間距離の計算
- ドリブル状態の検出
- ボール接触の判定

最新の情報については上記のドキュメントを参照してください。
