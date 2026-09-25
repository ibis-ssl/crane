# ボールトラッキングシステム

## 目的

SSL-VisionおよびSSL-Vision-Tracker（または互換外部トラッカー）からの観測データを統合し、3D物理モデル（`BallPhysicsModel`）に基づいたボールの位置・速度・状態を推定し、`/world_model` トピックとしてシステム全体に配信します。

## アーキテクチャと優先順位規約

- **トラッキング優先順位契約**:
  1. **外部トラッカー (`TrackedFrame`)**: 最優先ソース。トラッカーの推定データを採用する。
  2. **Vision生データ (`DetectionFrame`)**: トラッカー途絶・未検出時のフォールバックとして補完。
- **データフロー**:
  - `SSL Vision / Tracker UDP` ──> `WorldModelDataProvider` ──> `WorldModelPublisher` ──> `/world_model`
- **ネットワーク規約**: アドレス・ポートは [`crane.launch.xml`](https://github.com/ibis-ssl/crane/blob/develop/crane_bringup/launch/crane.launch.xml) の `sim` と各引数を正本とする。

## 物理モデル規約

- **状態規約**: `STOPPED`、`ROLLING`、`FLYING` の判定条件と物理パラメータは [`BallPhysicsModel`](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/include/crane_physics/ball_physics_model.hpp) および設定ファイルを正本とする。

## 実装リファレンス

- 世界モデル配信ノード: [world_model_publisher.cpp](https://github.com/ibis-ssl/crane/blob/develop/crane_world_model_publisher/src/world_model_publisher.cpp)
- ボール物理モデル: [ball_info.hpp](https://github.com/ibis-ssl/crane/blob/develop/utility/crane_physics/include/crane_physics/ball_info.hpp)
- クライアントAPI: [WorldModelWrapper](./world_model_wrapper.md)
