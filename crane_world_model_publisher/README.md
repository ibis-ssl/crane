# crane_world_model_publisher

## 役割

SSL-VisionおよびSSL-Vision-TrackerからのUDPパケットを受信・正規化し、ロボットおよびボールの3D物理モデルに基づく状態推定を行い、統合世界モデル（`/world_model`）を配信する知覚層の中核パッケージです。

## 固有の制約

- ボールは外部トラッカー（`TrackedFrame`）を最優先ソースとし、未検出時にVision生データで補完する。ロボット検出の経路は実装を参照すること。
- ボール物理モデルのキャリブレーション手順はトピックガイドを参照すること。

## トピックガイド

- [ボールトラッキングシステム設計書](../docs/ball_tracking_system.md)
- [ボール・キッカーの校正](../docs/ball_model_calibration_guide.md)

## リンク

- ソースディレクトリ: [crane_world_model_publisher](https://github.com/ibis-ssl/crane/tree/develop/crane_world_model_publisher)
