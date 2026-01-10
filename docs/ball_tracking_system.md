# ボールトラッキングシステム設計書

## 概要

このドキュメントでは、crane_world_model_publisherパッケージにおけるボールトラッキングシステムの設計と実装について説明します。本システムは、SSL-Vision-Tracker（または互換性のある外部トラッカー）からのTrackedFrameパケット、およびSSL-Vision生パケットからボール情報を取得し、高精度な世界状態を配信します。

## アーキテクチャ概要

### データフロー

```mermaid
SSL Vision / Tracker UDP → WorldModelDataProvider → WorldModelPublisher → /world_model topic
                                  ↓
                            BallPhysicsModel（物理特性・予測）
```

### 主要コンポーネント

1. **WorldModelDataProvider**: UDPパケット（Vision/Tracker）の受信と正規化。外部トラッカーからの高精度データとVision生データを統合します。
2. **WorldModelPublisher**: 統合された情報をWorldModelメッセージとしてパブリッシュします。
3. **BallPhysicsModel**: ボールの物理パラメータ管理と軌道予測。
4. **WorldModelWrapper**: 配信された情報を利用するクライアント側のラッパー。予測ユーティリティを提供します。

## コンポーネント詳細

### 1. WorldModelDataProvider

**責任**: 複数ソース（Vision UDP, Tracker UDP, RobotFeedback）からのデータ受信と統合。

**主要機能**:

- **マルチキャスト受信**:
  - Vision UDP（実機: 10006, シミュレーション: 10020）
  - Tracker UDP（既定: 10010）
- **データの正規化**: 外部トラッカーからの `TrackedFrame` を最優先ソースとし、未検出時はVision生データで補完。
- **チーム・ゲーム状態管理**: Referee信号に基づくチームカラー・サイド・ゲーム状況の管理。

### 2. BallPhysicsModel

**責任**: 物理計算の共有と予測計算。

**主要機能**:

- **物理パラメータ管理**: 転がり減速度、重力、空気抵抗、各状態遷移閾値の保持。
- **状態判定**: 位置・速度情報からの状態（STOPPED/ROLLING/FLYING）判定。
- **予測計算**: 現在の状態と物理モデルに基づく未来位置・速度の予測。

**設定パラメータ**:

```cpp
struct Config {
  double deceleration = 0.7;       // 転がり減速度 (m/s²)
  double gravity = -9.81;          // 重力加速度 (m/s²)
  double air_resistance = 0.0;     // 空気抵抗係数
  double height_threshold = 0.05;  // 飛行判定高度 (m)
  double speed_threshold = 0.1;    // 移動判定閾値 (m/s)
  double stop_threshold = 0.05;    // 停止判定閾値 (m/s)
};
```

## トラッキングの優先順位

1. **外部トラッカー (`TrackedFrame`)**: EKF等の高度なフィルタリングが施された外部システムからのデータを最優先します。
2. **Vision生データ (`DetectionFrame`)**: トラッカーが利用できない場合、Visionからの生の位置情報を使用します。

## 物理計算詳細

### 転がり物理

**減速モデル**: $v(t) = v_0 - a \cdot t$ (または指数減衰モデル)
**位置予測**: $x(t) = x_0 + v_0 \cdot t - 0.5 \cdot a \cdot t^2$

### 飛行物理

**放物運動**:

- $x(t) = x_0 + v_{x0} \cdot t$
- $y(t) = y_0 + v_{y0} \cdot t$
- $z(t) = z_0 + v_{z0} \cdot t + 0.5 \cdot g \cdot t^2$

## パフォーマンス最適化

1. **UDP非ブロッキング受信**: 10ms周期のタイマーでUDPパケットを即座に処理。
2. **予測計算のオンデマンド実行**: クライアント側（WorldModelWrapper）で必要な時のみ物理モデルを使用して予測。

## トラブルシューティング

### よくある問題

1. **ボール見失い**:
   - ネットワーク設定（マルチキャストアドレス・ポート）の確認。
   - 外部トラッカーの稼働状態の確認。

2. **予測精度低下**:
   - 物理パラメータ（deceleration等）の確認。
   - `ball_model_calibration_guide.md` に基づく再キャリブレーションの実施。

## まとめ

外部トラッカーの高精度な情報と、内製の物理モデルを統合することで、安定した世界モデルを提供します。
