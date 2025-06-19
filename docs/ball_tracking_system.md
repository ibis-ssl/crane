# ボールトラッキングシステム設計書

## 概要

このドキュメントでは、crane_world_model_publisherパッケージにおけるボールトラッキングシステムの設計と実装について説明します。本システムは、SSL（Small Size League）visionパケットからボール情報を取得し、EKF（Extended Kalman Filter）ベースの状態推定を行い、物理モデルを活用した予測機能を提供します。

## アーキテクチャ概要

### データフロー

```mermaid
SSL Vision UDP → VisionDataProcessor → BallTracker → world_model topic → WorldModelWrapper → Ball
                       ↓
                 BallPhysicsModel（独立コンポーネント）
```

### 主要コンポーネント

1. **VisionDataProcessor**: Visionパケットの受信とデータ抽出
2. **BallTracker**: EKFベースの状態推定
3. **BallTrackerManager**: 複数トラッカーの管理
4. **BallPhysicsModel**: 共有物理計算モデル
5. **Ball**: 予測ユーティリティの提供

## コンポーネント詳細

### 1. VisionDataProcessor

**責任**: SSL visionパケットの受信とBallTrackerへのデータ受け渡し

**主要機能**:

- UDPマルチキャスト受信（デフォルト: 224.5.23.2:10020）
- ボール位置データの抽出・変換（mm → m）
- BallTrackerManagerへのデータ委譲
- ジオメトリ情報の処理

**キーメソッド**:

```cpp
auto processVisionPackets() -> void;
auto getBallInfo() const -> crane_msgs::msg::BallInfo;
auto hasVisionUpdated() const -> bool;
```

### 2. BallTracker

**責任**: 単一ボールのEKFベース状態推定

**状態ベクトル**: [x, y, z, vx, vy, vz]^T (6次元)

**主要機能**:

- EKF予測ステップ（状態遷移行列使用）
- EKF更新ステップ（観測データ統合）
- マハラノビス距離による外れ値検出
- ボール状態遷移（STOPPED/ROLLING/FLYING）

**物理モデル統合**:

```cpp
auto predict(double dt) -> void;
auto update(const Eigen::Vector3d & measurement, Ball::State observed_state) -> void;
auto getMahalanobisDistance(const Eigen::Vector3d & measurement) const -> double;
```

### 3. BallTrackerManager

**責任**: 複数ボール仮説の管理とデータ関連付け

**主要機能**:

- 新規トラッカー生成（外れ値検出時）
- 最適トラッカー選択（信頼度ベース）
- 古いトラッカーの削除
- 統一されたボール情報出力

**設定パラメータ**:

- `OUTLIER_THRESHOLD`: 9.0（マハラノビス距離）
- `MIN_TRACKING_CONFIDENCE`: 0.3

### 4. BallPhysicsModel

**責任**: 物理計算の共有とEKF行列提供

**主要機能**:

- 状態遷移行列計算
- 制御入力計算（重力効果）
- 状態推定（位置・速度から状態判定）
- 予測計算（後方互換性維持）

**設定パラメータ**:

```cpp
struct Config {
  double deceleration = 0.5;        // 転がり減速度 (m/s²)
  double gravity = -9.81;           // 重力加速度 (m/s²)
  double air_resistance = 0.0;     // 空気抵抗係数
  double height_threshold = 0.05;  // 飛行判定閾値 (m)
  double speed_threshold = 0.1;    // 移動判定閾値 (m/s)
  double stop_threshold = 0.05;    // 停止判定閾値 (m/s)
};
```

### 5. Ball構造体

**責任**: 物理ベース予測ユーティリティの提供

**主要機能**:

- 状態対応位置予測
- 状態対応速度予測
- 軌道生成
- 停止時間・最大距離計算

**状態定義**:

```cpp
enum class State {
  STOPPED,  // 停止
  ROLLING,  // 転がり
  FLYING,   // 飛行
};
```

## EKF実装詳細

### 状態遷移モデル

**ROLLING状態**:

```text
F = [1  0  0  dt 0  0 ]
    [0  1  0  0  dt 0 ]
    [0  0  1  0  0  dt]
    [0  0  0  e^(-k*dt)  0  0 ]
    [0  0  0  0  e^(-k*dt)  0 ]
    [0  0  0  0  0  0 ]
```

**FLYING状態**:

```text
F = [1  0  0  dt 0  0 ]
    [0  1  0  0  dt 0 ]
    [0  0  1  0  0  dt]
    [0  0  0  1  0  0 ]
    [0  0  0  0  1  0 ]
    [0  0  0  0  0  1 ]

制御入力: u = [0, 0, 0.5*g*dt², 0, 0, g*dt]^T
```

### 観測モデル

位置のみ観測:

```text
H = [1  0  0  0  0  0]
    [0  1  0  0  0  0]
    [0  0  1  0  0  0]
```

### ノイズモデル

**プロセスノイズ**: Q = diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1]) * dt
**観測ノイズ**: R = diag([0.001, 0.001, 0.001])

## 状態遷移ロジック

### 自動状態遷移

1. **FLYING → ROLLING**: z ≤ 0.0
2. **ROLLING → STOPPED**: speed < stop_threshold
3. **STOPPED → ROLLING**: speed > speed_threshold

### 観測ベース状態推定

```cpp
auto estimateStateFromMeasurement(position, velocity) -> Ball::State {
  if (height > height_threshold) return FLYING;
  if (speed > speed_threshold) return ROLLING;
  return STOPPED;
}
```

## 物理計算詳細

### 転がり物理

**減速モデル**: v(t) = v₀ *e^(-k*t)
**位置予測**: x(t) = x₀ + ∫v(τ)dτ
**停止時間**: t_stop = v₀ / k

### 飛行物理

**放物運動**:

- x(t) = x₀ + vₓ₀ * t
- y(t) = y₀ + vᵧ₀ * t  
- z(t) = z₀ + vᵤ₀ *t + 0.5* g * t²

## パフォーマンス最適化

### 計算効率

1. **予測ステップ**: 10ms周期で実行
2. **マトリックス計算**: Eigenライブラリ使用
3. **メモリ管理**: shared_ptrで物理モデル共有

### 信頼性向上

1. **外れ値処理**: マハラノビス距離による自動検出
2. **多重仮説**: 複数トラッカーによるロバスト性
3. **状態制約**: 物理的制約の適用

## 設定とチューニング

### 主要パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|------------|------|
| vision_address | "224.5.23.2" | visionマルチキャストアドレス |
| vision_port | 10020 | visionポート番号 |
| OUTLIER_THRESHOLD | 9.0 | 外れ値判定閾値 |
| MIN_TRACKING_CONFIDENCE | 0.3 | 最小信頼度 |
| deceleration | 0.5 | 転がり減速度 |
| height_threshold | 0.05 | 飛行判定高度 |

### チューニング指針

1. **精度向上**: プロセスノイズ・観測ノイズの調整
2. **応答性**: 外れ値閾値の調整
3. **安定性**: 最小信頼度の調整

## トラブルシューティング

### よくある問題

1. **ボール見失い**:
   - 信頼度閾値の確認
   - vision更新頻度の確認

2. **状態遷移エラー**:
   - 物理パラメータの確認
   - 閾値設定の見直し

3. **外れ値誤検出**:
   - マハラノビス距離閾値の調整
   - ノイズモデルの見直し

### デバッグ方法

1. **ログ出力**: `getTrackingConfidence()`で信頼度監視
2. **可視化**: rvizでボール軌道表示
3. **統計**: `getMahalanobisDistance()`で距離監視

## 今後の拡張

### 予定されている改善

1. **Vision→Tracker処理**: 外部トラッカー依存の除去
2. **機械学習統合**: 深層学習ベース状態推定
3. **多ボール対応**: 複数ボール同時トラッキング

### API拡張

1. **予測精度向上**: より詳細な物理モデル
2. **リアルタイム調整**: 動的パラメータチューニング
3. **統計情報**: トラッキング品質メトリクス

## まとめ

本ボールトラッキングシステムは、EKFベースの堅牢な状態推定と物理モデルベースの予測を組み合わせることで、高精度かつリアルタイムなボール情報を提供します。モジュラー設計により、各コンポーネントが独立して機能し、将来の拡張にも対応可能な構造となっています。
