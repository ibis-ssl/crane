# crane_world_model_publisher

## 概要

**crane_world_model_publisher**は、Craneロボットサッカーシステムの中核となる知覚・状態推定コンポーネントです。SSL Visionカメラからの生データを処理し、ボール位置、ロボット位置、フィールド幾何学を含む統一世界モデルを生成する中央ハブとして機能します。EKFベースの高精度ボールトラッキング、3D物理モデル、リアルタイム処理により、全ての高次自律動作の基盤となる信頼性の高い世界状態情報を提供します。

## 主要機能

- **SSL Vision処理**: カメラデータからの物体認識・位置推定
- **3Dボール物理モデル**: 空気抵抗・重力を考慮した高精度ボール軌道予測
- **ボールトラッキング**: 状態依存（STOPPED/ROLLING/FLYING）の追跡システム
- **ロボット状態推定**: 位置・姿勢・速度の統合推定
- **統合世界モデル配信**: システム全体への状態情報提供

## アーキテクチャ上の役割

Craneシステムの**認識層の中核**として、生のセンサーデータから構造化された世界状態を生成し、上位の戦略・制御層に情報を提供します。リアルタイム制約下で高精度な状態推定を実現する重要コンポーネントです。

## コンポーネント構成

### メインノード

- **WorldModelPublisherNode**: 統合世界モデル配信ノード

### コアクラス

- **BallPhysicsModel**: 3D物理ボールモデル
  - 放物運動シミュレーション
  - 空気抵抗・重力計算
  - 状態遷移管理（停止→転がり→飛行）

- **BallTracker**: ボール追跡システム
  - カルマンフィルター実装
  - 予測・更新サイクル
  - 状態依存追跡

- **VisionDataProcessor**: Vision データ前処理
  - カメラデータ統合
  - ノイズフィルタリング
  - 座標系変換

- **TrackerDataProcessor**: 追跡データ処理
  - トラッキング結果統合
  - 状態履歴管理

- **VisualizationDataHandler**: 可視化データ生成
  - デバッグ情報配信
  - 軌道予測表示

- **WorldModelDataProvider**: 世界モデルデータ提供
  - 統合データ生成
  - メッセージ配信管理

## ボール物理モデル詳細

### 状態定義

```cpp
enum BallState {
  STOPPED,   // 静止状態（摩擦による完全停止）
  ROLLING,   // 転がり状態（地面接触・摩擦あり）
  FLYING     // 飛行状態（3D放物運動・空気抵抗）
};
```

### 3D物理方程式

- **重力**: `g = 9.81 m/s²`
- **空気抵抗**: `F_drag = -k * v * |v|`（速度二乗比例）
- **地面摩擦**: `F_friction = μ * N`（転がり摩擦）

### 予測アルゴリズム

```cpp
// 飛行状態での3D軌道予測
Vector3 predictPosition(double dt) {
  // 重力・空気抵抗を考慮した数値積分
  // 地面衝突検出・バウンド計算
  // 状態遷移判定
}
```

## 依存関係

### コア依存

- **crane_geometry**: 幾何学計算ライブラリ
- **crane_physics**: 物理計算・ボールモデル
- **crane_msg_wrappers**: メッセージ変換ユーティリティ
- **crane_msgs**: システムメッセージ定義
- **robocup_ssl_msgs**: SSL公式プロトコルメッセージ

### システム依存

- **boost**: 高性能C++ライブラリ
- **eigen**: 線形代数ライブラリ（一部使用）
- **libgoogle-glog-dev**: ログ出力
- **diagnostic_updater**: システム診断

## 使用方法

### 基本起動

```bash
# 単体起動
ros2 run crane_world_model_publisher world_model_publisher_node

# データパイプライン起動
ros2 launch crane_bringup data.launch.py
```

### トピック構成

```yaml
# 購読トピック
/vision: robocup_ssl_msgs/msg/SSL_WrapperPacket
/referee: robocup_ssl_msgs/msg/Referee

# 配信トピック  
/world_model: crane_msgs/msg/WorldModel
/ball_info: crane_msgs/msg/BallInfo
/robot_info: crane_msgs/msg/RobotInfo
```

### パラメータ設定例

```yaml
world_model_publisher:
  ros__parameters:
    ball_physics:
      gravity: 9.81
      air_resistance: 0.01
      bounce_factor: 0.7
    tracking:
      position_noise: 0.01
      velocity_noise: 0.1
```

## 最近の開発状況

### JapanOpen2025での実戦検証

- **🔥 ボールフィルタ**: 高精度状態推定システムが実戦で安定動作を確認（#881実装済み）
- **🔥 3D物理モデル**: ボール軌道予測精度が大幅向上し競技で有効性を実証（#872完了）  
- **Vector3D対応**: 3次元座標系での完全対応により空中ボール処理が向上（#880適用済み）
- **姿勢推定**: より正確な姿勢推定によりロボット制御精度が改善（#859実装完了）

### 開発活発度

🔴 **高活動**: Craneシステムで最も活発に開発されているコンポーネントの一つ。ボール物理モデルの改良、フィルタリング精度向上、リアルタイム性能最適化が継続的に行われている。

### 最新技術導入

- **状態依存フィルタ**: ボール状態に応じた適応的フィルタリング
- **予測軌道最適化**: より精確な軌道予測アルゴリズム
- **多カメラ統合**: 複数カメラからの情報統合技術

## パフォーマンス特性

### リアルタイム制約

- **更新頻度**: 60Hz（SSL Vision同期）
- **予測範囲**: 最大5秒先までの軌道予測
- **レイテンシ**: <10ms（認識→配信）

### 精度特性

- **位置精度**: ±1cm（静止物体）
- **速度精度**: ±0.1m/s（移動物体）
- **予測精度**: 1秒後±5cm（飛行ボール）

## トラブルシューティング

### よくある問題

1. **Vision信号なし**: カメラ接続・ネットワーク確認
2. **トラッキングロスト**: パラメータ調整・環境光確認
3. **予測精度低下**: 物理パラメータ再キャリブレーション

---

**関連パッケージ**: [crane_geometry](./crane_geometry.md) | [crane_physics](./crane_physics.md) | [crane_game_analyzer](./crane_game_analyzer.md) | [robocup_ssl_comm](./robocup_ssl_comm.md)
