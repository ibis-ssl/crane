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

- **WorldModelPublisherComponent**: 統合世界モデル配信ノード

### データ統合レイヤー（Stage 1リファクタリング完了）

- **DataSourceManager**: マルチソースデータ統合管理
  - Vision・Tracker・Feedback データの優先度ベース統合
  - 置き換え可能なデータソースアダプター
  - ゲーム設定による動的切り替え

- **WorldModelDataProvider**: 世界モデルデータ提供
  - 200行超の複雑な統合ロジックを40行に簡素化
  - データソース間の一貫性保証

### 知覚処理レイヤー（Stage 2リファクタリング完了）

- **VisionPacketReceiver**: UDP Vision パケット受信
  - マルチキャスト通信の専用処理
  - パケット品質管理とエラー処理

- **VisionDataConverter**: SSL Vision データ変換
  - プロトコルバッファから内部形式への変換
  - 座標系変換と単位統一

- **TrackerManagerFactory**: トラッカー管理ファクトリー
  - 依存性注入パターンによる独立化
  - BallTrackerManager・RobotTrackerManagerの統合管理

### トラッキングレイヤー

- **BallTracker**: EKFベースボール追跡
  - 6次元状態ベクトル [x,y,z,vx,vy,vz] による3D追跡
  - カルマンフィルター実装・予測更新サイクル
  - 物理モデル統合とマハラノビス距離外れ値検出

- **BallPhysicsModel**: 3D物理ボールモデル
  - 状態依存物理計算（STOPPED/ROLLING/FLYING）
  - 空気抵抗・重力・床面摩擦の統合シミュレーション
  - 高精度軌道予測（最大5秒先）

- **RobotTracker**: ロボット状態追跡
  - 位置・姿勢・速度の統合推定
  - Vision・フィードバックデータ融合

### 可視化システム（Stage 3リファクタリング完了）

- **VisualizationManager**: 統合可視化管理システム
  - 7つの散在したVisualizerを統一管理
  - 戦略パターンによる可視化レベル制御（簡易・標準・詳細）
  - トピック別可視化切り替えとパフォーマンス最適化

- **VisualizationBuilderRegistry**: ビルダー統一管理
  - リソース最適化と一元管理
  - 動的ビルダー生成・削除

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

## リファクタリング成果（2025年1月完了）

### 🎯 主要改善項目

**アーキテクチャの根本的改善**: 複雑に絡み合ったデータフローと責任分離の問題を4段階のリファクタリングで解決。単一責任原則の徹底とモジュール化により、保守性・拡張性・テスト容易性が大幅向上。

#### Stage 1: データソース統合の改善 ✅

- **複雑度削減**: `WorldModelDataProvider::getMsg()` 200行→40行（80%削減）
- **抽象化導入**: DataSourceManager・GameConfigurationによる設定と実装の分離
- **置き換え可能性**: アダプターパターンでVision/Tracker/Feedbackソースの動的切り替え実現

#### Stage 2: クラス責任の分離 ✅

- **VisionDataProcessor分割**: VisionPacketReceiver（UDP通信）+ VisionDataConverter（データ変換）
- **依存性注入**: TrackerManagerFactoryによるトラッカー独立化
- **単一責任原則**: 各クラスが明確に定義された単一の責任を持つ設計

#### Stage 3: 可視化システム統一 ✅

- **統合管理**: 7つの散在したVisualizerMessageBuilderをVisualizationManagerで一元管理
- **戦略パターン**: 可視化レベル（簡易/標準/詳細）の動的切り替え
- **パフォーマンス最適化**: トピック別ON/OFF、更新頻度制御、リソース最適化

#### Stage 4: ファイル構造最終整理 ✅

- **インターフェース明確化**: 各コンポーネントの役割と依存関係を明確に定義
- **コンパイル最適化**: 警告解消とビルド時間短縮
- **ドキュメント更新**: リファクタリング成果の反映

### 🚀 リファクタリング効果

#### コード品質向上

- **複雑度削減**: 主要メソッドの行数80%削減
- **責任分離**: 単一責任原則の徹底による保守性向上
- **テスト容易性**: 依存性注入による単体テスト実装の簡素化

#### パフォーマンス最適化

- **可視化制御**: 不要な描画処理の除去による計算負荷削減
- **リソース管理**: ビルダーの統一管理によるメモリ効率化
- **実行時切り替え**: ランタイムでの可視化レベル動的変更

#### 拡張性確保

- **プラグイン対応**: 新しいデータソースの容易な追加
- **設定駆動**: GameConfigurationによる柔軟な動作制御
- **戦略パターン**: 可視化・データ統合戦略の容易な変更

### 実戦検証

- **🔥 ボールフィルタ**: 高精度状態推定システムが実戦で安定動作を確認（#881実装済み）
- **🔥 3D物理モデル**: ボール軌道予測精度が大幅向上し競技で有効性を実証（#872完了）  
- **Vector3D対応**: 3次元座標系での完全対応により空中ボール処理が向上（#880適用済み）
- **姿勢推定**: より正確な姿勢推定によりロボット制御精度が改善（#859実装完了）

### 開発活発度

🔴 **高活動**: Craneシステムで最も活発に開発されているコンポーネント。リファクタリングにより技術的負債を解消し、今後の機能拡張基盤を確立。

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
