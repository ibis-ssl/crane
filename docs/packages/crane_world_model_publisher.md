# crane_world_model_publisher

## 概要

**crane_world_model_publisher**は、Craneロボットサッカーシステムの中核となる知覚・状態推定コンポーネントです。SSL Visionカメラからの生データおよびSSL-Vision-Trackerからのトラッキングデータを処理し、ボール位置、ロボット位置、フィールド幾何学を含む統一世界モデルを生成する中央ハブとして機能します。

## 主要機能

- **マルチソースVision処理**: SSL-Vision生データと外部トラッカー(TrackedFrame)の統合
- **3Dボール物理モデル**: 転がり摩擦・重力を考慮した高精度ボール軌道予測（BallPhysicsModel）
- **ロボット状態推定**: 位置・姿勢・速度の統合推定
- **統合世界モデル配信**: システム全体への状態情報提供
- **ゲーム状況判定**: パス先選定やキックイベントの検出

## アーキテクチャ上の役割

Craneシステムの**認識層の中核**として、生のセンサーデータから構造化された世界状態を生成し、上位の戦略・制御層に情報を提供します。

## コンポーネント構成

### メインノード

- **WorldModelPublisherComponent**: 統合世界モデル配信ノード。16ms周期で世界モデルをパブリッシュします。

### データ管理・通信

- **WorldModelDataProvider**: 統合データ管理・配信
  - Vision UDP（10006/10020）およびTracker UDP（10010）の非ブロッキング受信
  - ROS 2サブスクリプション管理（/referee, /robot_feedback, /play_situation）
  - チーム設定・ゲーム状態・フィールドジオメトリ管理
  - データの正規化（TrackedFrameを優先）

### 物理・分析レイヤー

- **BallPhysicsModel**: 3D物理ボールモデル
  - 状態依存物理計算（STOPPED/ROLLING/FLYING）
  - 転がり摩擦・重力・空気抵抗の統合シミュレーション
  - YAML設定によるパラメータカスタマイズ

- **KickEventDetector**: キックイベントの検出
  - ロボットとボールの接触・キック動作を検出し、GameAnalysisメッセージとして出力

- **PassTargetSelector**: パス先候補の評価と選択
  - ロボット配置や敵の妨害を考慮したパススコアの算出

### 可視化システム

- **VisualizationManager**: 統合可視化管理システム
  - フィールドジオメトリ、追跡オブジェクト、軌跡履歴、スラック時間などの可視化を一括管理

## 依存関係

### コア依存

- **crane_geometry**: 幾何学計算ライブラリ
- **crane_physics**: 物理計算・ボールモデル
- **crane_msg_wrappers**: メッセージ変換ユーティリティ
- **crane_msgs**: システムメッセージ定義
- **robocup_ssl_msgs**: SSL公式プロトコルメッセージ

### システム依存

- **eigen**: 線形代数計算
- **yaml-cpp**: 設定ファイル処理
- **diagnostic_updater**: システム診断

## 使用方法

### 基本起動

```bash
# 単体起動
ros2 run crane_world_model_publisher world_model_publisher_node

# crane_bringup経由での起動（推奨）
ros2 launch crane_bringup crane.launch.xml
```

### パラメータ設定

```yaml
world_model_publisher:
  ros__parameters:
    # ネットワーク設定
    vision_address: "224.5.23.2"
    vision_port: 10020
    tracker_address: "224.5.23.2"
    tracker_port: 10010
    use_udp_detection: false

    # チーム設定
    team_name: "ibis-ssl"
    initial_team_color: "BLUE"
    is_emplace_positive_side: true

    # 物理モデル設定
    ball_physics_config_path: "ball_physics.yaml"
```

## 診断機能

`vision/processing` 項目で以下の情報を監視します：

- Vision/Trackerデータの受信状態（1秒以上の無受信でERROR）
- ボール検出状態
- 検出ロボット数

---

**関連パッケージ**: [crane_geometry](./crane_geometry.md) | [crane_physics](./crane_physics.md) | [crane_game_analyzer](./crane_game_analyzer.md)
