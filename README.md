# Crane - RoboCup Small Size League (SSL) AI Framework

[ibis-ssl](https://ibis-ssl.github.io/ibis_documentation/)による高性能な自律ロボットサッカーAIシステム

## 概要

CraneはROS 2 Jazzyベースの自律ロボティクスシステムで、RoboCup Small Size League (SSL)競技向けに設計されています。小型自律ロボットチームのサッカー試合を制御するAIフレームワークです。

### 主な特徴

- **ROS 2 Jazzy**ベースのモジュラーアーキテクチャ
- **ModernORCAPlanner**による高性能な経路計画
- **リアルタイム制約システム**でのマルチロボット協調
- **SSL Vision/Referee**プロトコル完全対応
- **grSim**シミュレーション環境統合

## アーキテクチャ

### コアコンポーネント

- **crane_session_controller**: メイン試合制御とゲーム状態管理
- **crane_planner_plugins**: 戦略プランナー（攻撃、守備、フォーメーション）
- **crane_robot_skills**: 個別ロボット行動（ゴーリー、アタッカー、ディフェンダー）
- **crane_local_planner**: リアルタイム経路計画とModernORCAPlanner
- **crane_world_model_publisher**: ワールド状態推定とボール・ロボット追跡
- **crane_play_switcher**: ゲーム状況分析と自動プレー選択

### ModernORCAPlanner

RVO2Plannerの後継として開発された高性能局所プランナー：

#### 主要機能
- **ORCA（Optimal Reciprocal Collision Avoidance）**アルゴリズム
- **SSL制約システム**（ボール回避、ペナルティエリア、レフェリーコマンド）
- **マルチエージェント衝突回避**（味方・敵ロボット統合）
- **高精度位置制御**（台形速度プロファイル、加速度制限）
- **パフォーマンス監視**（リアルタイム性能測定）

#### 技術仕様
```cpp
// 主要パラメータ
double ORCA_TIME_STEP = 0.1;        // 時間ステップ
double ORCA_TIME_HORIZON = 2.0;     // 時間地平線
double ORCA_MAX_SPEED = 4.0;        // 最大速度
double ORCA_RADIUS = 0.05;          // ベース半径

// パフォーマンス監視
solve_time_ms_;         // ORCA解決時間（ms）
total_constraints_;     // 総制約数
```

#### RVO2との性能比較
- **処理速度**: ModernORCA ~0.03ms vs RVO2 ~0.05ms（約40%高速化）
- **制約システム**: より柔軟なSSL制約管理
- **メモリ効率**: 最適化されたエージェント管理

## セットアップ

### 必要要件

- **OS**: Ubuntu 24.04 LTS
- **ROS**: ROS 2 Jazzy
- **C++**: C++20対応コンパイラ
- **依存**: TBB（Parallel STL）、Boost Geometry

### ビルド

```bash
# 依存関係のインストール
vcs import src < src/crane/dependency_jazzy.repos
rosdep install -riy --from-paths src

# 開発ビルド
colcon build --symlink-install

# リリースビルド
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 環境設定
source install/local_setup.bash
```

### テスト

```bash
# 全テスト実行
colcon test --event-handlers console_cohesion+

# 特定パッケージのテスト
colcon test --packages-select crane_local_planner --event-handlers console_cohesion+

# ModernORCAPlannerテスト
colcon test --packages-select crane_local_planner --ctest-args -R test_modern_orca_planner
```

## 使用方法

### シミュレーション起動

```bash
# メインシステム（シミュレーション）
ros2 launch crane_bringup crane.launch.py sim:=true

# Dockerシミュレーション環境
cd docker/sim
docker compose up -d
```

### 実機環境

```bash
# 実機システム
ros2 launch crane_bringup crane.launch.py sim:=false

# Docker実機環境
cd docker/real
docker compose up -d
```

### 設定とデバッグ

```bash
# ModernORCAプランナーのデバッグ可視化有効化
ros2 param set /local_planner debug_visualize_constraints true
ros2 param set /local_planner debug_visualize_orca_lines true
ros2 param set /local_planner debug_show_performance_metrics true
```

## 開発

### コード規約

- **C++20**標準準拠
- **ament_cmake_auto**による自動CMake設定
- **事前コンパイルヘッダー**による高速ビルド
- **crane_lint_common**による統一リンティング

### テスト駆動開発

- **GTest**によるユニットテスト
- **Python RCST**による統合テスト
- **scenario_test/**でのシナリオテスト
- **継続的インテグレーション**でのテスト自動実行

### パッケージ依存関係

```
メッセージ層: crane_msgs, robocup_ssl_msgs
ユーティリティ層: crane_basics, crane_msg_wrappers  
コンポーネント層: crane_world_model_publisher, crane_robot_skills
プランニング層: crane_session_controller, crane_local_planner
統合層: crane_bringup, crane_sender
```

## パフォーマンス

### リアルタイム制約
- **ロボット制御**: 16ms周期での制御ループ
- **ボール物理**: 高精度予測モデル
- **通信遅延**: SSL protocol準拠のネットワーク通信

### 最適化機能
- **マルチロボット協調**: RVO2/ORCAアルゴリズムベースの衝突回避
- **動的制約**: ゲーム状況に応じた制約切り替え
- **パフォーマンス監視**: リアルタイム性能測定と可視化

## ライセンス

MIT License - 詳細は[LICENSE](LICENSE)ファイルを参照

## 貢献

1. このリポジトリをフォーク
2. フィーチャーブランチを作成 (`git checkout -b feature/amazing-feature`)
3. 変更をコミット (`git commit -m 'Add amazing feature'`)
4. ブランチにプッシュ (`git push origin feature/amazing-feature`)
5. プルリクエストを作成

## 関連リンク

- **公式ドキュメント**: [ibis-ssl.github.io](https://ibis-ssl.github.io/ibis_documentation/)
- **RoboCup SSL**: [ssl.robocup.org](https://ssl.robocup.org/)
- **grSim シミュレータ**: [github.com/RoboCup-SSL/grSim](https://github.com/RoboCup-SSL/grSim)

## サポート

質問やサポートが必要な場合：
- **Issues**: GitHubのIssuesで報告
- **Discussions**: GitHubのDiscussionsで議論
- **Documentation**: 公式ドキュメントを参照