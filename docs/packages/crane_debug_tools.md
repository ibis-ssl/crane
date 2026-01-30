# crane_debug_tools

## 概要

`crane_debug_tools`は、Craneロボットサッカーシステムの開発とデバッグを支援する最新のツールセットです。スキルのテスト実行、シナリオベースのテスト、Webベースの可視化インターフェースを提供します。

## 主要機能

### Webインターフェース

- **WebSocketサーバー** (`websocket_server.cpp`): リアルタイム双方向通信
- **Webブリッジサーバー** (`web_bridge_server.cpp`): ROS 2とWebの橋渡し
- **Webサーバー** (`simple_web_server.cpp`): HTTPサーバーとUI配信
- **WebUI統合ポータル** (PR #1124): 各種デバッグツールへの統一アクセス

### SVG動画生成機能

MCAP rosbagからSVGビジュアライゼーションを抽出し、MP4動画を生成する高性能パイプライン（PR #1122, #1127）。

**アーキテクチャ**:

```text
MCAP → SvgExtractor → SvgAssembler → SvgRenderer → VideoGenerator → MP4
```

**主要コンポーネント**:

- **SvgExtractor**: MCAPファイルからSVGメッセージ抽出、増分更新の累積適用
- **SvgAssembler**: レイヤーを単一SVGドキュメントに合成
- **SvgRenderer**: SVGをPNGにラスタライズ（複数バックエンド対応）
- **VideoGenerator**: PNGフレームをMP4動画に変換（ffmpeg）

**レンダリングバックエンド**:

- `cairosvg` (デフォルト) - Pure Python実装
- `rsvg` (librsvg) - GNOMEライブラリ、高速
- `resvg` - Rust実装、最高品質

詳細: `crane_debug_tools/crane_debug_tools/svg_video/README.md`

### アノテーション解析ツール

- **MCAPアノテーション解析** (PR #1104): 記録されたアノテーションの統計分析と抽出
- **リアルタイムアノテーションツール** (PR #1103): 試合中のリアルタイム観察記録（HumanAnnotationメッセージ連携）

## アーキテクチャ上の役割

**依存レイヤ**: 統合層（Layer 5）- 開発ツール

- 開発・デバッグフェーズでの生産性向上を目的とした補助ツール

## コンポーネント

### ノード

#### debug_web_server

Web UIとROS 2を接続するブリッジノード。

**機能**:

- WebSocket経由のリアルタイムデータ配信
- ブラウザからのスキル実行制御
- 可視化データのストリーミング

## 依存関係

### ビルド依存

- `ament_cmake_auto`
- `crane_msg_wrappers`
- `crane_msgs`
- `crane_robot_skills`
- `crane_visualization_interfaces`
- `rclcpp`, `rclcpp_action`
- `nlohmann-json-dev`
- `libboost-system-dev`, `libssl-dev`, `libgoogle-glog-dev`

## 使用方法

### Webインターフェース

```bash
# Webサーバー有効化で起動
ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=true

# ブラウザでアクセス
http://localhost:8080/standalone.html
```

### SVG動画生成

```bash
# 基本的な使い方
ros2 run crane_debug_tools svg_video_generator.py /path/to/rosbag_dir -o output.mp4

# フレームレート・品質指定
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o match.mp4 --fps 60 --crf 18

# 時間範囲指定（ハイライト生成など）
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o highlight.mp4 --start-time 120 --end-time 180

# レンダリングバックエンド指定
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4 --backend resvg
```

詳細オプション: `crane_debug_tools/crane_debug_tools/svg_video/README.md`

## 最近の開発状況

- **2026年1月（PR #1122, #1127）**: SVG動画生成機能の抜本的性能改善
  - 複数レンダリングバックエンド対応（cairosvg/rsvg/resvg）
  - RAWフレームモード追加による高速化
  - ストリーミングパイプライン最適化
- **2026年1月（PR #1103, #1104）**: アノテーション機能実装
  - リアルタイムアノテーションツール
  - MCAP記録データからの解析・統計機能
- **2026年1月（PR #1124）**: WebUI統合ポータル構築
- **2025年前半**: WebSocket/HTTP統合によるリアルタイム可視化強化
- **2025年JapanOpen**: 大会時のデバッグツールとして実戦運用
- **設計方針**: 開発効率向上のための実用的なツール群として継続拡充

## 関連パッケージ

- [crane_robot_skills](./crane_robot_skills.md) - テスト対象のスキルライブラリ
- [crane_msgs](./crane_msgs.md) - SkillExecutionアクションメッセージ
- [crane_session_coordinator](./crane_session_coordinator.md) - スキル実行サーバー

## トラブルシューティング

### アクションサーバーに接続できない

**原因**: Craneシステムが起動していない

**解決策**:

```bash
ros2 launch crane_bringup crane.launch.xml sim:=true
```

### コマンドが見つからない

**原因**: 環境変数が読み込まれていない

**解決策**:

```bash
source install/local_setup.bash
```

### パラメータエラー

**原因**: 不正なパラメータ形式

**正しい形式**: `key:value` （`key=value` ではない）

## 備考

- 本パッケージは開発・テスト専用であり、実際の試合では使用しない
- WebインターフェースはデバッグUIとして便利だが、パフォーマンスへの影響に注意
- シナリオテストは回帰テストの自動化に有効
