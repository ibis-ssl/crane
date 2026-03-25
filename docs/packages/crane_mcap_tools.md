# crane_mcap_tools

## 概要

`crane_mcap_tools`は、PythonベースのRosbag/MCAP解析・SVG動画生成ツールセットです。試合記録の分析、SVGビジュアライゼーションのMP4動画変換、MCAPアノテーション解析を提供します。

## 主要機能

### SVG動画生成

MCAPファイルからSVGビジュアライゼーションを抽出してMP4動画を生成するパイプライン。

```text
MCAP → SvgExtractor → SvgAssembler → SvgRenderer → VideoGenerator → MP4
```

**レンダリングバックエンド**:

- `cairosvg` (デフォルト) - Pure Python実装
- `rsvg` (librsvg) - GNOMEライブラリ、高速
- `resvg` - Rust実装、最高品質

### bag_analysis - rosbag解析ライブラリ

crane_bagのPython版。`crane_bag` CLIのPythonバインディングとして機能。

- **reader**: rosbag2_pyによるデータ読み込み
- **events**: イベント検出（ゴール・キック・ファウル等）
- **tracking**: ロボット・ボール位置追跡
- **control**: planning_factors・制御ターゲット分析
- **survey**: 試合概要集計
- **cli**: `crane_bag` Pythonコマンドエントリポイント

### mcap_analysis - MCAPアノテーション解析

- **mcap_tools**: MCAPファイル解析ツール
- **extractor**: アノテーションデータ抽出
- **report_generator**: 分析レポート生成
- **gemini_client**: Gemini AI連携（オプション）

## アーキテクチャ上の役割

**依存レイヤ**: 統合層（Layer 5）- オフライン解析ツール

Python実装のため、ROS 2の実行環境なしで単独実行可能（rosbag2_py依存あり）。

## 依存関係

- `rclpy`, `rosbag2_py`
- `python3-cairosvg`, `python3-yaml`

## 使用方法

### SVG動画生成

```bash
# 基本的な使い方
ros2 run crane_mcap_tools svg_video_generator.py /path/to/rosbag_dir -o output.mp4

# フレームレート・品質指定
ros2 run crane_mcap_tools svg_video_generator.py rosbag_dir -o match.mp4 --fps 60 --crf 18

# 時間範囲指定（ハイライト生成）
ros2 run crane_mcap_tools svg_video_generator.py rosbag_dir -o highlight.mp4 --start-time 120 --end-time 180

# レンダリングバックエンド指定
ros2 run crane_mcap_tools svg_video_generator.py rosbag_dir -o output.mp4 --backend resvg
```

詳細: `crane_mcap_tools/crane_mcap_tools/svg_video/README.md`

### bag_analysis ライブラリ（Python）

```python
from crane_mcap_tools.bag_analysis.reader import BagReader
from crane_mcap_tools.bag_analysis.events import detect_events

reader = BagReader("/path/to/rosbag_dir")
bag_data = reader.read()
events = detect_events(bag_data)
```

## 最近の開発状況

- **2026年2月（PR #1232）**: `crane_debug_tools`から分割・独立パッケージ化
- **2026年1月（PR #1122, #1127）**: SVG動画生成の性能改善（複数バックエンド対応、RAWフレームモード）
- **2026年1月（PR #1103, #1104）**: アノテーション機能実装

## 関連パッケージ

- [crane_bag](./crane_bag.md) - C++版rosbag解析CLIツール
- [crane_web_debugger](./crane_web_debugger.md) - リアルタイムWebデバッグツール
