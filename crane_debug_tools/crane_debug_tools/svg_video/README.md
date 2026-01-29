# SVG Video Generator

MCAP rosbagファイルからSVGメッセージを読み込み、MP4動画を生成するモジュールです。

## 概要

このモジュールは、以下のトピックからSVGビジュアライゼーションを抽出し、動画に変換します：

- `/aggregated_svgs` (SvgSnapshot) - 5秒周期のスナップショット
- `/visualizer_svgs` (SvgUpdates) - 高頻度の増分更新

## アーキテクチャ

```text
MCAP → SvgExtractor → SvgAssembler → SvgRenderer → VideoGenerator → MP4
```

### コンポーネント

1. **SvgExtractor**: MCAPファイルからSVGメッセージを抽出し、増分更新を累積適用
2. **SvgAssembler**: レイヤーを単一のSVGドキュメントに合成
3. **SvgRenderer**: SVGをPNGにラスタライズ（cairosvg使用）
4. **VideoGenerator**: PNGフレームをMP4動画に変換（ffmpeg使用）

## 使用方法

### 基本的な使い方

```bash
ros2 run crane_debug_tools svg_video_generator.py /path/to/rosbag_dir -o output.mp4
```

### オプション

```bash
# フレームレート指定
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4 --fps 60

# 2倍速再生
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4 --speed 2.0

# 特定のレイヤーのみ表示
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4 --layers "layer1,layer2"

# フレームをディレクトリに保存（デバッグ用）
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4 --save-frames frames/

# 時刻範囲を指定
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4 --start-time 10 --end-time 30

# 詳細ログ
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4 -v
```

### 全オプション

```text
使用法: svg_video_generator.py MCAP_PATH [-o OUTPUT] [OPTIONS]

位置引数:
  mcap_path             MCAPファイルまたはrosbag2ディレクトリのパス

オプション:
  -o, --output          出力ファイル（デフォルト: output.mp4）
  --fps                 フレームレート（デフォルト: 30）
  --width               出力幅（デフォルト: 1920）
  --height              出力高さ（デフォルト: 1080）
  --crf                 品質（0-51、低いほど高品質、デフォルト: 23）
  --preset              エンコーディングプリセット（デフォルト: medium）
  --speed               再生速度倍率（デフォルト: 1.0）
  --start-time          開始時刻（秒）
  --end-time            終了時刻（秒）
  --layers              表示レイヤー（カンマ区切り）
  --exclude-layers      除外レイヤー（カンマ区切り）
  --snapshot-topic      スナップショットトピック名（デフォルト: /aggregated_svgs）
  --update-topic        更新トピック名（デフォルト: /visualizer_svgs）
  --save-frames         フレームをディレクトリに保存
  -v, --verbose         詳細ログ
```

## 依存関係

### Python パッケージ

- `cairosvg` - SVGからPNGへの変換

インストール:

```bash
pip install cairosvg
```

### システムパッケージ

- `ffmpeg` - 動画生成（Docker環境にインストール済み）

## 技術仕様

### データフロー

1. **メッセージ抽出**: `/aggregated_svgs`と`/visualizer_svgs`をタイムスタンプ順にマージ
2. **増分更新の適用**: `svg_viewer.js`のロジックを移植
   - `replace`: レイヤー全体を置換
   - `append`: プリミティブを追加
   - `clear`: レイヤーをクリア
3. **SVG合成**: viewBox `-6000 -4500 12000 9000`（SSL field dimensions in mm）
4. **PNG変換**: cairosvgで指定解像度にラスタライズ
5. **動画生成**: ffmpegでMP4にエンコード

### メモリ効率

- **ストリーミングモード**: PNGフレームをffmpegにパイプ（低メモリ使用量）
- **フレーム保存モード**: デバッグ用（`--save-frames`オプション）

### フレームレート制御

- メッセージ間隔より高いフレームレートの場合は前フレームを複製
- `--speed`オプションで再生速度を調整可能

## トラブルシューティング

### cairosvgのインポートエラー

```bash
pip install cairosvg
```

Dockerコンテナ内の場合、libcairo2が必要な場合があります：

```bash
apt-get update && apt-get install -y libcairo2
```

### ffmpegが見つからない

```bash
apt-get update && apt-get install -y ffmpeg
```

### メモリ不足

フレーム保存モードを使用してストリーミングモードに切り替え：

```bash
# --save-framesオプションを削除
ros2 run crane_debug_tools svg_video_generator.py rosbag_dir -o output.mp4
```

## 例

### 試合全体を動画化

```bash
ros2 run crane_debug_tools svg_video_generator.py \
  ~/rosbags/match_2024_01_15/ \
  -o match.mp4 \
  --fps 30
```

### ハイライトシーンを高画質で

```bash
ros2 run crane_debug_tools svg_video_generator.py \
  ~/rosbags/match_2024_01_15/ \
  -o highlight.mp4 \
  --start-time 120 \
  --end-time 180 \
  --fps 60 \
  --crf 18 \
  --preset slow
```

### デバッグ用フレーム確認

```bash
ros2 run crane_debug_tools svg_video_generator.py \
  ~/rosbags/test/ \
  -o test.mp4 \
  --save-frames debug_frames/ \
  --start-time 0 \
  --end-time 5 \
  -v
```
