# SVG Video Generator アーキテクチャ

## 概要

rosbag (MCAP形式) から `/aggregated_svgs`（スナップショット）と `/visualizer_svgs`（増分更新）トピックを読み込み、各フレームのSVGをPNGにラスタライズし、ffmpegで動画（MP4）を生成するシステム。

## システム構成

```text
┌─────────────────────────────────────────────────────────────────┐
│                         MCAP File                                │
│  ┌──────────────────────┐  ┌──────────────────────┐            │
│  │ /aggregated_svgs     │  │ /visualizer_svgs     │            │
│  │ (SvgSnapshot, 5秒)   │  │ (SvgUpdates, 高頻度) │            │
│  └──────────────────────┘  └──────────────────────┘            │
└────────────┬────────────────────────────┬─────────────────────┘
             │                            │
             └────────────┬───────────────┘
                          ↓
             ┌────────────────────────┐
             │   SvgExtractor         │
             │  - rosbag2_py reader   │
             │  - タイムスタンプマージ │
             │  - 増分更新累積適用     │
             └────────────┬───────────┘
                          ↓
              SvgFrame (timestamp, layers)
                          ↓
             ┌────────────────────────┐
             │   SvgAssembler         │
             │  - viewBox設定         │
             │  - グリッド追加         │
             │  - レイヤー合成         │
             └────────────┬───────────┘
                          ↓
                  SVG String
                          ↓
             ┌────────────────────────┐
             │   SvgRenderer          │
             │  - cairosvg            │
             │  - 1920x1080 PNG       │
             └────────────┬───────────┘
                          ↓
                  PNG Bytes
                          ↓
             ┌────────────────────────┐
             │   VideoGenerator       │
             │  - ffmpeg pipe         │
             │  - libx264 encoding    │
             └────────────┬───────────┘
                          ↓
                      MP4 Video
```

## コンポーネント詳細

### 1. SvgExtractor

**役割**: MCAPファイルからSVGメッセージを抽出し、増分更新を累積適用

**入力**:

- MCAP rosbag ファイル
- トピック名（デフォルト: `/aggregated_svgs`, `/visualizer_svgs`）
- 時刻範囲（オプション）

**処理**:

1. `rosbag2_py.SequentialReader`でMCAPを読み込み
2. スナップショットと増分更新を収集
3. タイムスタンプ順にソート
4. 増分更新操作を適用:
   - `replace`: レイヤー全体を置換
   - `append`: プリミティブを追加（ベースレイヤーがある場合のみ）
   - `clear`: レイヤーをクリア
5. epoch変更時に状態をリセット

**出力**: `SvgFrame` ジェネレータ（timestamp, epoch, seq, layers）

**実装**: `svg_extractor.py`

**参考**: `extractor.py`のMCAP読み込みパターンを踏襲

### 2. SvgAssembler

**役割**: 複数のレイヤーを単一のSVGドキュメントに合成

**入力**:

- レイヤーマップ（`dict[str, list[str]]`）
- 表示レイヤーセット（オプション）

**処理**:

1. SVGヘッダー生成（viewBox: `-6000 -4500 12000 9000`）
2. グリッドパターン定義（1000mm間隔）
3. 背景レクト（`#6c757d`）
4. グリッドレクト（不透明度0.3）
5. 各レイヤーを`<g>`でグループ化
6. SVGプリミティブを配置

**出力**: 完全なSVGドキュメント文字列

**実装**: `svg_assembler.py`

**参考**: `svg_viewer.js`の`updateSvgDisplay()`ロジックを移植

### 3. SvgRenderer

**役割**: SVG文字列をPNGバイト列にラスタライズ

**入力**:

- SVGドキュメント文字列
- 出力サイズ（デフォルト: 1920x1080）
- DPI（デフォルト: 96）

**処理**:

- `cairosvg.svg2png()`で変換

**出力**: PNGバイト列

**実装**: `svg_renderer.py`

**依存**: cairosvg（pip install cairosvg）

### 4. VideoGenerator

**役割**: PNGフレームストリームをMP4動画に変換

**入力**:

- PNGバイト列イテレータ
- 出力ファイルパス
- エンコード設定（fps, codec, crf, preset）

**処理**:

1. ffmpegプロセス起動（`image2pipe`入力、`libx264`出力）
2. PNGバイトをstdinにストリーミング
3. プロセス完了を待機

**出力**: MP4動画ファイル

**実装**: `video_generator.py`

**依存**: ffmpeg

**代替モード**: ディレクトリ内のPNGファイルから生成（`generate_from_directory()`）

## データフロー詳細

### メッセージ処理フロー

```text
時刻 T0: SvgSnapshot { layers: [A, B] }
         → 状態: A=snapshot, B=snapshot

時刻 T1: SvgUpdates { updates: [C.replace] }
         → 状態: A=snapshot, B=snapshot, C=new

時刻 T2: SvgUpdates { updates: [A.append, D.replace] }
         → 状態: A=snapshot+append, B=snapshot, C=new, D=new

時刻 T3: SvgUpdates { updates: [B.clear] }
         → 状態: A=snapshot+append, B=[], C=new, D=new

時刻 T4: SvgSnapshot { layers: [X, Y] } (epoch変更)
         → 状態: X=snapshot, Y=snapshot （A,B,C,Dはリセット）
```

### フレームレート制御

```python
frame_duration = 1秒 / fps / speed

# メッセージ間隔 > frame_durationの場合
# → 前フレームを複製して補間

# メッセージ間隔 < frame_durationの場合
# → フレームをスキップ（間引き）
```

### メモリ効率

**ストリーミングモード**（デフォルト）:

- PNGバイトをffmpegにパイプ
- メモリ使用量: 1フレーム分のPNG（約10MB）

**フレーム保存モード**（`--save-frames`）:

- 全フレームをディスクに保存
- メモリ使用量: 1フレーム分のPNG + ディスク容量
- デバッグ用途

## 技術選定の理由

| 要素 | 選定 | 理由 |
|------|------|------|
| MCAP読み込み | rosbag2_py | ROS 2標準、既存パターン踏襲 |
| SVG→PNG | cairosvg | 高品質、純Python、cairo利用可能 |
| 動画生成 | ffmpeg | Docker環境インストール済み、ストリーミング対応 |
| パイプ入力 | subprocess.Popen | メモリ効率、リアルタイム処理 |

## パフォーマンス特性

### 処理時間の目安

- MCAP読み込み: O(N) N=メッセージ数
- SVG組み立て: O(M) M=レイヤー数
- PNG変換: O(1) per frame（解像度に依存）
- 動画エンコード: O(F) F=フレーム数

**例**:

- 10分間の録画（30fps）
- 18,000フレーム
- 処理時間: 約10-30分（マシンスペックに依存）

### メモリ使用量

- ストリーミングモード: 約50MB（1フレーム分）
- フレーム保存モード: 約50MB + (10MB × フレーム数)

## 拡張性

### 追加可能な機能

1. **複数トピック対応**: 他のSVGトピックも処理可能
2. **カスタムレンダリング**: viewBox、背景色のカスタマイズ
3. **リアルタイム処理**: rosbagの再生と同期して動画生成
4. **並列処理**: マルチプロセスでPNG変換を高速化
5. **GPU加速**: cairoのバックエンドをGPU対応に変更

## 参考実装

- `crane_debug_tools/mcap_analysis/extractor.py`: MCAP読み込みパターン
- `crane_debug_tools/web/svg_viewer.js`: SVG組み立て・増分更新ロジック
- ROS 2 rosbag2_py: MCAP読み込みAPI
- cairosvg: SVG→PNG変換
- ffmpeg: 動画エンコード
