# SVG動画生成

MCAPファイルまたはrosbag2ディレクトリのSVGメッセージからMP4を生成する。
`/aggregated_svgs` のスナップショットと `/visualizer_svgs` の増分更新を復元して描画するため、録画にはこれらのトピックを含める。

## 利用

ROS 2とビルド済みワークスペースの環境を読み込む。実行環境には `rosbag2_py`、SVGメッセージ型、ffmpeg、描画バックエンドが必要。
標準のCairoSVGにはlibcairo2が必要で、Python依存と追加バックエンドは[setup.py](https://github.com/ibis-ssl/crane/blob/develop/crane_mcap_tools/setup.py)を参照する。

```bash
source install/setup.bash
ros2 run crane_mcap_tools svg_video_generator.py /path/to/rosbag -o output.mp4
ros2 run crane_mcap_tools svg_video_generator.py --help
ros2 run crane_mcap_tools svg_video_generator.py --list-backends
```

レイヤー選択、画質、再生速度などはヘルプを参照する。`--save-frames` は描画確認用のPNGを保存するため、出力先の空き容量を確認する。

## 制約と実装

- 更新の適用とepoch変更時の扱いは[抽出処理](https://github.com/ibis-ssl/crane/blob/develop/crane_mcap_tools/crane_mcap_tools/svg_video/svg_extractor.py)を参照する。
- 時刻範囲指定はヘルプに「bag先頭から」とあるが、現実装は記録タイムスタンプに直接比較する。短い相対秒の例をそのまま使わず、記録時刻を確認する。範囲より前の状態も復元されないため、途中から切り出すとレイヤーが不足する場合がある。
- ffmpegへのフレーム出力はパイプを使うが、SVGメッセージとフレーム状態はメモリに収集する。長い録画の一定メモリ動作は保証しない。
- 描画・並列化・動画出力は[CLI](https://github.com/ibis-ssl/crane/blob/develop/crane_mcap_tools/scripts/svg_video_generator.py)と[モジュール](https://github.com/ibis-ssl/crane/tree/develop/crane_mcap_tools/crane_mcap_tools/svg_video)を参照する。
