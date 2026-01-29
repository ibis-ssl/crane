# SVG Video Generator インストール手順

## 依存関係のインストール

SVG動画生成機能を使用するには、以下の依存関係が必要です。

### 1. Python パッケージ

cairosvgをインストール:

```bash
pip install cairosvg
```

または、プロジェクトのsetup.pyから自動インストール:

```bash
cd /home/hans/workspace/ibis_ws/src/crane/crane_debug_tools
pip install -e .
```

### 2. システムパッケージ

cairosvgはlibcairo2に依存します。Dockerコンテナ内では既にインストールされているはずですが、必要に応じて以下を実行:

```bash
sudo apt-get update
sudo apt-get install -y libcairo2 libcairo2-dev
```

ffmpegも必要です（通常はDocker環境にインストール済み）:

```bash
sudo apt-get install -y ffmpeg
```

### 3. インストール確認

```bash
# cairosvgのインポート確認
python3 -c "import cairosvg; print(f'cairosvg version: {cairosvg.__version__}')"

# ffmpegの確認
ffmpeg -version
```

## Docker環境での設定

Dockerfileに以下を追加することで、コンテナビルド時に依存関係を自動インストール可能:

```dockerfile
# システムパッケージ
RUN apt-get update && apt-get install -y \
    libcairo2 \
    libcairo2-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Pythonパッケージ
RUN pip install cairosvg
```

## トラブルシューティング

### ImportError: No module named 'cairosvg'

```bash
pip install cairosvg
```

### cairo.h: No such file or directory

```bash
sudo apt-get install libcairo2-dev
```

### ffmpeg: command not found

```bash
sudo apt-get install ffmpeg
```

### メモリ不足エラー

大きなrosbagファイルを処理する場合、メモリ不足になる可能性があります。
その場合は`--save-frames`オプションを使用せず、ストリーミングモードで実行してください。

## 使用例

インストール後、以下のコマンドで動画を生成できます:

```bash
cd /home/hans/workspace/ibis_ws
source install/setup.bash

# 基本的な使い方
ros2 run crane_debug_tools svg_video_generator.py /path/to/rosbag -o output.mp4

# 詳細はヘルプを参照
ros2 run crane_debug_tools svg_video_generator.py --help
```
