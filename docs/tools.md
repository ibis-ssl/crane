# 開発ツールと設定

このドキュメントでは、Craneプロジェクトの開発に役立つツールや設定について説明します。

## コード品質ツール

### 1. clang-format

C++コードを自動フォーマットするためのツールです。プロジェクトのルートにある`.clang-format`ファイルに基づいてフォーマットされます。

ROS 2環境では`ament_clang_format`コマンドを使用できます：

```bash
# 特定のファイルやディレクトリをフォーマット
ament_clang_format --reformat <フォーマットしたいファイルかフォルダ>

# 現在のディレクトリ以下をすべてフォーマット
ament_clang_format --reformat .
```

### 2. ruff

Pythonコードのリンターとフォーマッターです。pre-commitフックとして設定されています。

### 3. cpplint

C++コードの静的解析ツールです。Googleのコーディング規約に基づいてコードをチェックします。

## コミット前の自動チェック

### pre-commit

コミット前に自動でコードチェックとフォーマットを行うツールです。下記の手順でインストールして利用できます。

```bash
# インストール
sudo apt install -y python3-venv pipx
pipx install pre-commit

# Craneプロジェクトで設定
cd ~/ibis_ws/src/crane
pre-commit install

# 手動で全ファイルに対して実行（初回実行時は時間がかかります）
pre-commit run -a
```

現在の設定ファイル：
[.pre-commit-config.yaml](https://github.com/ibis-ssl/crane/blob/develop/.pre-commit-config.yaml)

### pre-commitで実行される主なチェック

- **基本チェック**:
  - JSON/TOML/XML/YAMLファイルの構文チェック
  - マージ競合のチェック
  - 秘密鍵の検出
  - ファイル末尾の改行チェック
  - 行末の空白チェック

- **言語固有チェック**:
  - C++: clang-format, cpplint
  - Python: ruff (リンターとフォーマッター)
  - シェルスクリプト: shellcheck, shfmt
  - YAML: yamllint

## SSL関連ツール

### ssl-go-tools

RoboCup SSL関連のGo言語製ツール集です。試合のログ記録やデータ解析に役立ちます。

```bash
# インストール
sudo apt install -y golang-go
git clone https://github.com/RoboCup-SSL/ssl-go-tools.git
cd ssl-go-tools
make all
sudo make install
echo 'export PATH="$(go env GOPATH)/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

詳細は[GitHub リポジトリ](https://github.com/RoboCup-SSL/ssl-go-tools)を参照してください。

#### ssl-auto-recorder

Refereeの信号やビジョンデータを自動で記録するツールです。

```bash
# 基本的な使用方法
ssl-auto-recorder -referee-address "224.5.23.1:11003"

# HTTPサーバーを立ち上げてログを提供
ssl-auto-recorder -http-serve -http-port "8084"
```

主なオプション：

```
-output-folder string       出力フォルダ (default "logs")
-referee-address string     Refereeのマルチキャストアドレス (default "224.5.23.1:10003")
-vision-address string      Visionのマルチキャストアドレス (default "224.5.23.2:10006")
-vision-tracker-address     Vision Trackerのアドレス (default "224.5.23.2:10010")
-http-serve                 HTTPサーバーの有効化 (default true)
-http-port string           HTTPポート (default "8084")
```

#### ssl-match-client

試合の状態を表示するクライアントツールです。

```bash
ssl-match-client -address 224.5.23.1:11003
```

#### ssl-vision-tracker-tool

Vision Tracker情報を表示するためのツールです。

```bash
ssl-vision-tracker-tool -tracker-address 224.5.23.2:10010
```

## VS Code拡張機能

以下のVS Code拡張機能を使用すると開発効率が向上します：

- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) - C/C++のコード補完とデバッグ
- [CMake](https://marketplace.visualstudio.com/items?itemName=twxs.cmake) - CMakeファイルのシンタックスハイライト
- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) - Pythonのコード補完とデバッグ
- [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) - ROS開発のサポート
- [YAML](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml) - YAMLファイルのサポート
- [XML](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml) - XMLファイルのサポート

## コマンドライン便利ツール

### colcon

ROSパッケージのビルドツールです。以下のようなコマンドが便利です：

```bash
# 特定のパッケージのみビルド
colcon build --symlink-install --packages-select crane_robot_skills

# 依存関係のあるパッケージも含めてビルド
colcon build --symlink-install --packages-up-to crane_robot_skills

# 変更のあったパッケージのみビルド
colcon build --symlink-install --packages-select-by-dep --packages-above crane_robot_skills
```

### rqt_graph

実行中のROSノードとトピックの接続関係を視覚化します：

```bash
ros2 run rqt_graph rqt_graph
```
