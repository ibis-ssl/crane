# Crane DevContainer 環境

このディレクトリには、Crane プロジェクトの最適化された DevContainer 設定が含まれています。VS Code の Remote - Containers 拡張機能を使用して、一貫性のある開発環境を構築できます。

## 主な機能

### 🚀 自動セットアップ

- ROS 2 Jazzy 環境の自動構築
- 依存パッケージの自動インストール
- ワークスペースの自動ビルド
- pre-commit フックの自動インストール

### 🛠️ 開発ツール完備

- **C++ 開発**: clang-format, clang-tidy, gdb, valgrind
- **Python 開発**: ruff, pytest, pre-commit
- **ROS 2 ツール**: colcon, vcstool, rosdep
- **VS Code 拡張機能**: C++, ROS, Python, GitLens など

### 🎮 シミュレーション環境統合

- grSim (ロボットサッカーシミュレータ)
- SSL Game Controller (審判システム)
- SSL Vision Client (ビジョン確認)
- SSL Status Board (状態表示)

### ⚡ ビルド最適化

- 並列ビルド設定済み
- symlink-install によるビルド時間短縮
- VS Code タスクでワンクリックビルド

## クイックスタート

### GitHub Codespaces で始める（推奨）⚡

**起動時間: 30秒〜2分**

GitHub Codespaces を使用すると、ブラウザだけで即座に開発を開始できます。

1. **リポジトリページで Code ボタンをクリック**
2. **「Codespaces」タブを選択**
3. **「Create codespace on [ブランチ名]」をクリック**

初回セットアップは自動実行され、2〜3分で完了します。
ビルドは必要なときに手動で実行するため、起動時間を大幅に短縮しています。

```bash
# Codespaces 起動後、必要に応じてビルド
colcon build --symlink-install
source install/local_setup.bash
```

**Codespaces のメリット:**

- ✅ ローカル環境不要（ブラウザだけでOK）
- ✅ 高速起動（Prebuild により30秒〜2分）
- ✅ どこからでもアクセス可能
- ✅ チーム全員が同じ環境を使用

---

### ローカル環境で始める

**起動時間: 初回10〜15分、2回目以降3〜5分**

#### 1. 前提条件

- Docker Desktop または Docker Engine
- VS Code
- Remote - Containers 拡張機能

#### 2. コンテナの起動

VS Code でこのリポジトリを開き、以下のいずれかの方法でコンテナを起動します：

**方法 A: コマンドパレット**

1. `F1` または `Ctrl+Shift+P` でコマンドパレットを開く
2. `Remote-Containers: Reopen in Container` を選択

**方法 B: 通知からの起動**

1. リポジトリを開くと表示される通知から「Reopen in Container」をクリック

#### 3. 初回セットアップ（自動）

**ローカル環境のみ**、コンテナ起動時に以下が自動実行されます：

1. 依存パッケージのインポート (`vcs import`)
2. システム依存関係のインストール (`rosdep install`)
3. ワークスペースのビルド (`colcon build`)
4. pre-commit フックのインストール

初回セットアップには約 10〜15 分かかります。

#### 4. 動作確認

ターミナルで以下を実行してシミュレーションを起動：

```bash
ros2 launch crane_bringup crane.launch.py sim:=true
```

Web ブラウザで以下にアクセス：

- **Game Controller**: <http://localhost:8081>
- **Vision Client**: <http://localhost:8082>
- **Status Board**: <http://localhost:8083>

## ディレクトリ構成

```
ibis_ws/                      # ワークスペースルート (workspaceFolder)
├── src/
│   └── crane/                # Crane リポジトリ
│       └── .devcontainer/    # DevContainer 設定
│           ├── devcontainer.json   # メイン設定
│           ├── Dockerfile          # コンテナイメージ定義
│           ├── docker-compose.yaml # サービス構成
│           ├── postCreate.sh       # 初期セットアップスクリプト
│           └── README.md           # このファイル
├── .vscode/                  # VS Code ワークスペース設定
│   ├── tasks.json            # ビルド・テストタスク
│   ├── launch.json           # デバッグ設定
│   └── settings.json         # エディタ設定
├── build/                    # ビルド成果物（自動生成）
├── install/                  # インストールファイル（自動生成）
└── log/                      # ビルドログ（自動生成）
```

## VS Code タスク

`Ctrl+Shift+B` でビルドタスクメニューを開けます。

### ビルドタスク

- **colcon: build** (デフォルト) - 標準ビルド
- **colcon: build (optimized)** - 最適化スクリプトでビルド
- **colcon: build (current package)** - 指定パッケージのみビルド
- **colcon: clean** - ビルド成果物をクリーン

### テストタスク

- **colcon: test** - すべてのテスト実行
- **colcon: test (current package)** - 指定パッケージのテスト実行
- **pre-commit: run all** - すべての pre-commit チェック実行

### 起動タスク

- **ros2: launch crane simulation** - シミュレーション起動
- **ros2: launch data pipeline** - データパイプライン起動

## デバッグ

`F5` でデバッグメニューを開けます。

### C++ ノードのデバッグ

1. **ROS: C++ Launch Node** を選択
2. パッケージ名とノード名を入力
3. ブレークポイントを設定して実行

### Python ノードのデバッグ

1. デバッグしたい Python ファイルを開く
2. **Python: ROS Node** を選択
3. ブレークポイントを設定して実行

## よくあるコマンド

### ビルド

```bash
# 標準ビルド
colcon build --symlink-install

# 最適化ビルド
./src/crane/scripts/optimized_build.bash

# クリーンビルド
./src/crane/scripts/optimized_build.bash clean

# 特定パッケージのみ
colcon build --packages-select crane_world_model_publisher
```

### テスト

```bash
# すべてのテスト
colcon test --event-handlers console_cohesion+

# 特定パッケージのテスト
colcon test --packages-select crane_physics --event-handlers console_cohesion+
```

### ROS 2 起動

```bash
# シミュレーション
ros2 launch crane_bringup crane.launch.py sim:=true

# データパイプライン
ros2 launch crane_bringup data.launch.py
```

### 開発ツール

```bash
# pre-commit チェック
cd src/crane
pre-commit run --all-files

# フォーマット
clang-format -i src/my_file.cpp

# Lint
cpplint src/my_file.cpp
```

## トラブルシューティング

### ビルドエラー「依存関係が見つからない」

```bash
# rosdep を再実行
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro jazzy
```

### コンテナが起動しない

1. Docker が起動していることを確認
2. Docker Desktop のリソース設定を確認（推奨: メモリ 8GB以上）
3. コンテナを再ビルド: `Remote-Containers: Rebuild Container`

### シミュレーションが表示されない

```bash
# grSim コンテナの状態確認
docker ps | grep grsim

# コンテナの再起動
docker compose -f src/crane/.devcontainer/docker-compose.yaml restart grsim
```

### ポートフォワーディングが機能しない

VS Code の「ポート」タブで、以下のポートが転送されていることを確認：

- 8081 (Game Controller)
- 8082 (Vision Client)
- 8083 (Status Board)

手動でポート追加も可能：「ポートの転送」→ ポート番号を入力

### Git 認証が機能しない

Git 設定とSSHキーは、ホストマシンから自動的にマウントされます。
ホストマシンで `~/.gitconfig` と `~/.ssh` が正しく設定されていることを確認してください。

## カスタマイズ

### 追加の VS Code 拡張機能

`.devcontainer/devcontainer.json` の `extensions` セクションに追加：

```json
"extensions": [
  "existing.extension",
  "your.new-extension"
]
```

### 追加のパッケージインストール

`.devcontainer/Dockerfile` に追加：

```dockerfile
RUN apt-get update && apt-get install -y \
    your-package \
    && rm -rf /var/lib/apt/lists/*
```

### 環境変数の設定

`.devcontainer/docker-compose.yaml` の `environment` セクションに追加：

```yaml
environment:
  - YOUR_VAR=value
```

## GitHub Codespaces 最適化

### 環境の違い

| 項目 | Codespaces | ローカル |
|------|-----------|---------|
| **起動時間** | 30秒〜2分 | 10〜15分 |
| **セットアップ** | 依存関係のみ | 完全ビルド |
| **シミュレータ** | なし（軽量化） | grSim等統合 |
| **ベースイメージ** | jazzy-desktop | jazzy-desktop-full |

### Prebuild の設定

リポジトリ管理者向け：

1. **Settings > Codespaces** を開く
2. **「Set up prebuild」** をクリック
3. ブランチを選択（develop, main など）
4. **「Create」** をクリック

Prebuild を有効にすると、`.github/workflows/codespaces-prebuild.yml` が自動実行され、
事前ビルド済みの環境が準備されます。

### Codespaces でのワークフロー

```bash
# 1. Codespaces 起動（30秒〜2分）
#    依存関係は自動インストール済み

# 2. 必要に応じてビルド
cb  # エイリアス: colcon build --symlink-install

# 3. 環境読み込み
cs  # エイリアス: source install/local_setup.bash

# 4. 開発開始！
```

### 便利なエイリアス

コンテナ内で以下のエイリアスが使えます：

```bash
cb   # colcon build --symlink-install
ct   # colcon test --event-handlers console_cohesion+
cs   # source install/local_setup.bash
```

## パフォーマンスチューニング

### Codespaces マシンタイプ

起動時に選択可能：

- **2-core, 8GB RAM**: 軽量開発用
- **4-core, 16GB RAM**: 標準開発用（推奨）
- **8-core, 32GB RAM**: 大規模ビルド・テスト用

### Docker リソース設定（ローカル環境）

- **CPU**: 4コア以上
- **メモリ**: 8GB以上
- **ディスク**: 50GB以上

### ビルド高速化

```bash
# 並列数を指定
colcon build --parallel-workers 8

# ccache を有効化（既にインストール済み）
export CMAKE_CXX_COMPILER_LAUNCHER=ccache
```

## 詳細情報

- [Crane プロジェクト概要](../README.md)
- [開発ガイド](../AGENTS.md)
- [ROS 2 Jazzy ドキュメント](https://docs.ros.org/en/jazzy/)
- [VS Code Remote Development](https://code.visualstudio.com/docs/remote/remote-overview)

## ライセンス

このプロジェクトは MIT ライセンスの下で公開されています。
